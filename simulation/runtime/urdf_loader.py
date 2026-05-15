"""URDF and mesh prep shared by PyBullet and MuJoCo backends.

Converts fragile ROS meshes (binary STL to OBJ for PyBullet; DAE to OBJ for
MuJoCo), resolves package:// URIs to absolute paths, and injects the hidden
simlab frame both engines use. Mimic tags are ignored by the engines, so
parallelogram pairs get explicit gear/equality constraints via helper
functions here.

Entry points: prepare_urdf_for_pybullet, prepare_urdf_for_mujoco,
apply_parallelogram_constraint_pybullet, inject_equality_constraint_mjcf.
Outputs are cached under DOBOT_SIM_CACHE (see simulation README) using a hash
of the source URDF and referenced mesh mtimes/sizes.
"""

from __future__ import annotations

import hashlib
import math
import os
import shutil
import xml.etree.ElementTree as ET
from dataclasses import dataclass
from pathlib import Path
from typing import Iterable, Mapping

_STL_EXT = {".stl"}
_DAE_EXT = {".dae"}
_OBJ_EXT = {".obj"}


# ---------------------------------------------------------------------------
# URI resolution
# ---------------------------------------------------------------------------

def _default_package_roots(urdf_path: Path) -> dict[str, Path]:
    """Infer package roots by scanning the URDF for `package://<name>` URIs.

    Assumes the common ROS layout `<pkg_root>/urdf/<name>.urdf`, so the
    grandparent directory is the package root. Every `package://<name>`
    URI found in the URDF is aliased to that same root — this handles the
    common case where a vendored directory is renamed (e.g. the Magician
    URDF references `package://magician_description/...` but we vendor it
    as `vendor/magician_ros2_urdf/`).
    """
    pkg_dir = urdf_path.parent.parent.resolve()
    roots: dict[str, Path] = {pkg_dir.name: pkg_dir}
    try:
        text = urdf_path.read_text()
    except OSError:
        return roots
    import re
    for match in re.finditer(r"package://([^/\"'<>\s]+)/", text):
        roots.setdefault(match.group(1), pkg_dir)
    return roots


def _resolve_mesh_uri(uri: str, urdf_path: Path, package_roots: Mapping[str, Path]) -> Path:
    """Resolve a URDF mesh `filename` URI to an absolute filesystem path."""
    if uri.startswith("package://"):
        stripped = uri[len("package://"):]
        pkg_name, _, rel = stripped.partition("/")
        if pkg_name not in package_roots:
            raise FileNotFoundError(
                f"Unknown ROS package '{pkg_name}' in URI '{uri}'. "
                f"Known roots: {sorted(package_roots)}"
            )
        return (package_roots[pkg_name] / rel).resolve()
    if uri.startswith("file://"):
        return Path(uri[len("file://"):]).resolve()
    path = Path(uri)
    if path.is_absolute():
        return path.resolve()
    return (urdf_path.parent / path).resolve()


# ---------------------------------------------------------------------------
# Mesh conversion
# ---------------------------------------------------------------------------

def _is_git_lfs_pointer(path: Path) -> bool:
    """Detect a Git LFS pointer stub (e.g. Magician repo has LFS-stub DAE files)."""
    try:
        head = path.read_bytes()[:64]
    except OSError:
        return False
    return head.startswith(b"version https://git-lfs.github.com")


def _resolve_usable_source(src: Path) -> Path:
    """If src is a Git LFS stub, fall back to a sibling STL/OBJ of the same stem."""
    if not _is_git_lfs_pointer(src):
        return src
    stem = src.stem
    for ext in (".STL", ".stl", ".obj", ".OBJ"):
        sibling = src.parent / f"{stem}{ext}"
        if sibling.exists() and not _is_git_lfs_pointer(sibling):
            return sibling
    raise FileNotFoundError(
        f"Mesh {src.name} is a Git LFS pointer and no usable sibling "
        f"(.STL/.obj) was found in {src.parent}"
    )


def _convert_mesh(src: Path, dst: Path) -> None:
    """Convert src -> dst using trimesh. Output format is determined by dst suffix."""
    try:
        import trimesh
    except ImportError as exc:
        raise ImportError(
            "URDF preprocessing requires trimesh. "
            "Install with: pip install trimesh"
        ) from exc
    dst.parent.mkdir(parents=True, exist_ok=True)
    mesh = trimesh.load(src, force="mesh", process=False)
    if not hasattr(mesh, "export"):
        raise RuntimeError(f"trimesh could not load {src}")
    mesh.export(dst)


# ---------------------------------------------------------------------------
# URDF parsing helpers
# ---------------------------------------------------------------------------

def _sanitise_urdf_for_mujoco(tree: ET.ElementTree) -> None:
    """Strip URDF constructs MuJoCo's URDF parser does not accept.

    MuJoCo rejects top-level `<material>` definitions (only inline `<material>`
    elements inside `<visual>` with a child `<color>` are legal). ROS URDFs
    commonly declare reusable materials at the root and reference them by
    name inside each `<visual>`. We strip both the root defs and the
    name-only references — MuJoCo falls back to its default grey material.
    """
    root = tree.getroot()
    for mat in list(root.findall("material")):
        root.remove(mat)
    for visual in root.iter("visual"):
        for mat in list(visual.findall("material")):
            # Keep only if it has an inline <color> or <texture> child.
            if mat.find("color") is None and mat.find("texture") is None:
                visual.remove(mat)


def _iter_mesh_elements(tree: ET.ElementTree) -> Iterable[ET.Element]:
    """Yield every <mesh filename=...> element under <visual>/<collision>."""
    for mesh in tree.getroot().iter("mesh"):
        if "filename" in mesh.attrib:
            yield mesh


def _append_hidden_link(root: ET.Element, name: str) -> None:
    """Append a kinematics-only link with tiny inertia for MuJoCo compatibility."""
    link = ET.SubElement(root, "link", {"name": name})
    _ensure_link_inertial(link)


def _ensure_link_inertial(link: ET.Element) -> None:
    """Attach a tiny inertial block when a link does not define one."""
    if link.find("inertial") is not None:
        return
    inertial = ET.SubElement(link, "inertial")
    ET.SubElement(inertial, "origin", {"xyz": "0 0 0", "rpy": "0 0 0"})
    ET.SubElement(inertial, "mass", {"value": "1e-6"})
    ET.SubElement(
        inertial,
        "inertia",
        {
            "ixx": "1e-9",
            "ixy": "0",
            "ixz": "0",
            "iyy": "1e-9",
            "iyz": "0",
            "izz": "1e-9",
        },
    )


def _ensure_inertials(tree: ET.ElementTree) -> None:
    """PyBullet warns loudly for links with no inertial; fill in tiny defaults."""
    root = tree.getroot()
    for link in root.findall("link"):
        _ensure_link_inertial(link)


def _append_joint(
    root: ET.Element,
    *,
    name: str,
    joint_type: str,
    parent: str,
    child: str,
    xyz: str,
    rpy: str = "0 0 0",
    axis: str | None = None,
) -> None:
    joint = ET.SubElement(root, "joint", {"name": name, "type": joint_type})
    ET.SubElement(joint, "origin", {"xyz": xyz, "rpy": rpy})
    ET.SubElement(joint, "parent", {"link": parent})
    ET.SubElement(joint, "child", {"link": child})
    if axis is not None:
        ET.SubElement(joint, "axis", {"xyz": axis})
    if joint_type != "fixed":
        ET.SubElement(
            joint,
            "limit",
            {
                "effort": "1.0",
                "lower": "-6.283185307179586",
                "upper": "6.283185307179586",
                "velocity": "10.0",
            },
        )


def _remove_children(elem: ET.Element, tag: str) -> None:
    for child in list(elem.findall(tag)):
        elem.remove(child)


def _remove_link_geometries(root: ET.Element, link_name: str) -> None:
    link = root.find(f"./link[@name='{link_name}']")
    if link is None:
        return
    _remove_children(link, "visual")
    _remove_children(link, "collision")


def _replace_link_collision_with_cylinder(
    root: ET.Element,
    link_name: str,
    *,
    radius: float,
    length: float,
    xyz: str,
) -> None:
    link = root.find(f"./link[@name='{link_name}']")
    if link is None:
        return
    _remove_children(link, "collision")
    collision = ET.SubElement(link, "collision")
    ET.SubElement(collision, "origin", {"xyz": xyz, "rpy": "0 0 0"})
    geometry = ET.SubElement(collision, "geometry")
    ET.SubElement(geometry, "cylinder", {"radius": f"{radius:g}", "length": f"{length:g}"})


def _replace_link_collision_with_box(
    root: ET.Element,
    link_name: str,
    *,
    size_xyz: tuple[float, float, float],
    xyz: str = "0 0 0",
    rpy: str = "0 0 0",
) -> None:
    """Replace all collision entries on ``link_name`` with one oriented box."""
    link = root.find(f"./link[@name='{link_name}']")
    if link is None:
        return
    _remove_children(link, "collision")
    collision = ET.SubElement(link, "collision")
    ET.SubElement(collision, "origin", {"xyz": xyz, "rpy": rpy})
    geometry = ET.SubElement(collision, "geometry")
    sx, sy, sz = size_xyz
    ET.SubElement(geometry, "box", {"size": f"{sx:g} {sy:g} {sz:g}"})


def _simplify_magician_mujoco_collisions(tree: ET.ElementTree) -> None:
    """Replace Magician mesh collisions with boxes for MuJoCo.

    MuJoCo convex-hulls collision meshes via Qhull; vendor DAEs such as
    ``magicianLink2`` / ``magicianLink3`` can fail on some platforms. Visual
    meshes stay unchanged; proxies follow each link's URDF collision origin.
    """
    root = tree.getroot()
    if (root.get("name") or "").lower() != "magician":
        return
    _replace_link_collision_with_box(
        root,
        "magician_base_link",
        size_xyz=(0.17, 0.17, 0.058),
        xyz="0 0 0",
        rpy="0 0 0",
    )
    _replace_link_collision_with_box(
        root,
        "magician_link_1",
        size_xyz=(0.13, 0.19, 0.15),
        xyz="0 0 0",
        rpy="0 0 0",
    )
    _replace_link_collision_with_box(
        root,
        "magician_link_2",
        size_xyz=(0.14, 0.055, 0.20),
        xyz="0 0 0",
        rpy="0 -0.3490658503988659 0",
    )
    _replace_link_collision_with_box(
        root,
        "magician_link_3",
        size_xyz=(0.21, 0.045, 0.15),
        xyz="0.01645236135969491 0 -0.13399373047157848",
        rpy="0 -0.47123889803846897 0",
    )
    _replace_link_collision_with_box(
        root,
        "magician_link_4",
        size_xyz=(0.12, 0.045, 0.076),
        xyz="-0.17715067840465534 0 -0.058595107267811",
        rpy="0 0 0",
    )
    _replace_link_collision_with_box(
        root,
        "magician_link_suction_cup",
        size_xyz=(0.03, 0.025, 0.065),
        xyz="0 0 0",
        rpy="0 0 0",
    )


def _simplify_mg400_mujoco_collisions(tree: ET.ElementTree) -> None:
    """Use a primitive for the MG400 base collision mesh in MuJoCo.

    The vendor base_link.obj can trip MuJoCo/QHull convex-hull generation on
    some installs. The visual mesh is kept; only the collision proxy changes.
    """
    root = tree.getroot()
    if (root.get("name") or "").lower() != "mg400":
        return
    _replace_link_collision_with_cylinder(
        root,
        "mg400_base_link",
        radius=0.095,
        length=0.113,
        xyz="0 0 0.0565",
    )


def _fix_zero_size_box_geoms(tree: ET.ElementTree) -> None:
    """Replace zero-sized ``<box size="0 0 0"/>`` geoms with a tiny epsilon.

    MuJoCo rejects zero-size geoms ("size 0 must be positive"). The jkaniuka
    Magician URDF uses zero-size visuals on the virtual mimic link, which is
    fine for PyBullet (it ignores them) but blows up MuJoCo's parser. Setting
    a 1e-6 m box keeps the link visible-but-invisible without changing
    physics or collisions.
    """
    for box in tree.getroot().iter("box"):
        size = (box.attrib.get("size") or "").strip()
        if not size:
            continue
        parts = size.split()
        if all(float(s) == 0.0 for s in parts if s):
            box.set("size", "1e-6 1e-6 1e-6")


def _simlab_spec(tree: ET.ElementTree) -> dict[str, object] | None:
    """Return hidden lab-chain metadata for supported robots."""
    root = tree.getroot()
    robot_name = (root.get("name") or "").lower()
    if robot_name == "magician":
        has_suction = root.find("./link[@name='magician_link_suction_cup']") is not None
        uses_default_link4 = False
        for mesh in root.findall("./link[@name='magician_link_4']/visual/geometry/mesh"):
            filename = (mesh.attrib.get("filename") or "").lower()
            if "magicianlink4_default.dae" in filename:
                uses_default_link4 = True
                break
        if has_suction:
            tip_xyz = "0.06 0 -0.07"
        elif uses_default_link4:
            tip_xyz = "0 0 0"
        else:
            tip_xyz = "0.06 0 0"
        # Firmware-correct chain: J2=0 → upper arm vertical (+Z); J3=0 →
        # forearm horizontal (+X). The j3 simlab joint receives the
        # *relative* elbow angle (J3_fw − J2_fw); see ``sim_dobot.py``.
        # ``shoulder_axis`` and ``elbow_axis`` are world +Y; ``upper_arm_xyz``
        # places L1 along +Z so Ry(J2) sweeps it through +X as J2 increases.
        # Mounted on jkaniuka's `magician_root_link` (URDF root), which is
        # at the base mounting surface; `z_base=0` means the simlab tip
        # represents the firmware GetPose Z (referenced to the shoulder
        # pivot, not the table).
        return {
            "prefix": "simlab_magician",
            "root_parent": "magician_root_link",
            "x_base": 0.0,
            "z_base": 0.0,
            "l1": 0.135,
            "l2": 0.147,
            "shoulder_axis": "0 1 0",
            "elbow_axis": "0 1 0",
            "upper_arm_xyz": "0 0 0.135",
            "forearm_xyz": "0.147 0 0",
            "tip_xyz": tip_xyz,
            "elbow_relative": True,
        }
    if robot_name == "mg400":
        return {
            "prefix": "simlab_mg400",
            "root_parent": "arm_frame_link_offset",
            "z_base": 0.116,
            "l1": 0.175,
            "l2": 0.175,
        }
    return None


def _inject_simlab_chain(tree: ET.ElementTree) -> dict[str, str] | None:
    """Inject a hidden lab-frame kinematic chain used for parity checks."""
    spec = _simlab_spec(tree)
    if spec is None:
        return None

    root = tree.getroot()
    prefix = str(spec["prefix"])
    tip_link = f"{prefix}_tip"
    if root.find(f"./link[@name='{tip_link}']") is not None:
        return {
            "j1": f"{prefix}_j1",
            "j2": f"{prefix}_j2",
            "j3": f"{prefix}_j3",
            "j4": f"{prefix}_j4",
            "tip_link": tip_link,
        }

    mount = f"{prefix}_mount"
    base_rot = f"{prefix}_base_rot"
    shoulder = f"{prefix}_shoulder"
    upper = f"{prefix}_upper"
    elbow = f"{prefix}_elbow"
    forearm = f"{prefix}_forearm"
    wrist = f"{prefix}_wrist"

    for link_name in (mount, base_rot, shoulder, upper, elbow, forearm, wrist, tip_link):
        _append_hidden_link(root, link_name)

    shoulder_axis = str(spec.get("shoulder_axis", "0 -1 0"))
    elbow_axis = str(spec.get("elbow_axis", "0 -1 0"))
    upper_arm_xyz = str(spec.get("upper_arm_xyz", f"{spec['l1']} 0 0"))
    forearm_xyz = str(spec.get("forearm_xyz", f"{spec['l2']} 0 0"))

    _append_joint(
        root,
        name=f"{prefix}_mount_joint",
        joint_type="fixed",
        parent=str(spec["root_parent"]),
        child=mount,
        xyz=f"{spec.get('x_base', 0.0)} 0 0",
    )
    _append_joint(
        root,
        name=f"{prefix}_j1",
        joint_type="revolute",
        parent=mount,
        child=base_rot,
        xyz="0 0 0",
        axis="0 0 1",
    )
    _append_joint(
        root,
        name=f"{prefix}_shoulder_offset",
        joint_type="fixed",
        parent=base_rot,
        child=shoulder,
        xyz=f"0 0 {spec['z_base']}",
    )
    _append_joint(
        root,
        name=f"{prefix}_j2",
        joint_type="revolute",
        parent=shoulder,
        child=upper,
        xyz="0 0 0",
        axis=shoulder_axis,
    )
    _append_joint(
        root,
        name=f"{prefix}_upper_arm",
        joint_type="fixed",
        parent=upper,
        child=elbow,
        xyz=upper_arm_xyz,
    )
    _append_joint(
        root,
        name=f"{prefix}_j3",
        joint_type="revolute",
        parent=elbow,
        child=forearm,
        xyz="0 0 0",
        axis=elbow_axis,
    )
    _append_joint(
        root,
        name=f"{prefix}_forearm",
        joint_type="fixed",
        parent=forearm,
        child=wrist,
        xyz=forearm_xyz,
    )
    _append_joint(
        root,
        name=f"{prefix}_j4",
        joint_type="revolute",
        parent=wrist,
        child=tip_link,
        xyz=str(spec.get("tip_xyz", "0 0 0")),
        axis="0 0 1",
    )
    return {
        "j1": f"{prefix}_j1",
        "j2": f"{prefix}_j2",
        "j3": f"{prefix}_j3",
        "j4": f"{prefix}_j4",
        "tip_link": tip_link,
    }


def _hash_bundle(urdf_path: Path, mesh_paths: list[Path], extra: str = "") -> str:
    """Hash URDF content + (mtime, size) of every referenced mesh + extra."""
    h = hashlib.blake2b(digest_size=12)
    h.update(urdf_path.read_bytes())
    for p in sorted({m.resolve() for m in mesh_paths}):
        h.update(str(p).encode())
        if p.exists():
            st = p.stat()
            h.update(f"{st.st_size}:{int(st.st_mtime)}".encode())
    if extra:
        h.update(extra.encode())
    return h.hexdigest()


def _normalise_roots(
    urdf_path: Path,
    package_roots: Mapping[str, os.PathLike] | None,
) -> dict[str, Path]:
    if package_roots is None:
        return _default_package_roots(urdf_path)
    return {k: Path(v).resolve() for k, v in package_roots.items()}


# ---------------------------------------------------------------------------
# Public API — PyBullet
# ---------------------------------------------------------------------------

def prepare_urdf_for_pybullet(
    urdf_path: str | os.PathLike,
    out_dir: str | os.PathLike,
    package_roots: Mapping[str, os.PathLike] | None = None,
) -> Path:
    """Rewrite a URDF so every mesh URI is an absolute path PyBullet can load.

    STL meshes are converted to OBJ via trimesh (works around PyBullet's
    brittle binary-STL extractor). DAE and OBJ meshes are copied as-is.

    Args:
        urdf_path     : Path to source URDF file.
        out_dir       : Directory for cached prepared URDFs + converted meshes.
        package_roots : Optional map {pkg_name: pkg_root_dir} for `package://`
                        URI resolution. Defaults to the URDF's grandparent.

    Returns:
        Path to the rewritten URDF. Safe to pass to p.loadURDF().
    """
    urdf_path = Path(urdf_path).resolve()
    out_dir = Path(out_dir).resolve()
    roots = _normalise_roots(urdf_path, package_roots)

    tree = ET.parse(urdf_path)
    resolved = [
        _resolve_mesh_uri(m.attrib["filename"], urdf_path, roots)
        for m in _iter_mesh_elements(tree)
    ]

    _inject_simlab_chain(tree)
    _ensure_inertials(tree)

    bundle_hash = _hash_bundle(
        urdf_path,
        resolved,
        extra="pybullet-v11-magician-jkaniuka",
    )
    cache_dir = out_dir / "pybullet" / f"{urdf_path.stem}_{bundle_hash}"
    out_urdf = cache_dir / urdf_path.name
    if out_urdf.exists():
        return out_urdf

    meshes_out = cache_dir / "meshes"
    meshes_out.mkdir(parents=True, exist_ok=True)

    for mesh in _iter_mesh_elements(tree):
        src = _resolve_mesh_uri(mesh.attrib["filename"], urdf_path, roots)
        src = _resolve_usable_source(src)
        ext = src.suffix.lower()
        if ext in _STL_EXT:
            dst = meshes_out / f"{src.stem}.obj"
            if not dst.exists():
                _convert_mesh(src, dst)
        else:
            dst = meshes_out / src.name
            if not dst.exists():
                shutil.copy2(src, dst)
        mesh.set("filename", str(dst))

    cache_dir.mkdir(parents=True, exist_ok=True)
    tree.write(out_urdf, xml_declaration=True, encoding="utf-8")
    return out_urdf


# ---------------------------------------------------------------------------
# Public API — MuJoCo
# ---------------------------------------------------------------------------

@dataclass(frozen=True)
class EqualityConstraint:
    """Parallelogram constraint spec for a MuJoCo wrapper.

    Encodes `follower = multiplier * driver`, i.e. polycoef=[0, multiplier, 0, 0, 0].
    multiplier=-1.0 enforces the standard 4-bar equal-and-opposite linkage.
    """
    driver: str
    follower: str
    multiplier: float = -1.0


def prepare_urdf_for_mujoco(
    urdf_path: str | os.PathLike,
    out_dir: str | os.PathLike,
    package_roots: Mapping[str, os.PathLike] | None = None,
    equalities: Iterable[EqualityConstraint] | None = None,
) -> Path:
    """Prepare a URDF for MuJoCo + emit an MJCF wrapper `wrapper.xml`.

    DAE meshes are converted to OBJ (MuJoCo cannot parse Collada); STL and OBJ
    are copied as-is. The wrapper uses `<include>` so the URDF compiles inside
    an MJCF context, with `<compiler meshdir="meshes"/>` pointing at the
    co-located mesh directory. Provided EqualityConstraints are appended as
    `<joint joint1=... joint2=... polycoef=...>` entries.

    Args:
        urdf_path     : Path to source URDF file.
        out_dir       : Directory for cached prepared URDFs + converted meshes.
        package_roots : Optional map {pkg_name: pkg_root_dir} for `package://`
                        URI resolution. Defaults to the URDF's grandparent.
        equalities    : Iterable of EqualityConstraint (parallelogram bindings).

    Returns:
        Path to the prepared URDF (pass to mujoco.MjModel.from_xml_path).
        MuJoCo auto-detects URDF by `<robot>` root and compiles it; any
        injected `<mujoco>` child element carries MJCF extensions
        (compiler options, equality constraints).
    """
    urdf_path = Path(urdf_path).resolve()
    out_dir = Path(out_dir).resolve()
    roots = _normalise_roots(urdf_path, package_roots)
    equalities = list(equalities or [])

    tree = ET.parse(urdf_path)
    resolved = [
        _resolve_mesh_uri(m.attrib["filename"], urdf_path, roots)
        for m in _iter_mesh_elements(tree)
    ]

    _inject_simlab_chain(tree)
    _simplify_magician_mujoco_collisions(tree)
    _simplify_mg400_mujoco_collisions(tree)
    _fix_zero_size_box_geoms(tree)
    _ensure_inertials(tree)
    extra = "mujoco-v15-magician-mesh-collproxy|" + "|".join(
        f"{e.driver}->{e.follower}:{e.multiplier}" for e in equalities
    )
    bundle_hash = _hash_bundle(urdf_path, resolved, extra=extra)
    cache_dir = out_dir / "mujoco" / f"{urdf_path.stem}_{bundle_hash}"
    prepared_urdf = cache_dir / f"{urdf_path.stem}.urdf"
    if prepared_urdf.exists():
        return prepared_urdf

    meshes_out = cache_dir / "meshes"
    meshes_out.mkdir(parents=True, exist_ok=True)

    # Track already-converted sources so visual+collision references to the
    # same mesh map to the same output file. MuJoCo's <asset> table collapses
    # duplicates by filename, but sibling files (link_1.dae + link_1.STL)
    # that share a stem must still get disambiguated OBJ names.
    src_to_name: dict[Path, str] = {}
    used_names: set[str] = set()

    def _unique(name: str) -> str:
        if name not in used_names:
            return name
        stem, dot, ext = name.rpartition(".")
        i = 0
        while True:
            i += 1
            candidate = f"{stem}_{i}.{ext}" if dot else f"{name}_{i}"
            if candidate not in used_names:
                return candidate

    for mesh in _iter_mesh_elements(tree):
        src = _resolve_mesh_uri(mesh.attrib["filename"], urdf_path, roots)
        src = _resolve_usable_source(src)
        if src in src_to_name:
            mesh.set("filename", src_to_name[src])
            continue
        ext = src.suffix.lower()
        if ext in _DAE_EXT:
            dst_name = _unique(f"{src.stem}.obj")
            dst = meshes_out / dst_name
            if not dst.exists():
                _convert_mesh(src, dst)
        else:
            dst_name = _unique(src.name)
            dst = meshes_out / dst_name
            if not dst.exists():
                shutil.copy2(src, dst)
        used_names.add(dst_name)
        src_to_name[src] = dst_name
        # MuJoCo resolves mesh files relative to compiler.meshdir, so store
        # the bare filename rather than an absolute path.
        mesh.set("filename", dst_name)

    _sanitise_urdf_for_mujoco(tree)

    # Inject a <mujoco> child inside <robot> — this is MuJoCo's documented
    # extension point for URDF files (XMLreference "URDF extensions"). The
    # compiler settings + any <equality> blocks live inside it; the rest of
    # the URDF is parsed by MuJoCo's URDF compiler unchanged.
    root = tree.getroot()
    for existing in list(root.findall("mujoco")):
        root.remove(existing)
    mj = ET.SubElement(root, "mujoco")
    ET.SubElement(mj, "compiler", {
        "meshdir": "meshes",
        "discardvisual": "false",
        "strippath": "false",
        "balanceinertia": "true",
        "autolimits": "true",
    })
    if equalities:
        eq_root = ET.SubElement(mj, "equality")
        for eq in equalities:
            ET.SubElement(eq_root, "joint", {
                "joint1": eq.follower,
                "joint2": eq.driver,
                "polycoef": f"0 {eq.multiplier} 0 0 0",
            })

    tree.write(prepared_urdf, xml_declaration=True, encoding="utf-8")
    return prepared_urdf


# ---------------------------------------------------------------------------
# Public API — runtime constraint helpers
# ---------------------------------------------------------------------------

def _pybullet_joint_index_by_name(client, robot_id: int, name: str) -> int | None:
    import pybullet as p
    n = p.getNumJoints(robot_id, physicsClientId=client)
    for i in range(n):
        info = p.getJointInfo(robot_id, i, physicsClientId=client)
        if info[1].decode() == name:
            return i
    return None


def apply_parallelogram_constraint_pybullet(
    client, robot_id: int,
    driver_joint_name: str, follower_joint_name: str,
    ratio: float = -1.0, max_force: float = 100.0,
) -> int:
    """Create a JOINT_GEAR constraint between two named revolute joints.

    ratio=-1.0 enforces follower = -driver (standard 4-bar parallelogram).
    Returns the PyBullet constraint id (pass to p.removeConstraint to release).
    """
    import pybullet as p
    driver_idx = _pybullet_joint_index_by_name(client, robot_id, driver_joint_name)
    follower_idx = _pybullet_joint_index_by_name(client, robot_id, follower_joint_name)
    if driver_idx is None or follower_idx is None:
        raise ValueError(
            "Parallelogram constraint: joint(s) not found in URDF "
            f"(driver='{driver_joint_name}'->{driver_idx}, "
            f"follower='{follower_joint_name}'->{follower_idx})"
        )
    cid = p.createConstraint(
        parentBodyUniqueId=robot_id, parentLinkIndex=driver_idx,
        childBodyUniqueId=robot_id, childLinkIndex=follower_idx,
        jointType=p.JOINT_GEAR,
        jointAxis=[0, 1, 0],
        parentFramePosition=[0, 0, 0],
        childFramePosition=[0, 0, 0],
        physicsClientId=client,
    )
    p.changeConstraint(cid, gearRatio=ratio, maxForce=max_force, physicsClientId=client)
    return cid


def inject_equality_constraint_mjcf(
    prepared_urdf_path: str | os.PathLike,
    driver_joint: str, follower_joint: str,
    multiplier: float = -1.0,
) -> None:
    """Append one `<joint>` binding to an already-prepared URDF's MuJoCo block.

    Targets the `<mujoco><equality>...</equality></mujoco>` element inside
    the URDF's `<robot>` root (MuJoCo's URDF-extension convention). Creates
    both blocks if absent. Modifies the file in place.
    """
    prepared_urdf_path = Path(prepared_urdf_path)
    tree = ET.parse(prepared_urdf_path)
    root = tree.getroot()
    mj = root.find("mujoco")
    if mj is None:
        mj = ET.SubElement(root, "mujoco")
    eq = mj.find("equality")
    if eq is None:
        eq = ET.SubElement(mj, "equality")
    ET.SubElement(eq, "joint", {
        "joint1": follower_joint,
        "joint2": driver_joint,
        "polycoef": f"0 {multiplier} 0 0 0",
    })
    tree.write(prepared_urdf_path, xml_declaration=True, encoding="utf-8")


# ---------------------------------------------------------------------------
# CLI smoke test
# ---------------------------------------------------------------------------

def _default_cache_dir() -> Path:
    return Path(os.environ.get("DOBOT_SIM_CACHE", Path.home() / ".cache" / "dobot_sim"))


if __name__ == "__main__":
    import sys
    if len(sys.argv) < 2:
        print("Usage: python urdf_loader.py <urdf_path> [--mujoco]")
        sys.exit(1)
    urdf = Path(sys.argv[1]).resolve()
    out = _default_cache_dir()
    if "--mujoco" in sys.argv:
        path = prepare_urdf_for_mujoco(urdf, out)
        print(f"[urdf_loader] MJCF wrapper : {path}")
    else:
        path = prepare_urdf_for_pybullet(urdf, out)
        print(f"[urdf_loader] Prepared URDF: {path}")
    print(f"[urdf_loader] Cache dir     : {out}")

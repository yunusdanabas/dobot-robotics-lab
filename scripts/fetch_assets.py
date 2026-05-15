"""Download optional SDKs and URDF assets for ME403 labs."""

from __future__ import annotations

import argparse
import os
import shutil
import subprocess
import sys
import urllib.request
import xml.etree.ElementTree as ET
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
VENDOR = ROOT / "vendor"

MAGICIAN_URL = "https://github.com/jkaniuka/magician_ros2.git"
MG400_ROS2_URL = "https://github.com/HarvestX/MG400_ROS2.git"
MG400_SDK_URL = "https://github.com/Dobot-Arm/TCP-IP-4Axis-Python.git"

# DAE meshes referenced by the vendored URDFs (gripper + hand-stripped
# magician_none / magician_suction variants). All come from
# jkaniuka/magician_ros2 dobot_description/meshes/dae/.
MAGICIAN_MESHES = (
    "magicianBase.dae",
    "magicianLink1.dae",
    "magicianLink2.dae",
    "magicianLink3.dae",
    "magicianLink4.dae",
    "magicianLink4_default.dae",
    "suction_cup.dae",
    "gripper_core.dae",
    "jaw_left.dae",
    "jaw_right.dae",
)

MAGICIAN_URDFS = (
    "magician.urdf",
    "magician_none.urdf",
    "magician_motor.urdf",
    "magician_suction.urdf",
)

_MAGICIAN_GRIPPER_LINKS = {
    "magician_link_gripper_core",
    "magician_link_gripper_jaw_left",
    "magician_link_gripper_jaw_right",
}
_MAGICIAN_GRIPPER_JOINTS = {
    "magician_joint_4",
    "magician_joint_prismatic_l",
    "magician_joint_prismatic_r",
}


def _run(cmd: list[str]) -> None:
    print("+", " ".join(str(part) for part in cmd))
    subprocess.run(cmd, check=True)


def _clone(url: str, target: Path) -> None:
    if target.exists():
        print(f"[skip] {target} already exists")
        return
    target.parent.mkdir(parents=True, exist_ok=True)
    _run(["git", "clone", url, str(target)])


def _has_magician_base_tree(path: Path) -> bool:
    return (path / "urdf" / "magician.urdf").exists() and (path / "meshes" / "dae").exists()


def _is_magician_tree(path: Path) -> bool:
    urdf_dir = path / "urdf"
    return _has_magician_base_tree(path) and all((urdf_dir / name).exists() for name in MAGICIAN_URDFS)


def _remove_named_children(root: ET.Element, tag: str, names: set[str]) -> None:
    for child in list(root.findall(tag)):
        if child.get("name") in names:
            root.remove(child)


def _set_link4_mesh(root: ET.Element, mesh_name: str) -> None:
    link4 = root.find("./link[@name='magician_link_4']")
    if link4 is None:
        raise ValueError("Magician source URDF is missing magician_link_4")
    for mesh in link4.iter("mesh"):
        mesh.set("filename", f"package://dobot_description/meshes/dae/{mesh_name}")


def _append_joint4(root: ET.Element, child_link: str, origin_xyz: str) -> None:
    joint = ET.SubElement(root, "joint", name="magician_joint_4", type="revolute")
    ET.SubElement(joint, "parent", link="magician_link_4")
    ET.SubElement(joint, "child", link=child_link)
    ET.SubElement(joint, "axis", xyz="0 0 1")
    ET.SubElement(
        joint,
        "limit",
        effort="2000",
        lower="-2.6179938779914944",
        upper="2.6179938779914944",
        velocity="1",
    )
    ET.SubElement(joint, "origin", rpy="0 0 0", xyz=origin_xyz)
    ET.SubElement(joint, "dynamics", damping="0.0", friction="0.0")


def _append_suction_link(root: ET.Element) -> None:
    link = ET.SubElement(root, "link", name="magician_link_suction_cup")
    for tag in ("visual", "collision"):
        node = ET.SubElement(link, tag)
        ET.SubElement(node, "origin", rpy="0 0 0", xyz="0 0 0")
        geometry = ET.SubElement(node, "geometry")
        ET.SubElement(geometry, "mesh", filename="package://dobot_description/meshes/dae/suction_cup.dae")


def _write_magician_variant(base_urdf: Path, target: Path, mode: str) -> None:
    tree = ET.parse(base_urdf)
    root = tree.getroot()
    _remove_named_children(root, "link", _MAGICIAN_GRIPPER_LINKS)
    _remove_named_children(root, "joint", _MAGICIAN_GRIPPER_JOINTS)

    if mode == "none":
        _set_link4_mesh(root, "magicianLink4_default.dae")
        _append_joint4(root, "magician_link_ee", "0 0 0")
        ET.SubElement(root, "link", name="magician_link_ee")
    elif mode == "motor":
        _set_link4_mesh(root, "magicianLink4.dae")
        _append_joint4(root, "magician_link_ee", "0.06 0 0")
        ET.SubElement(root, "link", name="magician_link_ee")
    elif mode == "suction":
        _set_link4_mesh(root, "magicianLink4.dae")
        _append_joint4(root, "magician_link_suction_cup", "0.06 0 0")
        _append_suction_link(root)
    else:
        raise ValueError(f"Unknown Magician URDF mode {mode!r}")

    ET.indent(tree, space="  ")
    target.write_text(ET.tostring(root, encoding="unicode") + "\n", encoding="utf-8")


def _ensure_magician_urdf_variants(target: Path) -> None:
    urdf_dir = target / "urdf"
    base_urdf = urdf_dir / "magician.urdf"
    if not base_urdf.exists():
        return
    for mode in ("none", "motor", "suction"):
        variant = urdf_dir / f"magician_{mode}.urdf"
        if not variant.exists():
            print(f"[generate] {variant.name}")
            _write_magician_variant(base_urdf, variant, mode)


def _copy_tree(src: Path, dst: Path) -> None:
    if dst.exists():
        print(f"[skip] {dst} already exists")
        return
    print(f"[copy] {src} -> {dst}")
    shutil.copytree(src, dst)


def _resolve_magician_source_dir() -> Path | None:
    """Return a local Magician description source tree if one is available."""
    env_dir = os.environ.get("DOBOT_MAGICIAN_URDF_SOURCE_DIR")
    candidates: list[Path] = []
    if env_dir:
        candidates.append(Path(env_dir).expanduser().resolve())
    # Local monorepo fallback (useful when repackaging this course workspace).
    candidates.append((ROOT.parent / "vendor" / "magician_ros2_urdf").resolve())
    for cand in candidates:
        if _has_magician_base_tree(cand):
            return cand
    return None


def _clone_magician_from_candidates(target: Path) -> None:
    """Clone jkaniuka/magician_ros2 then extract URDF + DAE meshes."""
    env_url = os.environ.get("DOBOT_MAGICIAN_URDF_URL", "").strip()
    clone_urls = [u for u in (env_url, MAGICIAN_URL) if u]
    errors: list[str] = []
    work = target.parent / "_jkaniuka_magician_ros2"
    for url in clone_urls:
        try:
            if work.exists():
                shutil.rmtree(work)
            _run(["git", "clone", "--depth", "1", url, str(work)])
            src_urdf = work / "dobot_description" / "model" / "clean_model_no_macros.urdf"
            src_meshes = work / "dobot_description" / "meshes" / "dae"
            if not (src_urdf.exists() and src_meshes.exists()):
                errors.append(f"{url}: expected layout not found inside repo")
                continue
            target.mkdir(parents=True, exist_ok=True)
            (target / "urdf").mkdir(exist_ok=True)
            (target / "meshes" / "dae").mkdir(parents=True, exist_ok=True)
            shutil.copy2(src_urdf, target / "urdf" / "magician.urdf")
            for dae in src_meshes.glob("*.dae"):
                shutil.copy2(dae, target / "meshes" / "dae" / dae.name)
            _ensure_magician_urdf_variants(target)
            shutil.rmtree(work, ignore_errors=True)
            if _is_magician_tree(target):
                return
            errors.append(f"{url}: copy step left an incomplete tree at {target}")
        except subprocess.CalledProcessError as exc:
            errors.append(f"{url} clone failed: {exc}")
        finally:
            if work.exists():
                shutil.rmtree(work, ignore_errors=True)
    details = "\n".join(f"  - {line}" for line in errors) if errors else "  - no clone URLs configured"
    raise SystemExit(
        "Could not fetch Magician URDF assets automatically.\n"
        "Tried clone sources:\n"
        f"{details}\n\n"
        "Fix options:\n"
        "  1) Set DOBOT_MAGICIAN_URDF_URL to a working git URL\n"
        "  2) Set DOBOT_MAGICIAN_URDF_SOURCE_DIR to a local magician_ros2_urdf folder\n"
        f"  3) Manually place assets at: {target}"
    )


def _download(url: str, target: Path) -> None:
    target.parent.mkdir(parents=True, exist_ok=True)
    print(f"[download] {target.name}")
    urllib.request.urlretrieve(url, target)


def fetch_magician_urdf() -> None:
    target = VENDOR / "magician_ros2_urdf"
    if _is_magician_tree(target):
        print(f"[ok] {target} exists")
    elif _has_magician_base_tree(target):
        _ensure_magician_urdf_variants(target)
    else:
        local_source = _resolve_magician_source_dir()
        if local_source and local_source != target:
            _copy_tree(local_source, target)
            _ensure_magician_urdf_variants(target)
        elif not target.exists():
            _clone_magician_from_candidates(target)
        if not _is_magician_tree(target):
            raise FileNotFoundError(
                f"Expected Magician assets under {target} "
                f"(need {', '.join('urdf/' + name for name in MAGICIAN_URDFS)} and meshes/dae/)."
            )

    meshes = target / "meshes" / "dae"
    missing = [name for name in MAGICIAN_MESHES
               if not (meshes / name).exists() or (meshes / name).stat().st_size < 512]
    if missing:
        raise FileNotFoundError(
            f"Magician meshes missing under {meshes}: {missing}.\n"
            "Re-run with a clean target directory or set DOBOT_MAGICIAN_URDF_SOURCE_DIR "
            "to a local jkaniuka magician_ros2_urdf checkout."
        )
    print("[ok] Magician meshes look ready")


def fetch_mg400_urdf() -> None:
    source = VENDOR / "mg400_ros2"
    target = VENDOR / "mg400_description"
    _clone(MG400_ROS2_URL, source)

    source_desc = source / "mg400_description"
    if not source_desc.exists():
        raise FileNotFoundError(f"Expected MG400 description at {source_desc}")

    if not target.exists():
        print(f"[copy] {source_desc} -> {target}")
        shutil.copytree(source_desc, target)
    else:
        print(f"[skip] {target} already exists")

    urdf = target / "urdf" / "mg400.urdf"
    xacro_file = target / "urdf" / "mg400.urdf.xacro"
    if urdf.exists():
        print(f"[ok] {urdf} exists")
        return

    try:
        import xacro
    except ImportError as exc:
        raise SystemExit(
            "xacro is required to generate mg400.urdf. Run:\n"
            "  python -m pip install xacro"
        ) from exc

    print(f"[xacro] generating {urdf}")
    doc = xacro.process_file(str(xacro_file))
    urdf.write_text(doc.toprettyxml(indent="  "), encoding="utf-8")


def fetch_mg400_sdk() -> None:
    _clone(MG400_SDK_URL, VENDOR / "TCP-IP-4Axis-Python")


def main() -> int:
    parser = argparse.ArgumentParser(description="Fetch optional ME403 Dobot assets into vendor/.")
    parser.add_argument("--magician-urdf", action="store_true", help="Fetch Magician URDF and meshes")
    parser.add_argument("--mg400-urdf", action="store_true", help="Fetch MG400 URDF and meshes")
    parser.add_argument("--mg400-sdk", action="store_true", help="Fetch MG400 TCP/IP SDK")
    parser.add_argument("--all", action="store_true", help="Fetch all optional assets")
    args = parser.parse_args()

    if not shutil.which("git"):
        raise SystemExit("git is required to fetch assets. Install git, then run this script again.")

    VENDOR.mkdir(parents=True, exist_ok=True)

    if args.all or args.magician_urdf:
        fetch_magician_urdf()
    if args.all or args.mg400_urdf:
        fetch_mg400_urdf()
    if args.all or args.mg400_sdk:
        fetch_mg400_sdk()

    if not any((args.all, args.magician_urdf, args.mg400_urdf, args.mg400_sdk)):
        parser.print_help()
        return 0

    print("\nAsset fetch completed.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

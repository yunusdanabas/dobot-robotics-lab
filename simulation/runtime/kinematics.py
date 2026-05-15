"""Shared kinematics for ME403 simulation (stdlib only).

Implements firmware-aligned Magician FK/IK, MG400 pedagogical planar 2R helpers,
and the calibrated MG400 API-frame FK used by SimMG400. Teleops and smoke tests
import this module without pulling in PyBullet, MuJoCo, or URDF loaders.

Magician joints match firmware/GetPose semantics (vertical-zero shoulder,
horizontal-zero forearm in world axes; internal sim joints match hardware).

For joint tables, tool offsets, citations, and conventions, see
ME403_LabFiles/docs/simulation.md and AGENTS.md / CLAUDE.md in the repo root.

Legacy PlanarArmKinematics plus fk_2r_planar/ik_2r_planar serve the MG400
pedagogical model and workspace-boundary tests; Magician Cartesian math uses
fk_magician_firmware / ik_magician_firmware below.
"""

from __future__ import annotations

from dataclasses import dataclass
import math


@dataclass(frozen=True)
class PlanarArmKinematics:
    """Planar 2R link lengths (mm) and shoulder height for horizontal-zero model."""

    l1: float
    l2: float
    z_base: float


# Legacy planar-2R model (horizontal-zero shoulder, ``theta3 = q2 + q3``).
# Kept for the MG400 pedagogical FK (``fk_mg400``/``ik_mg400``) and for
# ``test_11_workspace_boundaries``. The Magician FK below no longer routes
# through this — it uses the firmware-correct vertical-zero shoulder
# convention via ``fk_magician_firmware``.
MAGICIAN = PlanarArmKinematics(l1=135.0, l2=147.0, z_base=103.0)
MG400 = PlanarArmKinematics(l1=175.0, l2=175.0, z_base=116.0)

# Hardware/API-frame MG400 constants.  These match the calibrated FK used by
# ``sim_mg400`` and are separate from the pedagogical lab model above.
MG400_API_L1 = 174.5
MG400_API_L2 = 175.5
MG400_API_C_R = 108.0
MG400_API_C_Z = -52.0


# ---------------------------------------------------------------------------
# Magician — firmware-correct FK / IK
# ---------------------------------------------------------------------------

# Magician link lengths (mm). Verified against jkaniuka magician_ros2 xacro
# (link_2_length=0.135, link_3_length=0.147) and OmarMalla peer-reviewed
# FK against 14 hardware ground-truth poses.
MAGICIAN_L1 = 135.0   # upper arm (shoulder pivot → elbow)
MAGICIAN_L2 = 147.0   # forearm (elbow → wrist axis / J4)
MAGICIAN_L4 = 0.0     # legacy alias for default tool X offset (mm)

# Per-tool TCP offset in the tool-local frame (mm). In this simplified
# firmware-aligned model the tool frame stays level to world Z (parallelogram),
# so tool-local X maps to radial reach and tool-local Z maps to world Z.
#
# Requested convention:
#   - none    : bare default flange reference (0, 0, 0)
#   - motor   : +60 mm forward on X (previous ``none`` behavior)
#   - suction : +60 mm forward on X, -70 mm down on Z (physical suction cup tip)
#               NOTE: DobotStudio factory "suction cup" preset uses (+60,0,0) — same as
#               motor — because it reports the motor-shaft TCP, not the physical cup tip.
#               Use the "motor" preset when comparing GetPose with DobotStudio suction mode.
MAGICIAN_TOOL_OFFSET: dict[str, tuple[float, float, float]] = {
    "none": (0.0, 0.0, 0.0),
    "motor": (60.0, 0.0, 0.0),
    "suction": (60.0, 0.0, -70.0),
}


def _resolve_tool_offset(
    tool: str | None,
    l4_override: float | None,
    z_override: float | None = None,
) -> tuple[float, float, float]:
    """Pick tool offset from explicit override → tool name → ``none`` default."""
    if l4_override is not None or z_override is not None:
        ox = MAGICIAN_L4 if l4_override is None else float(l4_override)
        oz = 0.0 if z_override is None else float(z_override)
        return float(ox), 0.0, float(oz)
    key = "none" if tool is None else tool
    try:
        ox, oy, oz = MAGICIAN_TOOL_OFFSET[key]
        return float(ox), float(oy), float(oz)
    except KeyError as exc:
        raise ValueError(
            f"Unknown Magician tool {tool!r}; expected one of "
            f"{sorted(MAGICIAN_TOOL_OFFSET)}"
        ) from exc


def fk_magician_firmware(
    j1_deg: float,
    j2_deg: float,
    j3_deg: float,
    j4_deg: float = 0.0,
    L1: float = MAGICIAN_L1,
    L2: float = MAGICIAN_L2,
    L4: float | None = None,
    Z4: float | None = None,
    tool: str | None = None,
) -> tuple[float, float, float, float]:
    """Geometric FK for the Dobot Magician using firmware joint semantics.

    Args:
        j1_deg, j2_deg, j3_deg, j4_deg: firmware joint angles in degrees
            as reported by ``GetPose``. J2=0 is upper arm vertical;
            J3=0 is forearm horizontal (absolute world reference).
        L1, L2, L4: link lengths (mm). ``L4`` is a legacy alias for the
            tool-local X offset. ``Z4`` optionally overrides tool-local Z.

    Returns:
        ``(x, y, z, r)`` in mm/deg. ``r`` is passed through as ``j4_deg``.

    Reference equations (with tool-local offset ``(ox, oy, oz)``):
        reach = L1·sin(J2) + L2·cos(J3) + ox
        z     = L1·cos(J2) − L2·sin(J3) + oz
        x     = reach·cos(J1) − oy·sin(J1)
        y     = reach·sin(J1) + oy·cos(J1)
    """
    ox, oy, oz = _resolve_tool_offset(tool, L4, Z4)
    j1 = math.radians(float(j1_deg))
    j2 = math.radians(float(j2_deg))
    j3 = math.radians(float(j3_deg))
    reach = L1 * math.sin(j2) + L2 * math.cos(j3) + ox
    z = L1 * math.cos(j2) - L2 * math.sin(j3) + oz
    x = reach * math.cos(j1) - oy * math.sin(j1)
    y = reach * math.sin(j1) + oy * math.cos(j1)
    return float(x), float(y), float(z), float(j4_deg)


def ik_magician_firmware(
    x: float,
    y: float,
    z: float,
    r: float = 0.0,
    q0: list[float] | tuple[float, ...] | None = None,
    L1: float = MAGICIAN_L1,
    L2: float = MAGICIAN_L2,
    L4: float | None = None,
    Z4: float | None = None,
    tool: str | None = None,
) -> list[float]:
    """Analytical IK for :func:`fk_magician_firmware`.

    Solves
        ρ = L1·sin(J2) + L2·cos(J3)
        h = L1·cos(J2) − L2·sin(J3)
    where ρ = hypot(x, y) − L4 and h = z. Re-parameterising with the elbow
    difference  d = J2 − J3  gives the law-of-cosines-style identity

        ρ² + h² = L1² + L2² + 2·L1·L2·sin(d)

    so ``d = asin((ρ²+h² − L1² − L2²) / (2 L1 L2))``. With ``d`` known, a
    standard linear-combination IK then recovers J2 (and J3 = J2 − d).

    Branch selection: the elbow-up real-robot home corresponds to ``d ≥ 0``
    (forearm at-or-above the upper-arm endpoint). When ``q0`` is supplied,
    its J3-relative-to-J2 sign selects the branch so small Cartesian moves
    stay branch-stable across singularities.

    Out-of-reach targets are projected to the workspace boundary by
    clamping ``sin(d)`` to ``[-1, 1]``.
    """
    ox, oy, oz = _resolve_tool_offset(tool, L4, Z4)
    x = float(x)
    y = float(y)
    z = float(z)
    j1 = math.degrees(math.atan2(y, x))
    if abs(oy) > 1e-9:
        raise NotImplementedError("Magician IK currently supports tool-local Y offset = 0 only.")
    rho = math.hypot(x, y) - ox
    h = z - oz

    sin_d = (rho * rho + h * h - L1 * L1 - L2 * L2) / (2.0 * L1 * L2)
    sin_d = max(-1.0, min(1.0, sin_d))

    # Branch: positive d = elbow tucked above (matches firmware home).
    if q0 is not None and len(q0) >= 3:
        seed_j2 = float(q0[1])
        seed_j3 = float(q0[2])
        branch_sign = 1.0 if (seed_j2 - seed_j3) >= 0.0 else -1.0
    else:
        branch_sign = 1.0
    cos_d = branch_sign * math.sqrt(max(0.0, 1.0 - sin_d * sin_d))
    d = math.atan2(sin_d, cos_d)        # = J2 − J3

    # ρ = L1·sin(J2) + L2·cos(J2 − d)
    # h = L1·cos(J2) − L2·sin(J2 − d)
    # Expand cos(J2−d) = cos·cos + sin·sin and sin(J2−d) = sin·cos − cos·sin,
    # then group: ρ = A·sin(J2) + B·cos(J2),  h = A·cos(J2) − B·sin(J2)
    # with  A = L1 + L2·sin(d),  B = L2·cos(d).
    A = L1 + L2 * math.sin(d)
    B = L2 * math.cos(d)
    j2 = math.atan2(rho * A - h * B, rho * B + h * A)
    j3 = j2 - d

    return [
        float(j1),
        float(math.degrees(j2)),
        float(math.degrees(j3)),
        float(r),
    ]


# ---------------------------------------------------------------------------
# Generic planar 2R (horizontal-zero shoulder) — used by MG400 lab FK and
# the workspace-boundary test. Kept verbatim from before this refactor.
# ---------------------------------------------------------------------------


def fk_2r_planar(
    q_deg: list[float] | tuple[float, ...],
    model: PlanarArmKinematics,
) -> tuple[float, float, float, float]:
    """Return (x, y, z, r) for a rotating-base planar 2R arm.

    ``q_deg`` is body-frame ``[q1, q2, q3, q4]`` in degrees:
    q1 is base rotation, q2 shoulder elevation, q3 elbow closure angle
    (positive bends downward), and q4 tool/wrist rotation.
    For 3-DOF flows, ``[q1, q2, q3]`` is also accepted and q4 defaults to 0.0.
    """
    q_vals = [float(v) for v in q_deg]
    if len(q_vals) == 3:
        q1, q2, q3 = q_vals
        q4 = 0.0
    elif len(q_vals) == 4:
        q1, q2, q3, q4 = q_vals
    else:
        raise ValueError(f"Expected 3 or 4 joint values, got {len(q_vals)}")
    theta2 = math.radians(q2)
    theta3 = math.radians(q2 + q3)
    reach = model.l1 * math.cos(theta2) + model.l2 * math.cos(theta3)
    z = model.z_base + model.l1 * math.sin(theta2) + model.l2 * math.sin(theta3)
    x = reach * math.cos(math.radians(q1))
    y = reach * math.sin(math.radians(q1))
    return float(x), float(y), float(z), float(q4)


def fk_magician(
    q_deg: list[float] | tuple[float, float, float, float],
    tool: str | None = None,
) -> tuple[float, float, float, float]:
    """Dobot Magician FK — firmware semantics (see :func:`fk_magician_firmware`).

    ``q_deg`` is ``[J1, J2, J3, J4]`` (or 3-element ``[J1, J2, J3]``) in
    degrees, matching what real-robot ``GetPose`` reports.
    ``tool`` selects the TCP offset; defaults to ``"none"``
    (``(0,0,0)`` mm). ``"motor"`` uses ``(+60,0,0)`` mm and
    ``"suction"`` uses ``(+60,0,-70)`` mm (physical cup tip).
    """
    q_vals = [float(v) for v in q_deg]
    if len(q_vals) == 3:
        j1, j2, j3 = q_vals
        j4 = 0.0
    elif len(q_vals) == 4:
        j1, j2, j3, j4 = q_vals
    else:
        raise ValueError(f"Expected 3 or 4 joint values, got {len(q_vals)}")
    return fk_magician_firmware(j1, j2, j3, j4, tool=tool)


def fk_mg400(q_deg: list[float] | tuple[float, float, float, float]) -> tuple[float, float, float, float]:
    """Dobot MG400 FK — pedagogical planar 2R (course lab constants).

    ``simulation.runtime.sim_mg400`` uses a separate calibrated ``_fk`` /
    ``_ik`` for hardware-aligned simulator poses (``R = J1 + J4``, ``C_R`` / ``C_Z``).
    """
    return fk_2r_planar(q_deg, MG400)


def mg400_body_to_firmware(q_deg: list[float] | tuple[float, float, float, float]) -> tuple[float, float, float, float]:
    """Convert internal/body MG400 joints to firmware angles.

    The MG400 controller reports/accepts J3 as an absolute elbow angle, while
    the simulator stores q3 as a body-frame offset from J2.
    """
    q1, q2, q3_body, q4 = (float(v) for v in q_deg)
    return q1, q2, q2 + q3_body, q4


def mg400_firmware_to_body(j_deg: list[float] | tuple[float, float, float, float]) -> tuple[float, float, float, float]:
    """Convert firmware MG400 angles to internal/body joints."""
    j1, j2, j3_fw, j4 = (float(v) for v in j_deg)
    return j1, j2, j3_fw - j2, j4


def fk_mg400_api(q_deg: list[float] | tuple[float, float, float, float]) -> tuple[float, float, float, float]:
    """Calibrated MG400 API-frame FK matching real ``GetPose()``.

    ``q_deg`` is body-frame ``[q1, q2, q3_body, q4]`` in degrees. Firmware J3
    is ``q2 + q3_body`` and dashboard pose rotation is ``R = J1 + J4``.
    """
    q1, q2, q3_body, q4 = (float(v) for v in q_deg)
    j3_fw = q2 + q3_body
    reach = MG400_API_C_R + MG400_API_L1 * math.sin(math.radians(q2)) + MG400_API_L2 * math.cos(math.radians(j3_fw))
    z = MG400_API_C_Z + MG400_API_L1 * math.cos(math.radians(q2)) - MG400_API_L2 * math.sin(math.radians(j3_fw))
    x = reach * math.cos(math.radians(q1))
    y = reach * math.sin(math.radians(q1))
    r = q1 + q4
    return float(x), float(y), float(z), float(r)


def ik_mg400_api(
    x: float,
    y: float,
    z: float,
    r: float,
    q0: list[float] | tuple[float, float, float, float] | None = None,
    branch: str = "firmware",
) -> list[float]:
    """Analytical IK for :func:`fk_mg400_api`.

    ``branch='firmware'`` selects the branch observed from the real controller
    in current diagnostics. ``branch='continuity'`` preserves the old simulator
    policy of following the sign of ``q0[2]``.
    """
    q1 = math.degrees(math.atan2(float(y), float(x)))
    r_corr = math.hypot(float(x), float(y)) - MG400_API_C_R
    h_corr = float(z) - MG400_API_C_Z
    sin_j3 = (MG400_API_L1 ** 2 + MG400_API_L2 ** 2 - r_corr ** 2 - h_corr ** 2) / (
        2.0 * MG400_API_L1 * MG400_API_L2
    )
    sin_j3 = max(-1.0, min(1.0, sin_j3))
    if branch == "firmware":
        branch_sign = 1.0
    elif branch == "opposite":
        branch_sign = -1.0
    elif branch == "continuity":
        q3_seed = 0.0 if q0 is None else float(q0[2])
        branch_sign = 1.0 if q3_seed >= 0.0 else -1.0
    else:
        raise ValueError("MG400 API IK branch must be 'firmware', 'opposite', or 'continuity'")
    q3_body = branch_sign * math.degrees(math.asin(sin_j3))
    a = MG400_API_L1 - MG400_API_L2 * math.sin(math.radians(q3_body))
    b = MG400_API_L2 * math.cos(math.radians(q3_body))
    q2 = math.degrees(math.atan2(r_corr, h_corr) - math.atan2(b, a))
    q4 = float(r) - q1
    return [float(q1), float(q2), float(q3_body), float(q4)]


def ik_2r_planar(
    x: float,
    y: float,
    z: float,
    r: float,
    model: PlanarArmKinematics,
    q0: list[float] | tuple[float, float, float, float] | None = None,
) -> list[float]:
    """Analytical IK for the legacy horizontal-zero planar 2R model.

    Out-of-reach targets are projected to the reachable boundary by clamping
    the cosine term. When ``q0`` is supplied, its q3 sign selects the elbow
    branch so small Cartesian moves remain branch-stable.
    """
    q1 = math.degrees(math.atan2(float(y), float(x)))
    reach = math.hypot(float(x), float(y))
    h = float(z) - model.z_base
    c3 = (reach * reach + h * h - model.l1 * model.l1 - model.l2 * model.l2) / (
        2.0 * model.l1 * model.l2
    )
    c3 = max(-1.0, min(1.0, c3))
    q3_seed = 0.0 if q0 is None else float(q0[2])
    s3_sign = 1.0 if q3_seed >= 0.0 else -1.0
    s3 = s3_sign * math.sqrt(max(0.0, 1.0 - c3 * c3))
    q3 = math.degrees(math.atan2(s3, c3))
    q2 = math.degrees(
        math.atan2(h, reach)
        - math.atan2(model.l2 * math.sin(math.radians(q3)), model.l1 + model.l2 * math.cos(math.radians(q3)))
    )
    return [float(q1), float(q2), float(q3), float(r)]


def ik_magician(
    x: float,
    y: float,
    z: float,
    r: float = 0.0,
    q0: list[float] | tuple[float, float, float, float] | None = None,
    tool: str | None = None,
) -> list[float]:
    """Magician analytical IK — firmware semantics.

    Returns ``[J1, J2, J3, J4]`` in degrees that, when fed to
    :func:`fk_magician`, recover ``(x, y, z, r)``. ``tool`` selects the
    TCP offset (defaults to ``"none"``).
    """
    return ik_magician_firmware(x, y, z, r, q0=q0, tool=tool)


def ik_mg400(
    x: float,
    y: float,
    z: float,
    r: float,
    q0: list[float] | tuple[float, float, float, float] | None = None,
) -> list[float]:
    """MG400 analytical IK using the course lab constants."""
    return ik_2r_planar(x, y, z, r, MG400, q0=q0)


def max_position_error(a: tuple[float, float, float, float], b: tuple[float, float, float, float]) -> float:
    """Chebyshev XYZ error in millimetres."""
    return max(abs(a[0] - b[0]), abs(a[1] - b[1]), abs(a[2] - b[2]))


def angle_error_deg(a: float, b: float) -> float:
    """Smallest absolute angular difference in degrees."""
    return abs((float(a) - float(b) + 180.0) % 360.0 - 180.0)

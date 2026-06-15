"""Map Dobot Magician firmware joint angles to the visual URDF used in sim.

jkaniuka/magician_ros2-derived meshes live under vendor/magician_ros2_urdf and
assume the firmware "L home" posture at zero joints. Simulators ignore URDF
mimic tags, so passive parallelogram joints are applied here:
firmware_deg_to_visual_rad builds the revolute values (rad) that match the
visible chain. See dobot-robotics-lab/docs/simulation.md and repository notes for
full joint semantics.
"""

from __future__ import annotations

import math
from dataclasses import dataclass


@dataclass(frozen=True)
class VisualJointAnglesRad:
    joint_1: float
    joint_2: float
    joint_3: float
    joint_4: float
    joint_mimic_1: float
    joint_mimic_2: float


def firmware_deg_to_visual_rad(
    j1_deg: float,
    j2_deg: float,
    j3_deg: float,
    j4_deg: float = 0.0,
) -> VisualJointAnglesRad:
    """Map firmware joint angles (deg) to visual URDF joint angles (rad)."""
    j1 = math.radians(float(j1_deg))
    j2 = math.radians(float(j2_deg))
    j3 = math.radians(float(j3_deg))
    j4 = math.radians(float(j4_deg))
    elbow_rel = j3 - j2
    return VisualJointAnglesRad(
        joint_1=j1,
        joint_2=j2,
        joint_3=elbow_rel,
        joint_4=j4,
        joint_mimic_1=-j2,
        joint_mimic_2=-elbow_rel,
    )


# Backwards-compatibility shim for callers still importing the old name.
def body_deg_to_visual_rad(
    j1_deg: float,
    j2_deg: float,
    j3_deg: float,
) -> VisualJointAnglesRad:
    """Deprecated alias for firmware_deg_to_visual_rad (J4 defaults to 0)."""
    return firmware_deg_to_visual_rad(j1_deg, j2_deg, j3_deg)

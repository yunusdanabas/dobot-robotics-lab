"""MG400 simulation: SimMG400 exposes dashboard/move_api/feed like the TCP driver.

URDF backends (default mujoco via DOBOT_SIM_BACKEND, pybullet optional) reuse meshes prepared by
urdf_loader. Enable with DOBOT_SIMULATION=1 and DOBOT_SIM_BACKEND as in the
package simulation README.

API-frame FK matches factory GetPose calibration (vertical J2, R = J1 + J4);
constants live in kinematics.py. Override URDF with DOBOT_MG400_URDF[_PATH];
otherwise the loader looks for vendor/mg400_description/urdf/mg400.urdf beside the package.
"""

from __future__ import annotations

import contextlib
import math
import os
import time
import warnings
from types import SimpleNamespace
from typing import Any

try:
    from .collision import CollisionGuardConfig, CollisionContact, contact_summaries, should_ignore_contact
except ImportError:
    from collision import CollisionGuardConfig, CollisionContact, contact_summaries, should_ignore_contact

try:
    from .kinematics import (
        MG400_API_C_R,
        MG400_API_C_Z,
        MG400_API_L1,
        MG400_API_L2,
        fk_mg400_api,
        ik_mg400_api,
    )
except ImportError:
    from kinematics import (
        MG400_API_C_R,
        MG400_API_C_Z,
        MG400_API_L1,
        MG400_API_L2,
        fk_mg400_api,
        ik_mg400_api,
    )
try:
    from .urdf_paths import resolve_mg400_urdf_path
except ImportError:
    from urdf_paths import resolve_mg400_urdf_path

# MuJoCo's passive viewer queries the GLFW window position on every sync().
# Wayland does not expose window positions, so GLFW raises this every call.
# It is non-fatal; suppress it so test output stays readable.
warnings.filterwarnings("ignore", message=r".*[Ww]ayland.*window position.*")


def _import_mujoco_viewer():
    """Import MuJoCo's viewer submodule explicitly."""
    try:
        import mujoco.viewer as mj_viewer
    except ImportError as exc:
        raise ImportError(
            "MuJoCo viewer is unavailable. Ensure the active environment can "
            "import `mujoco.viewer`."
        ) from exc
    return mj_viewer


L1_MG400     = MG400_API_L1   # mm — upper arm (calibrated from real-robot measurements)
L2_MG400     = MG400_API_L2   # mm — forearm   (calibrated)
Z_BASE_MG400 = 116.0   # mm — kept for student lab backward compat; not used in FK
C_R_MG400    = MG400_API_C_R   # mm — horizontal pivot offset (calibrated)
C_Z_MG400    = MG400_API_C_Z   # mm — vertical offset (calibrated)

SAFE_BOUNDS_MG400 = {
    "x": (60.0, 400.0),
    "y": (-220.0, 220.0),
    "z": (5.0, 140.0),
    "r": (-170.0, 170.0),
}
JOINT_BOUNDS_MG400 = {
    "j1": (-160.0, 160.0),
    "j2": (-25.0,  85.0),
    "j3": (-25.0, 105.0),   # firmware absolute; body-frame q[2] bounded implicitly
    "j4": (-180.0, 180.0),
}
READY_POSE_MG400 = (300.0, 0.0, 50.0, 0.0)


# ---------------------------------------------------------------------------
# Base class — pure-Python FK + IK (authoritative pose)
# ---------------------------------------------------------------------------

class _SimMG400Base:
    """Body-frame FK / IK and shared pydobotplus-free API.

    Angles stored as body-frame degrees; positions in mm. The `dashboard`
    and `move_api` surfaces mirror the three TCP objects the real MG400
    driver exposes.
    """

    L1     = L1_MG400
    L2     = L2_MG400
    Z_BASE = Z_BASE_MG400  # kept; not used in FK (student lab compat)
    C_R    = C_R_MG400
    C_Z    = C_Z_MG400

    _is_sim = True

    def __init__(self, collision_guard: bool | CollisionGuardConfig | None = None):
        self._collision_config = CollisionGuardConfig.from_value(collision_guard)
        self._collision_stopped = False
        self._last_collision: CollisionContact | None = None
        self._q = [0.0, 0.0, 0.0, 0.0]        # body-frame joint angles [deg]
        self._cartesian = self._fk(self._q)   # (x, y, z, r) mm / deg

    # -- Kinematics --------------------------------------------------------

    def _fk(self, q):
        """MG400 API-frame FK (mm/deg); see fk_mg400_api in kinematics."""
        return fk_mg400_api(q)

    def _ik(self, x, y, z, r, q0=None, branch="firmware"):
        """Analytic IK for API-frame pose; default branch matches hardware, optional branch='continuity' or 'opposite'."""
        q0 = q0 or self._q
        if branch != "firmware":
            return ik_mg400_api(x, y, z, r, q0=q0, branch=branch)
        q_preferred = ik_mg400_api(x, y, z, r, q0=q0, branch="firmware")
        if self._body_joints_are_valid(q_preferred):
            return q_preferred
        q_opposite = ik_mg400_api(x, y, z, r, q0=q0, branch="opposite")
        if self._body_joints_are_valid(q_opposite):
            return q_opposite
        return q_preferred

    @staticmethod
    def _clamp(v, lo, hi):
        return max(lo, min(hi, v))

    def _clamp_cartesian(self, x, y, z, r):
        b = SAFE_BOUNDS_MG400
        return (self._clamp(x, *b["x"]),
                self._clamp(y, *b["y"]),
                self._clamp(z, *b["z"]),
                self._clamp(r, *b["r"]))

    def _clamp_body_joints(self, q):
        """Clamp body-frame joints to MG400 firmware joint ranges."""
        j1_lo, j1_hi = JOINT_BOUNDS_MG400["j1"]
        j2_lo, j2_hi = JOINT_BOUNDS_MG400["j2"]
        j3_fw_lo, j3_fw_hi = JOINT_BOUNDS_MG400["j3"]
        j4_lo, j4_hi = JOINT_BOUNDS_MG400["j4"]
        q1 = self._clamp(float(q[0]), j1_lo, j1_hi)
        q2 = self._clamp(float(q[1]), j2_lo, j2_hi)
        j3_fw = self._clamp(q2 + float(q[2]), j3_fw_lo, j3_fw_hi)
        q3_body = j3_fw - q2
        q4 = self._clamp(float(q[3]), j4_lo, j4_hi)
        return [q1, q2, q3_body, q4]

    def _body_joints_are_valid(self, q) -> bool:
        clamped = self._clamp_body_joints(q)
        return all(abs(float(a) - float(b)) <= 1e-6 for a, b in zip(q, clamped))

    # -- URDF backend hook (overridden) -----------------------------------

    def _apply_joint_angles(self, q_deg) -> None:
        """Default no-op. URDF backends override to drive their simulator."""
        pass

    def get_collision_contacts(self) -> list[CollisionContact]:
        return []

    def is_collision_stopped(self) -> bool:
        return self._collision_stopped

    def clear_collision_stop(self) -> None:
        self._collision_stopped = False
        self._last_collision = None

    def collision_summary(self) -> str:
        if self._last_collision is None:
            return "Collision: OK" if self._collision_config.enabled else "Collision: OFF"
        return f"Collision: STOP {self._last_collision.summary()}"

    def _try_apply_motion(self, q_new, cartesian_new) -> bool:
        q_requested = list(q_new)
        q_new = self._clamp_body_joints(q_new)
        cartesian_actual = cartesian_new
        if any(abs(a - b) > 1e-9 for a, b in zip(q_requested, q_new)):
            cartesian_actual = self._fk(q_new)
        old_q = self._q[:]
        old_cartesian = self._cartesian
        self._apply_joint_angles(q_new)
        contacts = self.get_collision_contacts()
        if contacts and self._collision_config.stop_on_collision:
            self._apply_joint_angles(old_q)
            self._q = old_q
            self._cartesian = old_cartesian
            self._collision_stopped = True
            self._last_collision = contacts[0]
            print(f"[SimMG400] COLLISION STOP: {contact_summaries(contacts, limit=1)}")
            return False
        self._q = list(q_new)
        self._cartesian = cartesian_actual
        self._collision_stopped = False
        self._last_collision = None
        return True

    # -- MG400 move_api surface -------------------------------------------

    def MovJ(self, x, y, z, r, *args, **kwargs):
        x, y, z, r = self._clamp_cartesian(float(x), float(y), float(z), float(r))
        q_new = self._ik(x, y, z, r, q0=self._q)
        self._try_apply_motion(q_new, (x, y, z, r))
        print(f"[SimMG400] MovJ({x:.1f}, {y:.1f}, {z:.1f}, {r:.1f})")
        return "0,{},MovJ();"

    def MovL(self, x, y, z, r, *args, **kwargs):
        # Simulation: no path-planning — both MovJ and MovL teleport to target.
        # On real hardware MovL produces a straight Cartesian path; the motion
        # sequence (approach → descend → lift) still mirrors real usage.
        return self.MovJ(x, y, z, r, *args, **kwargs)

    def JointMovJ(self, j1, j2, j3, j4, *args, **kwargs):
        # MG400 firmware convention: j3_fw = q2 + q3 (absolute elbow angle).
        # Convert firmware → body-frame before storing, so the internal FK
        # formula (theta3 = q[1]+q[2]) stays consistent.
        j1 = self._clamp(float(j1), *JOINT_BOUNDS_MG400["j1"])
        j2 = self._clamp(float(j2), *JOINT_BOUNDS_MG400["j2"])
        j3 = self._clamp(float(j3), *JOINT_BOUNDS_MG400["j3"])
        j4 = self._clamp(float(j4), *JOINT_BOUNDS_MG400["j4"])
        q3_body = j3 - j2
        q_new = [j1, j2, q3_body, j4]
        self._try_apply_motion(q_new, self._fk(q_new))
        print(f"[SimMG400] JointMovJ({j1:.1f}, {j2:.1f}, {j3:.1f}, {j4:.1f})")
        return "0,{},JointMovJ();"

    def RelMovJ(self, dx, dy, dz, dr, *args, **kwargs):
        x0, y0, z0, r0 = self._cartesian
        return self.MovJ(x0 + float(dx), y0 + float(dy), z0 + float(dz), r0 + float(dr))

    def RelMovL(self, dx, dy, dz, dr, *args, **kwargs):
        return self.RelMovJ(dx, dy, dz, dr)

    def Sync(self):
        return "0,{},Sync();"

    def SpeedFactor(self, *args, **kwargs): return "0,{},SpeedFactor();"
    def SpeedJ(self, *args, **kwargs):      return "0,{},SpeedJ();"
    def SpeedL(self, *args, **kwargs):      return "0,{},SpeedL();"
    def AccJ(self, *args, **kwargs):        return "0,{},AccJ();"
    def AccL(self, *args, **kwargs):        return "0,{},AccL();"
    def DO(self, *args, **kwargs):          return "0,{},DO();"
    def ToolDO(self, index=1, state=0, *args, **kwargs):
        print(f"[SimMG400] ToolDO({index}, {state})")
        return "0,{},ToolDO();"

    # -- MG400 dashboard surface ------------------------------------------

    def EnableRobot(self):  return "0,{0},EnableRobot();"
    def DisableRobot(self): return "0,{0},DisableRobot();"
    def ClearError(self):   return "0,{0},ClearError();"
    def Continue(self):     return "0,{0},Continue();"
    def GetErrorID(self):   return "0,{[[],[]]},GetErrorID();"

    def GetPose(self):
        x, y, z, r = self._cartesian
        # MG400 dashboard pose string: "0,{x,y,z,rx,ry,rz},GetPose();"
        return f"0,{{{x:.4f},{y:.4f},{z:.4f},{r:.4f},0.0000,0.0000}},GetPose();"

    def GetAngle(self):
        # Real GetAngle() returns firmware angles: j3_fw = q2 + q3 (absolute).
        q1, q2, q3, q4 = self._q
        return f"0,{{{q1:.4f},{q2:.4f},{q2 + q3:.4f},{q4:.4f}}},GetAngle();"

    def RobotMode(self):
        return "0,{5},RobotMode();"     # 5 = ENABLE(idle)

    # -- Structured accessors (not part of the MG400 protocol, convenient) -

    def get_pose_tuple(self):
        return self._cartesian

    def get_joints_tuple(self):
        return tuple(self._q)

    def get_pose(self):
        """pydobotplus-style Pose for simulation code paths that prefer it."""
        x, y, z, r = self._cartesian
        j1, j2, j3, j4 = self._q
        return SimpleNamespace(
            position=SimpleNamespace(x=x, y=y, z=z, r=r),
            joints=SimpleNamespace(j1=j1, j2=j2, j3=j3, j4=j4),
        )

    # -- Smooth animation API (overridden by URDF backends) ----------------

    def interpolate_to(self, x, y, z, r=0, steps=30, pause=0.02, lock=None) -> None:
        """Smooth animated Cartesian move. Base: instant teleport fallback."""
        self.MovJ(x, y, z, r)

    def interpolate_joints_to(self, q_target, steps=30, pause=0.02, lock=None) -> None:
        """Smooth animated joint move. Base: instant teleport fallback."""
        self._q = self._clamp_body_joints(q_target)
        self._cartesian = self._fk(self._q)
        self._apply_joint_angles(self._q)

    def is_running(self) -> bool:
        return True

    def sync_viewer(self) -> None:
        pass


# ---------------------------------------------------------------------------
# Feedback stub (port 30004 emulation)
# ---------------------------------------------------------------------------

class _SimMG400Feed:
    """Minimal port-30004 stand-in. Holds a reference to the shared state."""

    def __init__(self, owner: "_SimMG400Base"):
        self._owner = owner

    @property
    def current_pose(self):
        x, y, z, r = self._owner.get_pose_tuple()
        return {"x": x, "y": y, "z": z, "r": r}

    def close(self): pass


# ---------------------------------------------------------------------------
# PyBullet backend
# ---------------------------------------------------------------------------

class _SimMG400PyBullet(_SimMG400Base):
    """PyBullet backend loading vendor/mg400_description/urdf/mg400.urdf."""

    JOINT_BASE      = "mg400_j1"
    JOINT_SHOULDER  = "mg400_j2_1"
    JOINT_SHOULDER2 = "mg400_j2_2"
    JOINT_ELBOW     = "mg400_j3_1"
    JOINT_ELBOW2    = "mg400_j3_2"
    JOINT_WRIST_P   = "mg400_j4_1"
    JOINT_WRIST_P2  = "mg400_j4_2"
    JOINT_TOOL      = "mg400_j5"
    EE_LINK         = "mg400_end_effector_flange"
    SIMLAB_JOINTS   = {
        "q1": "simlab_mg400_j1",
        "q2": "simlab_mg400_j2",
        "q3": "simlab_mg400_j3",
        "q4": "simlab_mg400_j4",
    }
    SIMLAB_TIP_LINK = "simlab_mg400_tip"

    def __init__(self, gui=True, collision_guard: bool | CollisionGuardConfig | None = None):
        super().__init__(collision_guard=collision_guard)
        _urdf = resolve_mg400_urdf_path()
        try:
            import pybullet as p
            import pybullet_data
        except ImportError as exc:
            raise ImportError("MG400 PyBullet backend requires: pip install pybullet") from exc

        try:
            from .urdf_loader import prepare_urdf_for_pybullet, apply_parallelogram_constraint_pybullet
        except ImportError:
            from urdf_loader import prepare_urdf_for_pybullet, apply_parallelogram_constraint_pybullet
        self._apply_constraint = apply_parallelogram_constraint_pybullet
        cache_dir = os.environ.get(
            "DOBOT_SIM_CACHE", os.path.join(os.path.expanduser("~"), ".cache", "dobot_sim")
        )
        _urdf_prepared = prepare_urdf_for_pybullet(_urdf, cache_dir)

        self._p = p
        self._client = p.connect(p.GUI if gui else p.DIRECT)
        p.setGravity(0, 0, -9.81, physicsClientId=self._client)
        p.setAdditionalSearchPath(pybullet_data.getDataPath(), physicsClientId=self._client)
        _plane_id = p.loadURDF("plane.urdf", physicsClientId=self._client)
        if not gui:   # headless/DIRECT: darken plane so it doesn't dominate camera view
            p.changeVisualShape(_plane_id, -1, rgbaColor=[0.15, 0.15, 0.15, 1.0],
                                physicsClientId=self._client)
        flags = p.URDF_MAINTAIN_LINK_ORDER
        if self._collision_config.enabled:
            flags |= getattr(p, "URDF_USE_SELF_COLLISION_EXCLUDE_PARENT", p.URDF_USE_SELF_COLLISION)
        self._robot_id = p.loadURDF(
            str(_urdf_prepared),
            useFixedBase=True,
            flags=flags,
            physicsClientId=self._client,
        )

        self._joint_idx = {}
        n = p.getNumJoints(self._robot_id, physicsClientId=self._client)
        for i in range(n):
            info = p.getJointInfo(self._robot_id, i, physicsClientId=self._client)
            self._joint_idx[info[1].decode()] = i
        self._link_names = {-1: "arm_frame_link_offset"}
        self._parent_links = {}
        for i in range(n):
            info = p.getJointInfo(self._robot_id, i, physicsClientId=self._client)
            self._link_names[i] = info[12].decode() or info[1].decode()
            self._parent_links[i] = int(info[16])
        ignored_pairs = {
            tuple(sorted((self._link_names.get(parent, str(parent)), self._link_names.get(child, str(child)))))
            for child, parent in self._parent_links.items()
        }
        ignored_pairs.update(self._collision_config.ignored_link_pairs)
        self._collision_config = CollisionGuardConfig(
            enabled=self._collision_config.enabled,
            stop_on_collision=self._collision_config.stop_on_collision,
            check_self=self._collision_config.check_self,
            check_environment=self._collision_config.check_environment,
            distance_threshold=self._collision_config.distance_threshold,
            ignored_name_prefixes=self._collision_config.ignored_name_prefixes,
            ignored_link_pairs=frozenset(ignored_pairs),
        )
        self._ee_link = self._joint_idx.get(self.JOINT_TOOL, n - 1)
        for name in self.SIMLAB_JOINTS.values():
            if name not in self._joint_idx:
                raise RuntimeError(
                    f"[SimMG400-PyBullet] Prepared URDF is missing simulator lab joint '{name}'."
                )
        self._simlab_tip_link = self._joint_idx[self.SIMLAB_JOINTS["q4"]]

        # Lock the parallelogram: only the shoulder pair shares a parent
        # (link1) and rotates together with a constant ratio.
        # The elbow and wrist pairs are driven by explicit angle mapping in
        # _apply_joint_angles because their relationships are not constant
        # ratios (j3_2 = -j2, j4_2 = +j3).
        self._apply_constraint(self._client, self._robot_id,
                               self.JOINT_SHOULDER, self.JOINT_SHOULDER2, ratio=1.0)
        self._payload_id = None   # PyBullet body ID of held object, or None
        print(
            f"[SimMG400-PyBullet] Loaded URDF: {n} joints "
            f"EE_link={self._ee_link} GUI={'on' if gui else 'off'}"
        )

    def ToolDO(self, index=1, state=0, *args, **kwargs):
        print(f"[SimMG400-PyBullet] ToolDO({index}, {state})")
        if int(index) == 1:
            if int(state) == 1:
                self._attach_payload()
            else:
                self._detach_payload()
        return "0,{},ToolDO();"

    def _ee_world_pos(self):
        state = self._p.getLinkState(
            self._robot_id, self._ee_link,
            computeForwardKinematics=True, physicsClientId=self._client)
        return list(state[0])

    def _attach_payload(self):
        if self._payload_id is not None:
            return
        p, cid = self._p, self._client
        pos = self._ee_world_pos()
        col = p.createCollisionShape(p.GEOM_SPHERE, radius=0.015, physicsClientId=cid)
        vis = p.createVisualShape(p.GEOM_SPHERE, radius=0.015,
                                  rgbaColor=[1.0, 0.4, 0.1, 1.0], physicsClientId=cid)
        self._payload_id = p.createMultiBody(
            baseMass=0.0, baseCollisionShapeIndex=col,
            baseVisualShapeIndex=vis, basePosition=pos, physicsClientId=cid)
        xc, yc, zc, _ = self._cartesian
        print(f"[SimMG400-PyBullet] suction ON  - FK ({xc:.1f}, {yc:.1f}, {zc:.1f}) mm")

    def _detach_payload(self):
        if self._payload_id is None:
            return
        self._p.removeBody(self._payload_id, physicsClientId=self._client)
        print("[SimMG400-PyBullet] suction OFF - payload removed")
        self._payload_id = None

    def _update_payload_position(self):
        if self._payload_id is None:
            return
        pos = self._ee_world_pos()
        self._p.resetBasePositionAndOrientation(
            self._payload_id, pos, [0, 0, 0, 1], physicsClientId=self._client)

    def _apply_joint_angles(self, q_deg) -> None:
        """Drive URDF joints using the same mapping as the RViz teleop."""
        p   = self._p
        cid = self._client
        j1, j2, j3_body, j4 = q_deg
        j1_r = math.radians(j1)
        j2_r = math.radians(j2)
        j3_abs_r = math.radians(j2 + j3_body)   # firmware absolute elbow
        j4_r = math.radians(j4)
        mapping = {
            self.JOINT_BASE:      j1_r,
            self.JOINT_SHOULDER:  j2_r,
            self.JOINT_SHOULDER2: j2_r,
            self.JOINT_ELBOW:     math.radians(j3_body),  # j3 - j2
            self.JOINT_ELBOW2:    -j2_r,                   # -j2
            self.JOINT_WRIST_P:   -j3_abs_r,               # -j3
            self.JOINT_WRIST_P2:  j3_abs_r,                # +j3
            self.JOINT_TOOL:      j4_r,
            self.SIMLAB_JOINTS["q1"]: j1_r,
            self.SIMLAB_JOINTS["q2"]: j2_r,
            self.SIMLAB_JOINTS["q3"]: math.radians(j3_body),
            self.SIMLAB_JOINTS["q4"]: j4_r,
        }
        for name, angle in mapping.items():
            idx = self._joint_idx.get(name)
            if idx is not None:
                p.resetJointState(self._robot_id, idx, angle, physicsClientId=cid)
        p.stepSimulation(physicsClientId=cid)
        self._update_payload_position()

    def step(self, n: int = 1) -> None:
        for _ in range(int(n)):
            self._p.stepSimulation(physicsClientId=self._client)

    def get_sim_ee_pose(self):
        state = self._p.getLinkState(
            self._robot_id,
            self._simlab_tip_link,
            computeForwardKinematics=True,
            physicsClientId=self._client,
        )
        x, y, z = state[0]
        r = math.degrees(
            self._p.getJointState(
                self._robot_id,
                self._joint_idx[self.SIMLAB_JOINTS["q4"]],
                physicsClientId=self._client,
            )[0]
        )
        return (x * 1000.0, y * 1000.0, z * 1000.0, float(r))

    def get_sim_ee_position(self):
        return self.get_sim_ee_pose()[:3]

    def get_collision_contacts(self) -> list[CollisionContact]:
        if not self._collision_config.enabled or not self._collision_config.check_self:
            return []
        contacts = []
        for c in self._p.getContactPoints(self._robot_id, self._robot_id, physicsClientId=self._client):
            link_a = self._link_names.get(c[3], str(c[3]))
            link_b = self._link_names.get(c[4], str(c[4]))
            if should_ignore_contact(link_a, link_b, self._collision_config):
                continue
            contacts.append(
                CollisionContact(
                    backend="pybullet",
                    kind="self",
                    body_a="mg400",
                    body_b="mg400",
                    link_a=link_a,
                    link_b=link_b,
                    distance=float(c[8]),
                    normal_force=float(c[9]),
                )
            )
        return contacts

    def interpolate_to(self, x, y, z, r=0, steps=30, pause=0.02, lock=None) -> None:
        """Smooth animated Cartesian move (uses analytic calibrated IK, matches MovJ).

        If *lock* is provided it is acquired per-step and released during sleep,
        allowing a concurrent viewport thread to render frames mid-move.
        """
        tx, ty, tz, tr = self._clamp_cartesian(float(x), float(y), float(z), float(r))
        q_raw = self._ik(tx, ty, tz, tr, q0=self._q)
        q_target = self._clamp_body_joints(q_raw)
        target_reachable = all(abs(a - b) <= 1e-6 for a, b in zip(q_raw, q_target))
        q_start  = self._q[:]
        ctx = lock if lock is not None else contextlib.nullcontext()
        for i in range(1, steps + 1):
            t   = i / steps
            q_i = [q_start[j] + t * (q_target[j] - q_start[j]) for j in range(4)]
            with ctx:
                if not self._try_apply_motion(q_i, self._fk(q_i)):
                    return
            time.sleep(pause)
        if target_reachable:
            with ctx:
                self._cartesian = (tx, ty, tz, tr)

    def interpolate_joints_to(self, q_target, steps=30, pause=0.02, lock=None) -> None:
        """Smooth animated joint move.

        If *lock* is provided it is acquired per-step and released during sleep.
        """
        q_start = self._q[:]
        ctx = lock if lock is not None else contextlib.nullcontext()
        for i in range(1, steps + 1):
            t   = i / steps
            q_i = [q_start[j] + t * (q_target[j] - q_start[j]) for j in range(4)]
            with ctx:
                if not self._try_apply_motion(q_i, self._fk(q_i)):
                    return
            time.sleep(pause)

    def is_running(self) -> bool:
        try:
            return bool(self._p.isConnected(self._client))
        except Exception:
            return False

    def sync_viewer(self) -> None:
        pass  # PyBullet GUI runs in its own thread

    def close(self):
        self._detach_payload()
        try:
            self._p.disconnect(self._client)
        except Exception:
            pass
        print("[SimMG400-PyBullet] Disconnected.")


# ---------------------------------------------------------------------------
# MuJoCo backend
# ---------------------------------------------------------------------------

class _SimMG400MuJoCo(_SimMG400Base):
    """MuJoCo backend loading vendor/mg400_description/urdf/mg400.urdf."""

    JOINT_BASE      = "mg400_j1"
    JOINT_SHOULDER  = "mg400_j2_1"
    JOINT_SHOULDER2 = "mg400_j2_2"
    JOINT_ELBOW     = "mg400_j3_1"
    JOINT_ELBOW2    = "mg400_j3_2"
    JOINT_WRIST_P   = "mg400_j4_1"
    JOINT_WRIST_P2  = "mg400_j4_2"
    JOINT_TOOL      = "mg400_j5"
    EE_BODY         = "mg400_end_effector_flange"
    SIMLAB_JOINTS   = {
        "q1": "simlab_mg400_j1",
        "q2": "simlab_mg400_j2",
        "q3": "simlab_mg400_j3",
        "q4": "simlab_mg400_j4",
    }
    SIMLAB_TIP_BODY = "simlab_mg400_tip"

    def __init__(self, gui=True, collision_guard: bool | CollisionGuardConfig | None = None):
        super().__init__(collision_guard=collision_guard)
        # Do not force EGL for GUI windows; MuJoCo's passive viewer owns GLFW,
        # and mixing them can crash some Linux/OpenGL drivers during shutdown.
        if not gui:
            os.environ.setdefault("MUJOCO_GL", "egl")
        _urdf = resolve_mg400_urdf_path()
        try:
            import mujoco
        except ImportError as exc:
            raise ImportError("MG400 MuJoCo backend requires: pip install mujoco") from exc

        try:
            from .urdf_loader import prepare_urdf_for_mujoco, EqualityConstraint
        except ImportError:
            from urdf_loader import prepare_urdf_for_mujoco, EqualityConstraint
        cache_dir = os.environ.get(
            "DOBOT_SIM_CACHE", os.path.join(os.path.expanduser("~"), ".cache", "dobot_sim")
        )
        # Parallelogram bindings baked in at compile time.
        # NOTE: Only the shoulder pair uses a constant-ratio equality.
        # j3_2 and j4_1/j4_2 are driven by correct angle mapping in
        # _apply_joint_angles because their relationships are not fixed ratios.
        equalities = [
            EqualityConstraint(driver=self.JOINT_SHOULDER, follower=self.JOINT_SHOULDER2, multiplier=1.0),
        ]
        prepared = prepare_urdf_for_mujoco(_urdf, cache_dir, equalities=equalities)

        self._mj    = mujoco
        self._model = mujoco.MjModel.from_xml_path(str(prepared))
        self._data  = mujoco.MjData(self._model)
        self._viewer = None
        self._viewer_mod = None

        self._qposadr = {}
        for name in (self.JOINT_BASE, self.JOINT_SHOULDER, self.JOINT_SHOULDER2,
                     self.JOINT_ELBOW, self.JOINT_ELBOW2,
                     self.JOINT_WRIST_P, self.JOINT_WRIST_P2, self.JOINT_TOOL,
                     *self.SIMLAB_JOINTS.values()):
            try:
                self._qposadr[name] = int(self._model.joint(name).qposadr[0])
            except Exception:
                pass
        required = [
            self.JOINT_BASE,
            self.JOINT_SHOULDER,
            self.JOINT_SHOULDER2,
            self.JOINT_ELBOW,
            self.JOINT_ELBOW2,
            self.JOINT_WRIST_P,
            self.JOINT_WRIST_P2,
            self.JOINT_TOOL,
            *self.SIMLAB_JOINTS.values(),
        ]
        missing = [name for name in required if name not in self._qposadr]
        if missing:
            raise RuntimeError(
                f"[SimMG400-MuJoCo] Prepared URDF is missing required joints: {missing}"
            )
        self._ee_body_id = int(self._model.body(self.EE_BODY).id)
        self._simlab_tip_body_id = int(self._model.body(self.SIMLAB_TIP_BODY).id)
        self._body_names = {
            i: (self._mj.mj_id2name(self._model, self._mj.mjtObj.mjOBJ_BODY, i) or str(i))
            for i in range(self._model.nbody)
        }
        ignored_pairs = {
            tuple(sorted((self._body_names.get(int(parent), str(parent)), self._body_names.get(i, str(i)))))
            for i, parent in enumerate(self._model.body_parentid)
            if i != 0
        }
        ignored_pairs.update(self._collision_config.ignored_link_pairs)
        ignored_pairs.update({
            tuple(sorted(pair))
            for pair in (
                ("mg400_link2_1", "mg400_link3_2"),
                ("mg400_link3_1", "mg400_link2_2"),
                ("mg400_link3_1", "mg400_link3_2"),
                ("mg400_link4_1", "mg400_link4_2"),
            )
        })
        self._collision_config = CollisionGuardConfig(
            enabled=self._collision_config.enabled,
            stop_on_collision=self._collision_config.stop_on_collision,
            check_self=self._collision_config.check_self,
            check_environment=self._collision_config.check_environment,
            distance_threshold=self._collision_config.distance_threshold,
            ignored_name_prefixes=self._collision_config.ignored_name_prefixes,
            ignored_link_pairs=frozenset(ignored_pairs),
        )

        if gui:
            self._viewer_mod = _import_mujoco_viewer()
            self._viewer = self._viewer_mod.launch_passive(self._model, self._data)
        print(
            f"[SimMG400-MuJoCo] Loaded URDF: nv={self._model.nv} "
            f"EE_body='{self.EE_BODY}' GUI={'on' if gui else 'off'}"
        )

    def _set_qpos(self, name: str, angle_rad: float):
        adr = self._qposadr.get(name)
        if adr is not None:
            self._data.qpos[adr] = angle_rad

    def _apply_joint_angles(self, q_deg, *, forward: bool = False) -> None:
        """Drive URDF joints using the same mapping as the RViz teleop."""
        mj = self._mj
        j1, j2, j3_body, j4 = q_deg
        j1_r = math.radians(j1)
        j2_r = math.radians(j2)
        j3_abs_r = math.radians(j2 + j3_body)   # firmware absolute elbow
        j4_r = math.radians(j4)
        self._set_qpos(self.JOINT_BASE,      j1_r)
        self._set_qpos(self.JOINT_SHOULDER,  j2_r)
        self._set_qpos(self.JOINT_SHOULDER2, j2_r)
        self._set_qpos(self.JOINT_ELBOW,     math.radians(j3_body))  # j3 - j2
        self._set_qpos(self.JOINT_ELBOW2,    -j2_r)                   # -j2
        self._set_qpos(self.JOINT_WRIST_P,   -j3_abs_r)               # -j3
        self._set_qpos(self.JOINT_WRIST_P2,  j3_abs_r)                # +j3
        self._set_qpos(self.JOINT_TOOL,      j4_r)
        self._set_qpos(self.SIMLAB_JOINTS["q1"], j1_r)
        self._set_qpos(self.SIMLAB_JOINTS["q2"], j2_r)
        self._set_qpos(self.SIMLAB_JOINTS["q3"], math.radians(j3_body))
        self._set_qpos(self.SIMLAB_JOINTS["q4"], j4_r)
        if forward:
            mj.mj_forward(self._model, self._data)
        else:
            mj.mj_kinematics(self._model, self._data)
        if self._viewer is not None:
            self._viewer.sync()

    def step(self, n: int = 1) -> None:
        for _ in range(int(n)):
            self._mj.mj_step(self._model, self._data)
        if self._viewer is not None:
            self._viewer.sync()

    def get_sim_ee_pose(self):
        self._mj.mj_kinematics(self._model, self._data)
        x, y, z = self._data.xpos[self._simlab_tip_body_id]
        r = math.degrees(self._data.qpos[self._qposadr[self.SIMLAB_JOINTS["q4"]]])
        return (float(x) * 1000.0, float(y) * 1000.0, float(z) * 1000.0, float(r))

    def get_sim_ee_position(self):
        return self.get_sim_ee_pose()[:3]

    def get_collision_contacts(self) -> list[CollisionContact]:
        if not self._collision_config.enabled or not self._collision_config.check_self:
            return []
        self._mj.mj_forward(self._model, self._data)
        contacts = []
        for i in range(int(self._data.ncon)):
            c = self._data.contact[i]
            body_a = int(self._model.geom_bodyid[c.geom1])
            body_b = int(self._model.geom_bodyid[c.geom2])
            name_a = self._body_names.get(body_a, str(body_a))
            name_b = self._body_names.get(body_b, str(body_b))
            if should_ignore_contact(name_a, name_b, self._collision_config):
                continue
            contacts.append(
                CollisionContact(
                    backend="mujoco",
                    kind="self",
                    body_a="mg400",
                    body_b="mg400",
                    link_a=name_a,
                    link_b=name_b,
                    distance=float(c.dist),
                )
            )
        return contacts

    def _try_apply_motion(self, q_new, cartesian_new) -> bool:
        q_requested = list(q_new)
        q_new = self._clamp_body_joints(q_new)
        cartesian_actual = cartesian_new
        if any(abs(a - b) > 1e-9 for a, b in zip(q_requested, q_new)):
            cartesian_actual = self._fk(q_new)
        old_q = self._q[:]
        old_cartesian = self._cartesian
        self._apply_joint_angles(q_new, forward=self._collision_config.enabled)
        contacts = self.get_collision_contacts()
        if contacts and self._collision_config.stop_on_collision:
            self._apply_joint_angles(old_q, forward=self._collision_config.enabled)
            self._q = old_q
            self._cartesian = old_cartesian
            self._collision_stopped = True
            self._last_collision = contacts[0]
            print(f"[SimMG400-MuJoCo] COLLISION STOP: {contact_summaries(contacts, limit=1)}")
            return False
        self._q = list(q_new)
        self._cartesian = cartesian_actual
        self._collision_stopped = False
        self._last_collision = None
        return True

    def interpolate_to(self, x, y, z, r=0, steps=30, pause=0.02, lock=None) -> None:
        """Smooth animated Cartesian move via analytical IK + joint interpolation.

        If *lock* is provided it is acquired per-step and released during sleep.
        """
        tx, ty, tz, tr = self._clamp_cartesian(float(x), float(y), float(z), float(r))
        q_raw = self._ik(tx, ty, tz, tr, q0=self._q)
        q_target = self._clamp_body_joints(q_raw)
        target_reachable = all(abs(a - b) <= 1e-6 for a, b in zip(q_raw, q_target))
        q_start  = self._q[:]
        ctx = lock if lock is not None else contextlib.nullcontext()
        for i in range(1, steps + 1):
            t   = i / steps
            q_i = [q_start[j] + t * (q_target[j] - q_start[j]) for j in range(4)]
            with ctx:
                if not self._try_apply_motion(q_i, self._fk(q_i)):
                    return
            time.sleep(pause)
        if target_reachable:
            with ctx:
                self._cartesian = (tx, ty, tz, tr)

    def interpolate_joints_to(self, q_target, steps=30, pause=0.02, lock=None) -> None:
        """Smooth animated joint move.

        If *lock* is provided it is acquired per-step and released during sleep.
        """
        q_start = self._q[:]
        ctx = lock if lock is not None else contextlib.nullcontext()
        for i in range(1, steps + 1):
            t   = i / steps
            q_i = [q_start[j] + t * (q_target[j] - q_start[j]) for j in range(4)]
            with ctx:
                if not self._try_apply_motion(q_i, self._fk(q_i)):
                    return
            time.sleep(pause)

    def is_running(self) -> bool:
        if self._viewer is not None:
            return bool(self._viewer.is_running())
        return getattr(self, "_model", None) is not None

    def sync_viewer(self) -> None:
        if self._viewer is not None and self._viewer.is_running():
            self._viewer.sync()

    def close(self):
        viewer = self._viewer
        if viewer is not None:
            try:
                viewer.close()
                deadline = time.time() + 5.0
                while viewer.is_running() and time.time() < deadline:
                    time.sleep(0.02)
            except Exception:
                pass
            self._viewer = None
        # Nullify before Python GC to avoid MuJoCo/GLFW use-after-free on exit
        self._data  = None
        self._model = None
        print("[SimMG400-MuJoCo] Closed.")


# ---------------------------------------------------------------------------
# Factory
# ---------------------------------------------------------------------------

class SimMG400:
    """Build a shared SimMG400 backend; connect() returns (dashboard, move_api, feed) like the real driver."""

    def __init__(
        self,
        backend: str | None = None,
        gui: bool | None = None,
        collision_guard: bool | CollisionGuardConfig | None = None,
    ):
        backend = (backend or os.environ.get("DOBOT_SIM_BACKEND", "mujoco")).lower()
        if gui is None:
            gui = (os.environ.get("DOBOT_SIM_GUI", "1") != "0"
                   and os.environ.get("DOBOT_VIZ", "1") != "0")
        if backend == "pybullet":
            self._impl = _SimMG400PyBullet(gui=gui, collision_guard=collision_guard)
        elif backend == "mujoco":
            self._impl = _SimMG400MuJoCo(gui=gui, collision_guard=collision_guard)
        else:
            raise ValueError(
                f"Unknown SimMG400 backend '{backend}'. Expected: pybullet, mujoco."
            )
        self._feed = _SimMG400Feed(self._impl)

    def connect(self):
        """Return (dashboard, move_api, feed) sharing the same backend state."""
        return self._impl, self._impl, self._feed

    def interpolate_to(self, x, y, z, r=0, steps=30, pause=0.02, lock=None) -> None:
        self._impl.interpolate_to(x, y, z, r, steps=steps, pause=pause, lock=lock)

    def interpolate_joints_to(self, q_target, steps=30, pause=0.02, lock=None) -> None:
        self._impl.interpolate_joints_to(q_target, steps=steps, pause=pause, lock=lock)

    def is_running(self) -> bool:
        return self._impl.is_running()

    def sync_viewer(self) -> None:
        self._impl.sync_viewer()

    def get_collision_contacts(self) -> list[CollisionContact]:
        return self._impl.get_collision_contacts()

    def clear_collision_stop(self) -> None:
        self._impl.clear_collision_stop()

    def is_collision_stopped(self) -> bool:
        return self._impl.is_collision_stopped()

    def collision_summary(self) -> str:
        return self._impl.collision_summary()

    def close(self):
        self._impl.close()


# ---------------------------------------------------------------------------
# Smoke test (pure-Python FK, runnable with no extras installed)
# ---------------------------------------------------------------------------

if __name__ == "__main__":
    import math as _math
    bot = _SimMG400Base()
    # Expected values from calibrated FK (C_R=108, C_Z=-52, L1=174.5, L2=175.5).
    # Real-robot "joint_zero" measurement: X=Y=284.6, Z=121.2 (RMSE < 0.05 mm).
    configs = [
        ([0,  0,   0,  0], (283.5,   0.0, 122.5, 0.0)),   # J2=0, J3_abs=0
        ([0, 30, -30,  0], (370.8,   0.0,  99.1, 0.0)),   # J3_abs=0, J2=30°
        ([45, 0,   0,  0], (283.5 / _math.sqrt(2),
                            283.5 / _math.sqrt(2), 122.5, 45.0)),  # base-rotated, R=J1+J4
    ]
    print("MG400 FK smoke test (calibrated formula):")
    all_ok = True
    for q, (xe, ye, ze, re) in configs:
        bot._q = q[:]
        x, y, z, r = bot._fk(q)
        ok = abs(x - xe) < 1.0 and abs(y - ye) < 1.0 and abs(z - ze) < 1.0 and abs(r - re) < 0.1
        status = "PASS" if ok else "FAIL"
        if not ok:
            all_ok = False
        print(f"  {status}  q={q}  X={x:.2f} Y={y:.2f} Z={z:.2f} R={r:.2f}"
              f"  (expected X={xe:.2f} Y={ye:.2f} Z={ze:.2f} R={re:.2f})")
    print("sim_mg400.py:", "FK OK" if all_ok else "FK MISMATCH")

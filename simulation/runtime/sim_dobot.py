"""Magician simulation: SimDobot mirrors the pydobotplus Dobot API for ME403 labs.

Default backend is MuJoCo URDF simulation; PyBullet is the optional alternate
backend. Activate with DOBOT_SIMULATION=1 from lab helpers; tune backend with
DOBOT_SIM_BACKEND.

End-effector URDF variants (none/motor/suction) follow DOBOT_EE or constructor
arguments. Override URDF with DOBOT_MAGICIAN_URDF[_PATH]; otherwise loaders
search vendor/magician_ros2_urdf/urdf/ near the package. Bootstrap and mesh setup:
dobot-robotics-lab/simulation/README.md and docs/simulation.md.
"""

import contextlib
import math
import os
import time
import warnings
from types import SimpleNamespace

try:
    from .collision import CollisionGuardConfig, CollisionContact, contact_summaries, should_ignore_contact
except ImportError:
    from collision import CollisionGuardConfig, CollisionContact, contact_summaries, should_ignore_contact
try:
    from .magician_joint_mapping import firmware_deg_to_visual_rad
except ImportError:
    from magician_joint_mapping import firmware_deg_to_visual_rad
try:
    from .kinematics import fk_magician_firmware, ik_magician_firmware, MAGICIAN_TOOL_OFFSET
except ImportError:
    from kinematics import fk_magician_firmware, ik_magician_firmware, MAGICIAN_TOOL_OFFSET
try:
    from .urdf_paths import resolve_magician_urdf_path
except ImportError:
    from urdf_paths import resolve_magician_urdf_path

# MuJoCo's passive viewer queries the GLFW window position on every sync().
# Wayland does not expose window positions, so GLFW raises this every call.
# It is non-fatal; suppress it so test output stays readable.
warnings.filterwarnings("ignore", message=r".*[Ww]ayland.*window position.*")

# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def _import_mujoco_viewer():
    """Import MuJoCo's viewer submodule explicitly.

    The top-level `mujoco` module does not always populate `mujoco.viewer`,
    even when the submodule is installed and importable.
    """
    try:
        import mujoco.viewer as mj_viewer
    except ImportError as exc:
        raise ImportError(
            "MuJoCo viewer is unavailable. Ensure the active environment can "
            "import `mujoco.viewer`."
        ) from exc
    return mj_viewer


def _show_visual_geoms_only(viewer) -> None:
    """Hide MuJoCo collision proxies in the viewer without disabling physics."""
    try:
        viewer.opt.geomgroup[0] = 0
        viewer.opt.geomgroup[1] = 1
    except Exception:
        pass


class _FallbackMOVJAngleMode:
    """Stand-in MODE_PTP.MOVJ_ANGLE when pydobotplus is unavailable."""

    __slots__ = ()

    def __str__(self):
        return "MOVJ_ANGLE"


_FALLBACK_MOVJ_ANGLE_MODE = _FallbackMOVJAngleMode()


def _is_joint_mode(mode) -> bool:
    """Return True if mode signals MOVJ_ANGLE (joint-space move)."""
    if mode is None:
        return False
    # Try pydobotplus enum directly (works if pydobotplus is installed)
    try:
        from pydobotplus.dobotplus import MODE_PTP
        if mode == MODE_PTP.MOVJ_ANGLE:
            return True
    except (ImportError, AttributeError):
        pass
    # Fallback: match by name string (works with any enum-like object)
    return "ANGLE" in str(mode).upper()


def _make_pose(x, y, z, r, j1, j2, j3, j4):
    """Build a pose object whose attributes match pydobotplus Pose exactly."""
    return SimpleNamespace(
        position=SimpleNamespace(x=float(x), y=float(y), z=float(z), r=float(r)),
        joints=SimpleNamespace(j1=float(j1), j2=float(j2), j3=float(j3), j4=float(j4)),
    )


# ---------------------------------------------------------------------------
# Base class
# ---------------------------------------------------------------------------

class _SimDobotBase:
    """Firmware-degree Magician pose state with pydobotplus-compatible methods.

    Tool mode none|motor|suction selects TCP offsets and URDF variant; see
    kinematics.MAGICIAN_TOOL_OFFSET and dobot-robotics-lab/docs/simulation.md.
    """

    L1 = 135.0    # mm — upper arm (J2 pivot → elbow)
    L2 = 147.0    # mm — forearm  (elbow → wrist axis / J4)

    # Sentinel so teardown() in utils_sim.py can reliably distinguish SimDobot
    # from a real pydobotplus.Dobot without depending on private attributes.
    _is_sim = True

    def __init__(
        self,
        collision_guard: bool | CollisionGuardConfig | None = None,
        tool: str = "none",
    ):
        if tool not in MAGICIAN_TOOL_OFFSET:
            raise ValueError(
                f"Unknown Magician tool {tool!r}; expected one of "
                f"{sorted(MAGICIAN_TOOL_OFFSET)}"
            )
        self._tool = tool
        self._collision_config = CollisionGuardConfig.from_value(collision_guard)
        self._collision_stopped = False
        self._last_collision: CollisionContact | None = None
        self._q         = [0.0, 0.0, 0.0, 0.0]     # firmware angles [deg]
        self._cartesian = self._fk(self._q)          # (x,y,z,r) in mm/deg

    # -- Forward kinematics -------------------------------------------------

    def _fk(self, q):
        """Firmware FK via kinematics.fk_magician_firmware; q=[J1..J4] deg; respects self._tool."""
        return fk_magician_firmware(
            float(q[0]), float(q[1]), float(q[2]),
            j4_deg=0.0, L1=self.L1, L2=self.L2, tool=self._tool,
        )

    # -- pydobotplus-compatible API -----------------------------------------

    def get_pose(self):
        """Current pose as pydobotplus-compatible object (xyzr + joints j1–j4)."""
        x, y, z, r = self._cartesian
        j1, j2, j3, j4 = self._q
        return _make_pose(x, y, z, r, j1, j2, j3, j4)

    def get_alarms(self):
        return []

    def clear_alarms(self):
        pass

    def suck(self, state: bool) -> None:
        print(f"[SimDobot] suck({'ON' if state else 'OFF'})")

    def _update_payload_position(self) -> None:
        pass  # overridden in PyBullet backend

    def interpolate_to(self, x, y, z, r=0, steps=30, pause=0.02) -> None:
        """Animated Cartesian move. Override in physics backends for smooth rendering."""
        self.move_to(x, y, z, r)

    def interpolate_joints_to(self, q_target, steps=30, pause=0.02) -> None:
        """Animated joint move. Override in physics backends for smooth rendering."""
        try:
            from pydobotplus.dobotplus import MODE_PTP
            mode = MODE_PTP.MOVJ_ANGLE
        except (ImportError, AttributeError):
            mode = _FALLBACK_MOVJ_ANGLE_MODE
        try:
            self.move_to(
                float(q_target[0]),
                float(q_target[1]),
                float(q_target[2]),
                float(q_target[3]),
                mode=mode,
            )
        except NotImplementedError:
            warnings.warn(
                "interpolate_joints_to skipped: subclass does not implement move_to.",
                UserWarning,
                stacklevel=2,
            )

    def is_running(self) -> bool:
        return True

    def sync_viewer(self) -> None:
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

    def close(self):
        print("[SimDobot] Simulation closed.")

    def move_to(self, x=None, y=None, z=None, r=0, wait=True, mode=None):
        raise NotImplementedError("Subclass must implement move_to()")


# ---------------------------------------------------------------------------
# PyBullet backend
# ---------------------------------------------------------------------------

class _SimDobotPyBullet(_SimDobotBase):
    """Magician URDF in PyBullet; firmware angles drive named revolute joints (see magician_joint_mapping)."""

    # jkaniuka/magician_ros2 visible-chain joint names. The mimic_1 / mimic_2
    # joints model the parallelogram passive constraint; URDF has <mimic> tags
    # but PyBullet ignores them — so they're driven explicitly from software.
    JOINT_BASE     = "magician_joint_1"
    JOINT_SHOULDER = "magician_joint_2"
    JOINT_FOREARM  = "magician_joint_3"
    JOINT_WRIST    = "magician_joint_4"
    JOINT_MIMIC_1  = "magician_joint_mimic_1"
    JOINT_MIMIC_2  = "magician_joint_mimic_2"
    EE_LINK_BY_TOOL = {
        "none": "magician_link_ee",
        "motor": "magician_link_ee",
        "suction": "magician_link_suction_cup",
    }
    SIMLAB_JOINTS  = {
        "q1": "simlab_magician_j1",
        "q2": "simlab_magician_j2",
        "q3": "simlab_magician_j3",
        "q4": "simlab_magician_j4",
    }
    SIMLAB_TIP_LINK = "simlab_magician_tip"
    def __init__(
        self,
        gui=True,
        collision_guard: bool | CollisionGuardConfig | None = None,
        tool: str = "none",
    ):
        super().__init__(collision_guard=collision_guard, tool=tool)
        _urdf = resolve_magician_urdf_path(tool=self._tool)
        self.EE_LINK_NAME = self.EE_LINK_BY_TOOL[self._tool]
        try:
            import pybullet as p
            import pybullet_data
        except ImportError as exc:
            raise ImportError("PyBullet backend requires: pip install pybullet") from exc

        try:
            from .urdf_loader import prepare_urdf_for_pybullet
        except ImportError:
            from urdf_loader import prepare_urdf_for_pybullet
        cache_dir = os.environ.get(
            "DOBOT_SIM_CACHE", os.path.join(os.path.expanduser("~"), ".cache", "dobot_sim")
        )
        _urdf_prepared = prepare_urdf_for_pybullet(_urdf, cache_dir)

        self._p = p
        self._client = p.connect(p.GUI if gui else p.DIRECT)
        p.setGravity(0, 0, -9.81, physicsClientId=self._client)
        p.setAdditionalSearchPath(pybullet_data.getDataPath(), physicsClientId=self._client)
        p.loadURDF("plane.urdf", physicsClientId=self._client)
        flags = p.URDF_MAINTAIN_LINK_ORDER
        if self._collision_config.enabled:
            flags |= getattr(p, "URDF_USE_SELF_COLLISION_EXCLUDE_PARENT", p.URDF_USE_SELF_COLLISION)
        self._robot_id = p.loadURDF(
            str(_urdf_prepared),
            useFixedBase=True,
            flags=flags,
            physicsClientId=self._client,
        )

        self._joint_idx = self._build_joint_index_map()
        self._link_names, self._parent_links = self._build_link_maps()
        self._link_idx = self._build_link_index_map()
        self._collision_config = self._with_ignored_parent_pairs(self._collision_config)
        required = [self.JOINT_BASE, self.JOINT_SHOULDER, self.JOINT_FOREARM,
                    self.JOINT_WRIST, self.JOINT_MIMIC_1, self.JOINT_MIMIC_2,
                    *self.SIMLAB_JOINTS.values()]
        missing = [n for n in required if n not in self._joint_idx]
        if missing:
            raise RuntimeError(
                f"[SimDobot-PyBullet] URDF is missing required joints: {missing}. "
                f"Found: {sorted(self._joint_idx)}"
            )
        if self.EE_LINK_NAME not in self._link_idx:
            raise RuntimeError(
                f"[SimDobot-PyBullet] URDF is missing EE link {self.EE_LINK_NAME!r} "
                f"for tool={self._tool!r}. Found links: {sorted(self._link_idx)}"
            )
        # EE link drives IK targets and the suck() payload attachment.
        # In PyBullet, link N is reached via the joint that has it as child;
        # _build_link_index_map gives us {child_link_name: joint_index}.
        self._ee_link = self._link_idx[self.EE_LINK_NAME]
        self._simlab_tip_link = self._joint_idx[self.SIMLAB_JOINTS["q4"]]
        self._payload_id = None   # PyBullet body ID of held object, or None
        self._ik_lower, self._ik_upper, self._ik_ranges, self._ik_rest = self._build_ik_limits()
        n_joints = p.getNumJoints(self._robot_id, physicsClientId=self._client)
        print(
            f"[SimDobot-PyBullet] Loaded URDF: {n_joints} joints "
            f"EE_link={self._ee_link} GUI={'on' if gui else 'off'}"
        )

    def _build_joint_index_map(self) -> dict:
        """Return {joint_name: pybullet_joint_index} for every joint in the URDF."""
        p = self._p
        n = p.getNumJoints(self._robot_id, physicsClientId=self._client)
        out = {}
        for i in range(n):
            info = p.getJointInfo(self._robot_id, i, physicsClientId=self._client)
            out[info[1].decode()] = i
        return out

    def _build_link_index_map(self) -> dict:
        """Return {child_link_name: pybullet_link_index} for every joint child."""
        p = self._p
        n = p.getNumJoints(self._robot_id, physicsClientId=self._client)
        out: dict[str, int] = {}
        for i in range(n):
            info = p.getJointInfo(self._robot_id, i, physicsClientId=self._client)
            child_link_name = info[12].decode()
            if child_link_name:
                out[child_link_name] = i
        return out

    def _build_link_maps(self):
        p = self._p
        n = p.getNumJoints(self._robot_id, physicsClientId=self._client)
        link_names = {-1: "base_link"}
        parent_links = {}
        for i in range(n):
            info = p.getJointInfo(self._robot_id, i, physicsClientId=self._client)
            link_names[i] = info[12].decode() or info[1].decode()
            parent_links[i] = int(info[16])
        return link_names, parent_links

    def _with_ignored_parent_pairs(self, config: CollisionGuardConfig) -> CollisionGuardConfig:
        ignored = {
            tuple(sorted((self._link_names.get(parent, str(parent)), self._link_names.get(child, str(child)))))
            for child, parent in self._parent_links.items()
        }
        ignored.update(config.ignored_link_pairs)
        return CollisionGuardConfig(
            enabled=config.enabled,
            stop_on_collision=config.stop_on_collision,
            check_self=config.check_self,
            check_environment=config.check_environment,
            distance_threshold=config.distance_threshold,
            ignored_name_prefixes=config.ignored_name_prefixes,
            ignored_link_pairs=frozenset(ignored),
        )

    def _build_ik_limits(self):
        """Build (lower, upper, ranges, rest) arrays for all movable joints.

        PyBullet's calculateInverseKinematics requires all four null-space arrays
        to be the same length as the number of non-fixed joints (in joint-index order).
        Zero-range joints (e.g. simlab mirror joints with lo==hi==0) are opened to
        ±2π so they don't fight the solver.
        """
        p, cid = self._p, self._client
        n = p.getNumJoints(self._robot_id, physicsClientId=cid)
        lower, upper = [], []
        for i in range(n):
            info = p.getJointInfo(self._robot_id, i, physicsClientId=cid)
            if info[2] == p.JOINT_FIXED:
                continue
            lo, hi = info[8], info[9]
            if abs(hi - lo) < 0.01:   # unconstrained / zero-range → open fully
                lo, hi = -math.pi * 2, math.pi * 2
            lower.append(lo)
            upper.append(hi)
        ranges = [hi - lo for lo, hi in zip(lower, upper)]
        rest   = [(lo + hi) / 2.0 for lo, hi in zip(lower, upper)]
        return lower, upper, ranges, rest

    def _apply_joint_angles(self, q_deg) -> None:
        """Apply body-frame angles to URDF joints (by name), keeping wrist level."""
        p   = self._p
        cid = self._client
        mapped = firmware_deg_to_visual_rad(
            q_deg[0],
            q_deg[1],
            q_deg[2],
            q_deg[3] if len(q_deg) > 3 else 0.0,
        )
        # Simlab chain expects RELATIVE elbow on q3 (matches firmware-correct
        # geometry built in urdf_loader._simlab_spec for the Magician).
        j1, j2, j3 = (math.radians(float(a)) for a in q_deg[:3])
        simlab_j3 = j3 - j2
        j4 = 0.0
        for name, angle in (
            (self.JOINT_BASE,     mapped.joint_1),
            (self.JOINT_SHOULDER, mapped.joint_2),
            (self.JOINT_FOREARM,  mapped.joint_3),
            (self.JOINT_WRIST,    mapped.joint_4),
            (self.JOINT_MIMIC_1,  mapped.joint_mimic_1),
            (self.JOINT_MIMIC_2,  mapped.joint_mimic_2),
            (self.SIMLAB_JOINTS["q1"], j1),
            (self.SIMLAB_JOINTS["q2"], j2),
            (self.SIMLAB_JOINTS["q3"], simlab_j3),
            (self.SIMLAB_JOINTS["q4"], j4),
        ):
            p.resetJointState(self._robot_id, self._joint_idx[name], angle, physicsClientId=cid)
        p.stepSimulation(physicsClientId=cid)

    def get_visual_tool_axes(self):
        """Return (x_axis, y_axis, z_axis) world-frame unit vectors for visual tool link."""
        state = self._p.getLinkState(
            self._robot_id,
            self._ee_link,
            computeForwardKinematics=True,
            physicsClientId=self._client,
        )
        rot = self._p.getMatrixFromQuaternion(state[1])
        x_axis = (float(rot[0]), float(rot[3]), float(rot[6]))
        y_axis = (float(rot[1]), float(rot[4]), float(rot[7]))
        z_axis = (float(rot[2]), float(rot[5]), float(rot[8]))
        return x_axis, y_axis, z_axis

    def step(self, n: int = 1) -> None:
        """Advance the PyBullet simulation by `n` fixed timesteps (test hook)."""
        for _ in range(int(n)):
            self._p.stepSimulation(physicsClientId=self._client)

    def get_sim_ee_pose(self):
        """Return simulator-derived (x, y, z, r) in the lab frame."""
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
                    body_a="magician",
                    body_b="magician",
                    link_a=link_a,
                    link_b=link_b,
                    distance=float(c[8]),
                    normal_force=float(c[9]),
                )
            )
        return contacts

    def _try_apply_joint_angles(self, q_deg, cartesian) -> bool:
        old_q = self._q[:]
        old_cartesian = self._cartesian
        self._apply_joint_angles(q_deg)
        contacts = self.get_collision_contacts()
        if contacts and self._collision_config.stop_on_collision:
            self._apply_joint_angles(old_q)
            self._q = old_q
            self._cartesian = old_cartesian
            self._collision_stopped = True
            self._last_collision = contacts[0]
            print(f"[SimDobot-PyBullet] COLLISION STOP: {contact_summaries(contacts, limit=1)}")
            return False
        self._q = list(q_deg)
        self._cartesian = cartesian
        self._collision_stopped = False
        self._last_collision = None
        self._update_payload_position()
        return True

    def _solve_ik_cartesian(self, x, y, z, r):
        """Run PyBullet IK for (x,y,z,r) mm/deg. Returns [q1,q2,q3,q4] degrees.

        All four null-space arrays (lowerLimits, upperLimits, jointRanges, restPoses)
        are passed together — PyBullet silently ignores limits when any are missing.
        """
        p, cid = self._p, self._client
        raw = p.calculateInverseKinematics(
            self._robot_id, self._ee_link,
            [float(x)/1000, float(y)/1000, float(z)/1000],
            lowerLimits=self._ik_lower,
            upperLimits=self._ik_upper,
            jointRanges=self._ik_ranges,
            restPoses=self._ik_rest,
            maxNumIterations=200, residualThreshold=1e-5,
            physicsClientId=cid,
        )
        movable = [n for n, i in sorted(self._joint_idx.items(), key=lambda kv: kv[1])
                   if p.getJointInfo(self._robot_id, i, physicsClientId=cid)[2] != p.JOINT_FIXED]
        ib = dict(zip(movable, raw))
        # JOINT_FOREARM is the relative elbow (J3_fw − J2_fw); convert back.
        j2_rad = ib.get(self.JOINT_SHOULDER, 0.0)
        j3_rel_rad = ib.get(self.JOINT_FOREARM, 0.0)
        return [
            math.degrees(ib.get(self.JOINT_BASE, 0.0)),
            math.degrees(j2_rad),
            math.degrees(j2_rad + j3_rel_rad),
            0.0,
        ]

    def move_to(self, x=None, y=None, z=None, r=0, wait=True, mode=None):
        if _is_joint_mode(mode):
            q_deg = [float(x), float(y), float(z), 0.0]
            self._try_apply_joint_angles(q_deg, self._fk(q_deg))
            print(f"[SimDobot-PyBullet] MOVJ_ANGLE q={[f'{v:.1f}' for v in q_deg]} deg")
        else:
            q_deg = self._solve_ik_cartesian(x, y, z, r)
            self._try_apply_joint_angles(q_deg, (float(x), float(y), float(z), 0.0))
            print(f"[SimDobot-PyBullet] move_to({float(x):.1f}, {float(y):.1f}, {float(z):.1f}, 0.0)")

    def interpolate_to(self, x, y, z, r=0, steps=30, pause=0.02, lock=None) -> None:
        """Smooth animated Cartesian move via joint interpolation."""
        q_target = self._solve_ik_cartesian(x, y, z, r)
        q_start  = self._q[:]
        ctx = lock if lock is not None else contextlib.nullcontext()
        for i in range(1, steps + 1):
            t    = i / steps
            q_i  = [q_start[j] + t * (q_target[j] - q_start[j]) for j in range(4)]
            with ctx:
                if not self._try_apply_joint_angles(q_i, self._fk(q_i)):
                    return
            time.sleep(pause)
        self._cartesian = (float(x), float(y), float(z), 0.0)

    def interpolate_joints_to(self, q_target, steps=30, pause=0.02, lock=None) -> None:
        """Smooth animated joint move."""
        q_start = self._q[:]
        ctx = lock if lock is not None else contextlib.nullcontext()
        for i in range(1, steps + 1):
            t   = i / steps
            q_i = [q_start[j] + t * (q_target[j] - q_start[j]) for j in range(4)]
            with ctx:
                if not self._try_apply_joint_angles(q_i, self._fk(q_i)):
                    return
            time.sleep(pause)

    def is_running(self) -> bool:
        try:
            return bool(self._p.isConnected(self._client))
        except Exception:
            return False

    def sync_viewer(self) -> None:
        pass  # PyBullet GUI runs in its own thread

    def suck(self, state: bool) -> None:
        if state and self._tool == "none":
            print("[SimDobot-PyBullet] suck() requested but tool='none' "
                  "- attaching payload at the bare-default TCP. Pass tool='suction' "
                  "to SimDobot() (or DOBOT_EE=suction) for the cup mount.")
        if state:
            self._attach_payload()
        else:
            self._detach_payload()

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
        print(f"[SimDobot-PyBullet] suck ON  - FK ({xc:.1f}, {yc:.1f}, {zc:.1f}) mm")

    def _detach_payload(self):
        if self._payload_id is None:
            return
        self._p.removeBody(self._payload_id, physicsClientId=self._client)
        print("[SimDobot-PyBullet] suck OFF - payload removed")
        self._payload_id = None

    def _update_payload_position(self):
        if self._payload_id is None:
            return
        pos = self._ee_world_pos()
        self._p.resetBasePositionAndOrientation(
            self._payload_id, pos, [0, 0, 0, 1], physicsClientId=self._client)

    def close(self):
        self._detach_payload()
        try:
            self._p.disconnect(self._client)
        except Exception:
            pass
        print("[SimDobot-PyBullet] Disconnected.")


# ---------------------------------------------------------------------------
# MuJoCo backend
# ---------------------------------------------------------------------------

class _SimDobotMuJoCo(_SimDobotBase):
    """Magician URDF compiled for MuJoCo via urdf_loader (DAE→OBJ, MJCF wrapper). Joint drive matches PyBullet path."""

    # jkaniuka/magician_ros2 visible-chain joint + body names. Mimic joints
    # represent the parallelogram passive constraint; software-driven because
    # MuJoCo does not honour URDF <mimic> tags.
    JOINT_BASE     = "magician_joint_1"
    JOINT_SHOULDER = "magician_joint_2"
    JOINT_FOREARM  = "magician_joint_3"
    JOINT_WRIST    = "magician_joint_4"
    JOINT_MIMIC_1  = "magician_joint_mimic_1"
    JOINT_MIMIC_2  = "magician_joint_mimic_2"
    EE_BODY_BY_TOOL = {
        "none": "magician_link_ee",
        "motor": "magician_link_ee",
        "suction": "magician_link_suction_cup",
    }
    SIMLAB_JOINTS  = {
        "q1": "simlab_magician_j1",
        "q2": "simlab_magician_j2",
        "q3": "simlab_magician_j3",
        "q4": "simlab_magician_j4",
    }
    SIMLAB_TIP_BODY = "simlab_magician_tip"
    def __init__(
        self,
        gui=True,
        collision_guard: bool | CollisionGuardConfig | None = None,
        tool: str = "none",
    ):
        super().__init__(collision_guard=collision_guard, tool=tool)
        # Do not force EGL for GUI windows; MuJoCo's passive viewer owns GLFW,
        # and mixing them can crash some Linux/OpenGL drivers during shutdown.
        if not gui:
            os.environ.setdefault("MUJOCO_GL", "egl")
        _urdf = resolve_magician_urdf_path(tool=self._tool)
        self.EE_BODY = self.EE_BODY_BY_TOOL[self._tool]
        try:
            import mujoco
        except ImportError as exc:
            raise ImportError("MuJoCo backend requires: pip install mujoco") from exc

        try:
            from .urdf_loader import prepare_urdf_for_mujoco
        except ImportError:
            from urdf_loader import prepare_urdf_for_mujoco
        cache_dir = os.environ.get(
            "DOBOT_SIM_CACHE", os.path.join(os.path.expanduser("~"), ".cache", "dobot_sim")
        )
        wrapper = prepare_urdf_for_mujoco(_urdf, cache_dir)

        self._mj    = mujoco
        self._model = mujoco.MjModel.from_xml_path(str(wrapper))
        self._data  = mujoco.MjData(self._model)
        self._viewer = None
        self._viewer_mod = None
        self._qposadr = {}
        for name in (self.JOINT_BASE, self.JOINT_SHOULDER, self.JOINT_FOREARM,
                     self.JOINT_WRIST, self.JOINT_MIMIC_1, self.JOINT_MIMIC_2,
                     *self.SIMLAB_JOINTS.values()):
            j = self._model.joint(name)
            self._qposadr[name] = int(j.qposadr[0])
        self._ee_body_name = self.EE_BODY
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
                ("link_1", "link_5"),
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
            # Non-blocking viewer window; user closes via the window's X button.
            self._viewer_mod = _import_mujoco_viewer()
            self._viewer = self._viewer_mod.launch_passive(self._model, self._data)
            _show_visual_geoms_only(self._viewer)
        print(
            f"[SimDobot-MuJoCo] Loaded wrapper: nv={self._model.nv} "
            f"EE_body='{self._ee_body_name}' GUI={'on' if gui else 'off'}"
        )

    def _apply_joint_angles(self, q_deg, *, forward: bool = False) -> None:
        """Write firmware angles into mj data.qpos and recompute kinematics."""
        mj = self._mj
        mapped = firmware_deg_to_visual_rad(
            q_deg[0],
            q_deg[1],
            q_deg[2],
            q_deg[3] if len(q_deg) > 3 else 0.0,
        )
        # Simlab chain expects RELATIVE elbow on q3 (firmware-correct
        # geometry built in urdf_loader._simlab_spec for the Magician).
        j1, j2, j3 = (math.radians(float(a)) for a in q_deg[:3])
        simlab_j3 = j3 - j2
        j4 = 0.0
        self._data.qpos[self._qposadr[self.JOINT_BASE]]     = mapped.joint_1
        self._data.qpos[self._qposadr[self.JOINT_SHOULDER]] = mapped.joint_2
        self._data.qpos[self._qposadr[self.JOINT_FOREARM]]  = mapped.joint_3
        self._data.qpos[self._qposadr[self.JOINT_WRIST]]    = mapped.joint_4
        self._data.qpos[self._qposadr[self.JOINT_MIMIC_1]]  = mapped.joint_mimic_1
        self._data.qpos[self._qposadr[self.JOINT_MIMIC_2]]  = mapped.joint_mimic_2
        self._data.qpos[self._qposadr[self.SIMLAB_JOINTS["q1"]]] = j1
        self._data.qpos[self._qposadr[self.SIMLAB_JOINTS["q2"]]] = j2
        self._data.qpos[self._qposadr[self.SIMLAB_JOINTS["q3"]]] = simlab_j3
        self._data.qpos[self._qposadr[self.SIMLAB_JOINTS["q4"]]] = j4
        if forward:
            mj.mj_forward(self._model, self._data)
        else:
            mj.mj_kinematics(self._model, self._data)
        if self._viewer is not None:
            self._viewer.sync()

    def get_visual_tool_axes(self):
        """Return (x_axis, y_axis, z_axis) world-frame unit vectors for visual tool body."""
        self._mj.mj_kinematics(self._model, self._data)
        xmat = self._data.xmat[self._ee_body_id]
        x_axis = (float(xmat[0]), float(xmat[3]), float(xmat[6]))
        y_axis = (float(xmat[1]), float(xmat[4]), float(xmat[7]))
        z_axis = (float(xmat[2]), float(xmat[5]), float(xmat[8]))
        return x_axis, y_axis, z_axis

    def step(self, n: int = 1) -> None:
        """Advance MuJoCo dynamics by `n` timesteps (test hook)."""
        for _ in range(int(n)):
            self._mj.mj_step(self._model, self._data)
        if self._viewer is not None:
            self._viewer.sync()

    def get_sim_ee_pose(self):
        """Return simulator-derived (x, y, z, r) in the lab frame."""
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
                    body_a="magician",
                    body_b="magician",
                    link_a=name_a,
                    link_b=name_b,
                    distance=float(c.dist),
                )
            )
        return contacts

    def _try_apply_joint_angles(self, q_deg, cartesian) -> bool:
        old_q = self._q[:]
        old_cartesian = self._cartesian
        self._apply_joint_angles(q_deg, forward=self._collision_config.enabled)
        contacts = self.get_collision_contacts()
        if contacts and self._collision_config.stop_on_collision:
            self._apply_joint_angles(old_q, forward=self._collision_config.enabled)
            self._q = old_q
            self._cartesian = old_cartesian
            self._collision_stopped = True
            self._last_collision = contacts[0]
            print(f"[SimDobot-MuJoCo]   COLLISION STOP: {contact_summaries(contacts, limit=1)}")
            return False
        self._q = list(q_deg)
        self._cartesian = cartesian
        self._collision_stopped = False
        self._last_collision = None
        return True

    def move_to(self, x=None, y=None, z=None, r=0, wait=True, mode=None):
        if _is_joint_mode(mode):
            q_deg = [float(x), float(y), float(z), 0.0]
            self._try_apply_joint_angles(q_deg, self._fk(q_deg))
            print(f"[SimDobot-MuJoCo]   MOVJ_ANGLE q={[f'{v:.1f}' for v in q_deg]} deg")
        else:
            # MuJoCo ships no built-in IK; fall back to the analytical FK
            # model: solve the planar 2R IK ourselves.
            tx, ty, tz, tr = float(x), float(y), float(z), 0.0
            q_deg = self._solve_ik_2r(tx, ty, tz, tr, self._q)
            self._try_apply_joint_angles(q_deg, (tx, ty, tz, tr))
            print(f"[SimDobot-MuJoCo]   move_to({tx:.1f}, {ty:.1f}, {tz:.1f}, {tr:.1f})")

    def _solve_ik_2r(self, x, y, z, r, q0):
        """Magician analytical IK — firmware joint semantics.

        Delegates to :func:`kinematics.ik_magician_firmware`. Returns
        ``[J1, J2, J3, 0]`` in degrees, matching real-robot ``GetPose``
        sign conventions. ``q0`` (firmware seed) drives elbow-branch
        continuity for cartesian moves that cross J2 = J3.
        """
        return ik_magician_firmware(x, y, z, r, q0=q0, L1=self.L1, L2=self.L2, tool=self._tool)

    def interpolate_to(self, x, y, z, r=0, steps=30, pause=0.02, lock=None) -> None:
        """Smooth animated Cartesian move via analytical IK + joint interpolation."""
        tx, ty, tz, tr = float(x), float(y), float(z), float(r)
        q_target = self._solve_ik_2r(tx, ty, tz, tr, self._q)
        q_start  = self._q[:]
        ctx = lock if lock is not None else contextlib.nullcontext()
        for i in range(1, steps + 1):
            t   = i / steps
            q_i = [q_start[j] + t * (q_target[j] - q_start[j]) for j in range(4)]
            with ctx:
                if not self._try_apply_joint_angles(q_i, self._fk(q_i)):
                    return
            time.sleep(pause)
        self._cartesian = (tx, ty, tz, 0.0)

    def interpolate_joints_to(self, q_target, steps=30, pause=0.02, lock=None) -> None:
        """Smooth animated joint move."""
        q_start = self._q[:]
        ctx = lock if lock is not None else contextlib.nullcontext()
        for i in range(1, steps + 1):
            t   = i / steps
            q_i = [q_start[j] + t * (q_target[j] - q_start[j]) for j in range(4)]
            with ctx:
                if not self._try_apply_joint_angles(q_i, self._fk(q_i)):
                    return
            time.sleep(pause)

    def is_running(self) -> bool:
        """True while the session is open.

        With a passive viewer window, mirrors ``viewer.is_running()`` (False after
        the user closes the window). In headless mode (no viewer), returns True until
        :meth:`close` nulls the model — matching PyBullet ``is_connected`` semantics
        so visual pick-place loops don't exit immediately under MuJoCo + ``gui=False``.
        """
        if self._viewer is not None:
            return bool(self._viewer.is_running())
        return getattr(self, "_model", None) is not None

    def sync_viewer(self) -> None:
        if self._viewer is not None and self._viewer.is_running():
            self._viewer.sync()

    def show(self):
        """Open (or re-open) the MuJoCo viewer window at the current state."""
        mj = self._mj
        if self._viewer is None:
            self._viewer_mod = self._viewer_mod or _import_mujoco_viewer()
            self._viewer = self._viewer_mod.launch_passive(self._model, self._data)
            _show_visual_geoms_only(self._viewer)
        mj.mj_kinematics(self._model, self._data)
        self._viewer.sync()
        print("[SimDobot-MuJoCo]   Viewer synced; close the window (or call .close()) when done.")

    def close(self):
        viewer = self._viewer
        if viewer is not None:
            try:
                viewer.close()
                # launch_passive: render_loop runs on another thread; exit is not synchronous.
                deadline = time.time() + 5.0
                while viewer.is_running() and time.time() < deadline:
                    time.sleep(0.02)
            except Exception:
                pass
            self._viewer = None
        # Nullify before Python GC to avoid MuJoCo/GLFW use-after-free on exit
        self._data  = None
        self._model = None
        print("[SimDobot-MuJoCo]   Closed.")


# ---------------------------------------------------------------------------
# Factory
# ---------------------------------------------------------------------------

def SimDobot(
    backend: str | None = None,
    gui: bool | None = None,
    collision_guard: bool | CollisionGuardConfig | None = None,
    tool: str | None = None,
):
    """Construct SimDobot: backend mujoco|pybullet, with GUI and tool env overrides."""
    backend = (backend or os.environ.get("DOBOT_SIM_BACKEND", "mujoco")).lower()
    if gui is None:
        gui = (os.environ.get("DOBOT_SIM_GUI", "1") != "0"
               and os.environ.get("DOBOT_VIZ", "1") != "0")
    tool = (tool or os.environ.get("DOBOT_EE", "none")).lower()
    if backend == "pybullet":
        return _SimDobotPyBullet(gui=gui, collision_guard=collision_guard, tool=tool)
    if backend == "mujoco":
        return _SimDobotMuJoCo(gui=gui, collision_guard=collision_guard, tool=tool)
    raise ValueError(
        f"Unknown SimDobot backend '{backend}'. Expected one of: pybullet, mujoco."
    )


# ---------------------------------------------------------------------------
# Smoke test
# ---------------------------------------------------------------------------

if __name__ == "__main__":
    # FK smoke test using firmware semantics with current default tool='none'
    # (zero TCP offset). Home is (147, 0, 135).
    configs = [
        ([0,   0,   0, 0], (147.00,    0.00, 135.00)),  # bare-none home
        ([0,  90,   0, 0], (282.00,    0.00,   0.00)),  # arm horizontal
        ([0,   0,  90, 0], (  0.00,    0.00, -12.00)),  # forearm pointing down
        ([90,  0,   0, 0], (  0.00,  147.00, 135.00)),  # base rotated 90°
        ([0,  30,   0, 0], (214.50,    0.00, 116.91)),
        ([45, 30,  20, 0], (145.41,  145.41,  66.64)),
    ]
    print("FK smoke test (firmware semantics):")
    all_ok = True
    for q, (xe, ye, ze) in configs:
        x, y, z, _ = fk_magician_firmware(q[0], q[1], q[2])
        ok = abs(x - xe) < 0.5 and abs(y - ye) < 0.5 and abs(z - ze) < 0.5
        status = "PASS" if ok else "FAIL"
        if not ok:
            all_ok = False
        print(
            f"  {status}  q={q}  X={x:7.2f} Y={y:7.2f} Z={z:7.2f}  "
            f"(expected X={xe} Y={ye} Z={ze})"
        )
    print("sim_dobot.py: syntax OK" + ("  FK OK" if all_ok else "  FK MISMATCH"))

"""
PyQt5 control panel for Dobot Magician (ME403).

Port pick, connect/disconnect, alarms, home, ready pose, J1-J4 steps, live pose,
and a log. Run: python 00_magician_gui.py. Setup: magician/README.md.
"""

from __future__ import annotations

import sys
from pathlib import Path
from typing import Callable, Optional

from PyQt5.QtCore import Qt, QThread, QTimer, pyqtSignal
from PyQt5.QtGui import QFont
from PyQt5.QtWidgets import (
    QApplication, QComboBox, QDoubleSpinBox, QFrame, QGroupBox,
    QHBoxLayout, QLabel, QMainWindow, QPushButton,
    QTextEdit, QVBoxLayout, QWidget,
)

# ---------------------------------------------------------------------------
# Ensure local directory is on sys.path for utils import
# ---------------------------------------------------------------------------

_HERE = Path(__file__).parent
if str(_HERE) not in sys.path:
    sys.path.insert(0, str(_HERE))

from pydobotplus import Dobot  # noqa: E402
from pydobotplus.dobotplus import MODE_PTP  # noqa: E402

from utils import (  # noqa: E402
    clamp, find_port, go_home, prepare_robot, unpack_pose,
    SAFE_READY_POSE, HOME_JOINTS,
)

# ---------------------------------------------------------------------------
# Constants
# ---------------------------------------------------------------------------

JOINT_BOUNDS = {
    "j1": (-90.0,  90.0),
    "j2": (  0.0,  85.0),
    "j3": (-10.0,  85.0),
    "j4": (-90.0,  90.0),
}

JOINT_DESCRIPTIONS = {
    "j1": "Base Rotation",
    "j2": "Rear Arm Elevation",
    "j3": "Forearm Elevation",
    "j4": "End-Effector Rotation",
}

POLL_INTERVAL_MS = 500

_STATUS_STYLE: dict[str, tuple[str, str]] = {
    "—":             ("#bdc3c7", "#2c3e50"),
    "CONNECTING":    ("#2980b9", "white"),
    "CONNECTED":     ("#27ae60", "white"),
    "MOVING":        ("#8e44ad", "white"),
    "DISCONNECTING": ("#e67e22", "white"),
    "ERROR":         ("#c0392b", "white"),
}


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def clamp_joints(j1: float, j2: float, j3: float, j4: float):
    """Clamp firmware angles to JOINT_BOUNDS. Returns (j1,j2,j3,j4, was_clamped)."""
    cj1 = clamp(j1, *JOINT_BOUNDS["j1"])
    cj2 = clamp(j2, *JOINT_BOUNDS["j2"])
    cj3 = clamp(j3, *JOINT_BOUNDS["j3"])
    cj4 = clamp(j4, *JOINT_BOUNDS["j4"])
    was_clamped = (cj1, cj2, cj3, cj4) != (j1, j2, j3, j4)
    return cj1, cj2, cj3, cj4, was_clamped


# ---------------------------------------------------------------------------
# Port discovery helper
# ---------------------------------------------------------------------------

def list_available_ports() -> list[str]:
    """Return a list of serial port device names."""
    from serial.tools import list_ports
    return [p.device for p in sorted(list_ports.comports(), key=lambda p: p.device)]


# ---------------------------------------------------------------------------
# AxisRow — one labeled axis row with spinbox and +/− step buttons
# ---------------------------------------------------------------------------

class AxisRow(QWidget):
    """Label | SpinBox | [−] [+] [Move] — clicking +/− adjusts spinbox only."""

    move_requested = pyqtSignal(float)

    def __init__(
        self,
        label: str,
        lo: float,
        hi: float,
        decimals: int = 2,
        suffix: str = "",
        parent: Optional[QWidget] = None,
    ) -> None:
        super().__init__(parent)
        self._lo = lo
        self._hi = hi

        row = QHBoxLayout(self)
        row.setContentsMargins(0, 2, 0, 2)

        lbl = QLabel(label)
        lbl.setFixedWidth(40)
        lbl.setFont(QFont("monospace", 10, QFont.Bold))
        row.addWidget(lbl)

        self.spin = QDoubleSpinBox()
        self.spin.setRange(lo, hi)
        self.spin.setDecimals(decimals)
        self.spin.setSuffix(suffix)
        self.spin.setFixedWidth(104)
        self.spin.setFont(QFont("monospace", 10))
        row.addWidget(self.spin)

        self.minus_btn = QPushButton("−")
        self.minus_btn.setFixedWidth(32)
        self.minus_btn.clicked.connect(self._step_down)
        row.addWidget(self.minus_btn)

        self.plus_btn = QPushButton("+")
        self.plus_btn.setFixedWidth(32)
        self.plus_btn.clicked.connect(self._step_up)
        row.addWidget(self.plus_btn)

        self.move_btn = QPushButton("Move")
        self.move_btn.setFixedWidth(52)
        self.move_btn.clicked.connect(lambda: self.move_requested.emit(self.spin.value()))
        row.addWidget(self.move_btn)

        row.addStretch()

        self.get_step: Callable[[], float] = lambda: 1.0

    def _step_down(self) -> None:
        self.spin.setValue(max(self._lo, self.spin.value() - self.get_step()))

    def _step_up(self) -> None:
        self.spin.setValue(min(self._hi, self.spin.value() + self.get_step()))

    def set_enabled(self, en: bool) -> None:
        for w in (self.spin, self.minus_btn, self.plus_btn, self.move_btn):
            w.setEnabled(en)


# ---------------------------------------------------------------------------
# Workers (QThread)
# ---------------------------------------------------------------------------

class ConnectWorker(QThread):
    done = pyqtSignal(bool, str)

    def __init__(self, port: str) -> None:
        super().__init__()
        self.port = port
        self.bot: Optional[Dobot] = None

    def run(self) -> None:
        try:
            bot = Dobot(port=self.port)
            prepare_robot(bot)
            self.bot = bot
            self.done.emit(True, f"Connected on {self.port}")
        except Exception as exc:
            self.done.emit(False, str(exc))


class MotionWorker(QThread):
    done = pyqtSignal(bool, str)

    def __init__(self, motion_fn: Callable) -> None:
        super().__init__()
        self._motion_fn = motion_fn

    def run(self) -> None:
        try:
            msg = self._motion_fn()
            self.done.emit(True, msg or "")
        except Exception as exc:
            self.done.emit(False, str(exc))


# ---------------------------------------------------------------------------
# Main window
# ---------------------------------------------------------------------------

class MagicianWindow(QMainWindow):
    def __init__(self) -> None:
        super().__init__()
        self.setWindowTitle("Dobot Magician — Introduction Week GUI  (ME 403)")
        self.setMinimumSize(720, 580)

        self._bot: Optional[Dobot] = None
        self._connect_worker:  Optional[ConnectWorker] = None
        self._motion_worker:   Optional[MotionWorker]  = None

        self._poll_timer = QTimer(self)
        self._poll_timer.setInterval(POLL_INTERVAL_MS)
        self._poll_timer.timeout.connect(self._poll_pose)

        self._build_ui()
        self._set_connected(False)

    # ------------------------------------------------------------------
    # UI construction
    # ------------------------------------------------------------------

    def _build_ui(self) -> None:
        central = QWidget()
        self.setCentralWidget(central)
        root = QVBoxLayout(central)
        root.setSpacing(8)
        root.setContentsMargins(10, 10, 10, 10)

        # ── Connection bar ────────────────────────────────────────────
        conn_bar = QHBoxLayout()

        conn_bar.addWidget(QLabel("Port:"))
        self._port_combo = QComboBox()
        self._port_combo.setFixedWidth(120)
        self._port_combo.setFont(QFont("monospace", 9))
        self._refresh_ports()
        conn_bar.addWidget(self._port_combo)

        self._refresh_btn = QPushButton("⟳")
        self._refresh_btn.setFixedWidth(28)
        self._refresh_btn.setToolTip("Refresh serial port list")
        self._refresh_btn.clicked.connect(self._refresh_ports)
        conn_bar.addWidget(self._refresh_btn)

        conn_bar.addStretch()

        self._connect_btn = QPushButton("Connect")
        self._connect_btn.setFixedWidth(88)
        self._connect_btn.clicked.connect(self._on_connect)
        conn_bar.addWidget(self._connect_btn)

        self._disconnect_btn = QPushButton("Disconnect")
        self._disconnect_btn.setFixedWidth(96)
        self._disconnect_btn.clicked.connect(self._on_disconnect)
        conn_bar.addWidget(self._disconnect_btn)

        self._status_badge = QLabel("—")
        self._status_badge.setFixedWidth(120)
        self._status_badge.setAlignment(Qt.AlignCenter)
        self._status_badge.setFont(QFont("monospace", 9, QFont.Bold))
        self._apply_badge("—")
        conn_bar.addWidget(self._status_badge)

        root.addLayout(conn_bar)

        # ── Action bar ────────────────────────────────────────────────
        action_bar = QHBoxLayout()

        self._clear_alarms_btn = QPushButton("Clear Alarms")
        self._clear_alarms_btn.setFixedWidth(100)
        self._clear_alarms_btn.setToolTip("get_alarms() → clear_alarms()")
        self._clear_alarms_btn.clicked.connect(self._on_clear_alarms)
        action_bar.addWidget(self._clear_alarms_btn)

        self._home_btn = QPushButton("Home")
        self._home_btn.setFixedWidth(72)
        self._home_btn.setToolTip("Move to joint zero (0, 0, 0, 0)")
        self._home_btn.clicked.connect(self._on_home)
        action_bar.addWidget(self._home_btn)

        self._ready_btn = QPushButton("Ready Pose")
        self._ready_btn.setFixedWidth(96)
        self._ready_btn.setToolTip(f"Move to Cartesian {SAFE_READY_POSE}")
        self._ready_btn.clicked.connect(self._on_ready_pose)
        action_bar.addWidget(self._ready_btn)

        action_bar.addStretch()

        action_bar.addWidget(QLabel("Step°:"))
        self._step_combo = QComboBox()
        for v in ("0.5°", "1°", "5°", "10°"):
            self._step_combo.addItem(v, userData=float(v.rstrip("°")))
        self._step_combo.setCurrentIndex(1)
        self._step_combo.setFixedWidth(64)
        action_bar.addWidget(self._step_combo)

        root.addLayout(action_bar)

        sep1 = QFrame()
        sep1.setFrameShape(QFrame.HLine)
        sep1.setFrameShadow(QFrame.Sunken)
        root.addWidget(sep1)

        # ── Middle: joint controls + live readout ─────────────────────
        mid = QHBoxLayout()

        # --- Left: Joint controls ---
        joint_box = QGroupBox("Joint Control")
        joint_layout = QVBoxLayout(joint_box)
        joint_layout.setSpacing(4)

        axes = [
            ("J1", *JOINT_BOUNDS["j1"]),
            ("J2", *JOINT_BOUNDS["j2"]),
            ("J3", *JOINT_BOUNDS["j3"]),
            ("J4", *JOINT_BOUNDS["j4"]),
        ]
        self._joint_rows: list[AxisRow] = []
        for idx, (label, lo, hi) in enumerate(axes):
            row = AxisRow(label, lo, hi, decimals=2, suffix="°")
            row.move_requested.connect(
                lambda v, i=idx: self._on_move_single(i, v)
            )
            joint_layout.addWidget(row)
            self._joint_rows.append(row)

        joint_layout.addStretch()

        send_all = QPushButton("Send All  (J1 – J4)")
        send_all.setFixedHeight(32)
        send_all.clicked.connect(self._on_send_all)
        self._send_all_btn = send_all
        joint_layout.addWidget(send_all)

        mid.addWidget(joint_box, stretch=2)

        # --- Right: Live pose + Joint limits ---
        right_col = QVBoxLayout()

        # Live pose readout
        pose_box = QGroupBox("Live Pose")
        pose_layout = QVBoxLayout(pose_box)
        pose_layout.setSpacing(4)

        mono = QFont("monospace", 10)
        self._pose_labels: dict[str, QLabel] = {}
        for key in ("X", "Y", "Z", "R", "J1", "J2", "J3", "J4"):
            lbl = QLabel(f"{key}: —")
            lbl.setFont(mono)
            pose_layout.addWidget(lbl)
            self._pose_labels[key] = lbl

        pose_layout.addStretch()
        right_col.addWidget(pose_box)

        # Joint limits reference
        limits_box = QGroupBox("Joint Limits")
        limits_layout = QVBoxLayout(limits_box)
        limits_layout.setSpacing(2)

        small_mono = QFont("monospace", 9)
        for key in ("j1", "j2", "j3", "j4"):
            lo, hi = JOINT_BOUNDS[key]
            desc = JOINT_DESCRIPTIONS[key]
            lbl = QLabel(f"{key.upper()}: {lo:+.0f}° to {hi:+.0f}°  ({desc})")
            lbl.setFont(small_mono)
            lbl.setStyleSheet("color:#2c3e50;")
            limits_layout.addWidget(lbl)

        right_col.addWidget(limits_box)

        mid.addLayout(right_col, stretch=1)
        root.addLayout(mid)

        sep2 = QFrame()
        sep2.setFrameShape(QFrame.HLine)
        sep2.setFrameShadow(QFrame.Sunken)
        root.addWidget(sep2)

        # ── Details panel ─────────────────────────────────────────────
        detail_hdr = QHBoxLayout()
        detail_hdr.addWidget(QLabel("Details"))
        detail_hdr.addStretch()
        clear_btn = QPushButton("Clear")
        clear_btn.setFixedWidth(58)
        clear_btn.clicked.connect(lambda: self._detail.clear())
        detail_hdr.addWidget(clear_btn)
        root.addLayout(detail_hdr)

        self._detail = QTextEdit()
        self._detail.setReadOnly(True)
        self._detail.setFont(QFont("monospace", 9))
        self._detail.setMaximumHeight(140)
        self._detail.setPlaceholderText(
            "Connection logs, alarm info, move results will appear here.\n"
            "Select a port and click Connect to begin."
        )
        root.addWidget(self._detail)

        # ── Footer ────────────────────────────────────────────────────
        footer = QLabel("Prepared by Yunus Emre Danabas  —  ME 403 Introduction to Robotics")
        footer.setAlignment(Qt.AlignCenter)
        footer.setFont(QFont("sans-serif", 8))
        footer.setStyleSheet("color:#95a5a6; margin-top:4px;")
        root.addWidget(footer)

        # ── Wire step providers to AxisRow widgets ────────────────────
        def deg_step() -> float:
            return self._step_combo.currentData()

        for row in self._joint_rows:
            row.get_step = deg_step

    # ------------------------------------------------------------------
    # Port list refresh
    # ------------------------------------------------------------------

    def _refresh_ports(self) -> None:
        current = self._port_combo.currentText()
        self._port_combo.clear()
        ports = list_available_ports()
        auto = find_port()
        if auto and auto not in ports:
            ports.insert(0, auto)
        for p in ports:
            tag = f"{p}  ★" if p == auto else p
            self._port_combo.addItem(tag, userData=p)
        preferred = current.split("  ")[0].strip() if current else auto
        if preferred:
            for i in range(self._port_combo.count()):
                if self._port_combo.itemData(i) == preferred:
                    self._port_combo.setCurrentIndex(i)
                    break

    # ------------------------------------------------------------------
    # Badge helper
    # ------------------------------------------------------------------

    def _apply_badge(self, status: str) -> None:
        bg, fg = _STATUS_STYLE.get(status, ("#95a5a6", "white"))
        self._status_badge.setText(status)
        self._status_badge.setStyleSheet(
            f"background-color:{bg}; color:{fg}; "
            "border-radius:4px; padding:2px 6px;"
        )

    # ------------------------------------------------------------------
    # Log helper
    # ------------------------------------------------------------------

    def _log(self, text: str) -> None:
        self._detail.append(text)
        sb = self._detail.verticalScrollBar()
        sb.setValue(sb.maximum())

    # ------------------------------------------------------------------
    # Connected / disconnected state
    # ------------------------------------------------------------------

    def _set_connected(self, connected: bool) -> None:
        self._connect_btn.setEnabled(not connected)
        self._port_combo.setEnabled(not connected)
        self._refresh_btn.setEnabled(not connected)
        self._disconnect_btn.setEnabled(connected)
        self._clear_alarms_btn.setEnabled(connected)
        self._home_btn.setEnabled(connected)
        self._ready_btn.setEnabled(connected)
        self._send_all_btn.setEnabled(connected)
        for row in self._joint_rows:
            row.set_enabled(connected)
        if not connected:
            self._apply_badge("—")
            for key, lbl in self._pose_labels.items():
                lbl.setText(f"{key}: —")

    def _set_motion_busy(self, busy: bool) -> None:
        en = not busy
        for row in self._joint_rows:
            row.set_enabled(en)
        self._send_all_btn.setEnabled(en)
        self._home_btn.setEnabled(en)
        self._ready_btn.setEnabled(en)
        self._disconnect_btn.setEnabled(en)
        self._clear_alarms_btn.setEnabled(en)
        self._apply_badge("MOVING" if busy else "CONNECTED")

    # ------------------------------------------------------------------
    # Connection lifecycle
    # ------------------------------------------------------------------

    def _on_connect(self) -> None:
        if self._connect_worker is not None:
            return
        port = self._port_combo.currentData()
        if not port:
            self._log("[Error] No port selected.")
            return
        self._apply_badge("CONNECTING")
        self._connect_btn.setEnabled(False)
        self._log(f"Connecting on {port} ...")

        worker = ConnectWorker(port)
        worker.done.connect(self._on_connect_done)
        self._connect_worker = worker
        worker.start()

    def _on_connect_done(self, success: bool, msg: str) -> None:
        worker = self._connect_worker
        self._connect_worker = None

        if not success:
            self._apply_badge("ERROR")
            self._connect_btn.setEnabled(True)
            self._log(f"[Error] {msg}")
            return

        self._bot = worker.bot
        self._log(msg)

        self._init_spinboxes()
        self._set_connected(True)
        self._apply_badge("CONNECTED")
        self._poll_timer.start()

    def _on_disconnect(self) -> None:
        self._poll_timer.stop()
        self._apply_badge("DISCONNECTING")
        self._disconnect_btn.setEnabled(False)

        if self._bot is not None:
            try:
                self._bot.close()
            except Exception:
                pass
            self._bot = None

        self._set_connected(False)
        self._log("Disconnected.")

    # ------------------------------------------------------------------
    # Init spinboxes from current robot state
    # ------------------------------------------------------------------

    def _init_spinboxes(self) -> None:
        if self._bot is None:
            return
        try:
            _, _, _, _, j1, j2, j3, j4 = unpack_pose(self._bot.get_pose())
        except Exception:
            return
        for row, v in zip(self._joint_rows, (j1, j2, j3, j4)):
            row.spin.setValue(v)

    # ------------------------------------------------------------------
    # Poll pose (500 ms)
    # ------------------------------------------------------------------

    def _poll_pose(self) -> None:
        if self._motion_worker is not None or self._bot is None:
            return
        try:
            x, y, z, r, j1, j2, j3, j4 = unpack_pose(self._bot.get_pose())
        except Exception:
            return

        for key, val in zip(
            ("X", "Y", "Z", "R", "J1", "J2", "J3", "J4"),
            (x, y, z, r, j1, j2, j3, j4),
        ):
            self._pose_labels[key].setText(f"{key}: {val:.2f}")

    # ------------------------------------------------------------------
    # Motion: Send All
    # ------------------------------------------------------------------

    def _on_send_all(self) -> None:
        if self._bot is None or self._motion_worker is not None:
            return
        bot = self._bot
        j1, j2, j3, j4 = [r.spin.value() for r in self._joint_rows]
        j1c, j2c, j3c, j4c, clamped = clamp_joints(j1, j2, j3, j4)
        if clamped:
            self._log(
                f"[clamp] ({j1:.1f},{j2:.1f},{j3:.1f},{j4:.1f})"
                f" → ({j1c:.1f},{j2c:.1f},{j3c:.1f},{j4c:.1f})"
            )

        def _fn():
            bot.move_to(j1c, j2c, j3c, j4c, wait=True, mode=MODE_PTP.MOVJ_ANGLE)
            return f"MOVJ_ANGLE {j1c:.1f} {j2c:.1f} {j3c:.1f} {j4c:.1f}"

        self._set_motion_busy(True)
        worker = MotionWorker(_fn)
        worker.done.connect(self._on_motion_done)
        self._motion_worker = worker
        worker.start()

    # ------------------------------------------------------------------
    # Motion: Single joint
    # ------------------------------------------------------------------

    def _on_move_single(self, axis_idx: int, val: float) -> None:
        if self._bot is None or self._motion_worker is not None:
            return
        bot = self._bot
        axis_names = ["J1", "J2", "J3", "J4"]
        try:
            _, _, _, _, cj1, cj2, cj3, cj4 = unpack_pose(bot.get_pose())
            combined = [cj1, cj2, cj3, cj4]
        except Exception:
            combined = [r.spin.value() for r in self._joint_rows]
        combined[axis_idx] = val
        j1c, j2c, j3c, j4c, _ = clamp_joints(*combined)

        def _fn():
            bot.move_to(j1c, j2c, j3c, j4c, wait=True, mode=MODE_PTP.MOVJ_ANGLE)
            return f"MOVJ_ANGLE {axis_names[axis_idx]}={val:.1f}"

        self._set_motion_busy(True)
        worker = MotionWorker(_fn)
        worker.done.connect(self._on_motion_done)
        self._motion_worker = worker
        worker.start()

    # ------------------------------------------------------------------
    # Motion done callback
    # ------------------------------------------------------------------

    def _on_motion_done(self, success: bool, msg: str) -> None:
        self._motion_worker = None
        self._set_motion_busy(False)
        if not success:
            self._log(f"[Error] {msg}")
        else:
            self._log(f"[Move] {msg}")
        self._init_spinboxes()

    # ------------------------------------------------------------------
    # Clear Alarms
    # ------------------------------------------------------------------

    def _on_clear_alarms(self) -> None:
        if self._bot is None or self._motion_worker is not None:
            return
        bot = self._bot

        def _fn():
            alarms = bot.get_alarms()
            if alarms:
                names = [str(getattr(a, "name", a)) for a in alarms]
                bot.clear_alarms()
                return f"Cleared {len(alarms)} alarm(s): {', '.join(names)}"
            return "No alarms active."

        self._set_motion_busy(True)
        worker = MotionWorker(_fn)
        worker.done.connect(self._on_motion_done)
        self._motion_worker = worker
        worker.start()

    # ------------------------------------------------------------------
    # Home
    # ------------------------------------------------------------------

    def _on_home(self) -> None:
        if self._bot is None or self._motion_worker is not None:
            return
        bot = self._bot

        def _fn():
            go_home(bot)
            return "Home (0, 0, 0, 0)"

        self._set_motion_busy(True)
        worker = MotionWorker(_fn)
        worker.done.connect(self._on_motion_done)
        self._motion_worker = worker
        worker.start()

    # ------------------------------------------------------------------
    # Ready Pose
    # ------------------------------------------------------------------

    def _on_ready_pose(self) -> None:
        if self._bot is None or self._motion_worker is not None:
            return
        bot = self._bot
        from utils import safe_move as _safe_move

        def _fn():
            _safe_move(bot, *SAFE_READY_POSE)
            return f"Ready Pose {SAFE_READY_POSE}"

        self._set_motion_busy(True)
        worker = MotionWorker(_fn)
        worker.done.connect(self._on_motion_done)
        self._motion_worker = worker
        worker.start()

    # ------------------------------------------------------------------
    # Close event
    # ------------------------------------------------------------------

    def closeEvent(self, event) -> None:
        self._poll_timer.stop()

        if self._motion_worker is not None:
            self._motion_worker.quit()
            self._motion_worker.wait(1000)
            self._motion_worker = None

        if self._bot is not None:
            try:
                self._bot.close()
            except Exception:
                pass
            self._bot = None

        if self._connect_worker is not None:
            self._connect_worker.quit()
            self._connect_worker.wait(1000)

        event.accept()


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------

def main() -> None:
    app = QApplication(sys.argv)
    app.setStyle("Fusion")
    win = MagicianWindow()
    win.show()
    sys.exit(app.exec_())


if __name__ == "__main__":
    main()

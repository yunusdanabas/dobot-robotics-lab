"""
00_connectivity_gui.py — MG400 connectivity GUI (ME403): four lab robots, per-row check/fix/demo, optional extra IP.

Run: python 00_connectivity_gui.py
     python 00_connectivity_gui.py --ip 192.168.2.7

Demo is a short MovJ sequence within SAFE_BOUNDS (see robots/mg400/README.md).
"""

from __future__ import annotations

import argparse
import contextlib
import importlib.util
import io
import re
import sys
import time
from pathlib import Path
from typing import Optional

from PyQt5.QtCore import Qt, QThread, pyqtSignal
from PyQt5.QtGui import QFont
from PyQt5.QtWidgets import (
    QApplication, QFrame, QHBoxLayout, QLabel, QLineEdit,
    QMainWindow, QPushButton, QTextEdit, QVBoxLayout, QWidget,
)

# ---------------------------------------------------------------------------
# Load 00_connectivity_check.py via importlib (module name starts with digit)
# ---------------------------------------------------------------------------

_HERE = Path(__file__).parent
_spec = importlib.util.spec_from_file_location(
    "connectivity_check", _HERE / "00_connectivity_check.py"
)
_cc = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(_cc)

# After _cc is loaded, utils_mg400 + dobot_api are already on sys.path.
from utils_mg400 import READY_POSE, DASHBOARD_PORT, MOVE_PORT, ROBOT_IPS  # noqa: E402

# Human-readable labels for each robot
ROBOT_LABELS = {
    1: "Robot 1",
    2: "Robot 2",
    3: "Robot 3",
    4: "Robot 4",
}
from dobot_api import DobotApiDashboard, DobotApiMove                       # noqa: E402

# ---------------------------------------------------------------------------
# Demo waypoints
# ---------------------------------------------------------------------------

DEMO_SEQUENCE = [
    (300,   0,  50, 0),   # 1. READY_POSE (home)
    (300,   0, 110, 0),   # 2. lift up
    (300,  80,  80, 0),   # 3. sweep Y+
    (300, -80,  80, 0),   # 4. sweep Y-
    (300,   0,  50, 0),   # 5. return home
]

_DEMO_LABELS = {1: "home", 2: "lift", 3: "sweep Y+", 4: "sweep Y-", 5: "home"}

# ---------------------------------------------------------------------------
# Status badge colours
# ---------------------------------------------------------------------------

_STATUS_STYLE: dict[str, tuple[str, str]] = {
    "—":           ("#bdc3c7", "#2c3e50"),
    "CHECKING":    ("#2980b9", "white"),
    "OK":          ("#27ae60", "white"),
    "DISABLED":    ("#e67e22", "white"),
    "FEED BUSY":   ("#e67e22", "white"),
    "ERROR":       ("#c0392b", "white"),
    "PORT ERROR":  ("#c0392b", "white"),
    "UNREACHABLE": ("#c0392b", "white"),
    "API ERROR":   ("#c0392b", "white"),
    "FIXING":      ("#16a085", "white"),
    "MOVING":      ("#8e44ad", "white"),
}


def _determine_status(result: dict) -> str:
    p1  = result["p1"]
    p2  = result.get("p2")
    bad = [n for n, i in p1["ports"].items() if not i["ok"]]

    if not p1["reachable"]:
        return "UNREACHABLE"
    if "dashboard" in bad or "move" in bad:
        return "PORT ERROR"
    if p2 is None or p2.get("connect_error"):
        return "API ERROR"

    mode = p2.get("mode_raw")
    if mode == 9:
        return "ERROR"
    if mode == 4:
        return "DISABLED"
    if "feed" in bad:
        return "FEED BUSY"
    if mode in (5, 6, 7, 10, 11):
        return "OK"
    return f"MODE {mode}"


def _is_valid_ip(s: str) -> bool:
    parts = s.strip().split(".")
    if len(parts) != 4:
        return False
    try:
        return all(0 <= int(p) <= 255 for p in parts)
    except ValueError:
        return False


def _payload_snippet(payload: object, limit: int = 140) -> str:
    """One-line view of raw payload for diagnostics panel."""
    try:
        # Uses the same normalisation helper as the CLI checker.
        text = _cc._normalize_response(payload)
    except Exception:
        text = "" if payload is None else str(payload)
    compact = (
        text.replace("\r", "\\r")
        .replace("\n", "\\n")
        .replace("\t", "\\t")
        .strip()
    )
    if len(compact) <= limit:
        return compact
    return compact[: limit - 1] + "…"


# ---------------------------------------------------------------------------
# Detail / log helpers
# ---------------------------------------------------------------------------

def _build_check_detail(robot_id: int, ip: str, label: str,
                         result: dict, sdk_log: str) -> str:
    p1      = result["p1"]
    p2      = result.get("p2")
    actions = result.get("actions", [])
    lines: list[str] = []

    lines.append(f"{label}  ({ip})")
    lines.append("─" * 54)
    lines.append("")
    lines.append("[Phase 1] Network")
    lines.append(f"  Ping: {'OK' if p1['reachable'] else 'FAILED'}")
    for name, info in p1["ports"].items():
        tag = "OPEN" if info["ok"] else "REFUSED"
        lines.append(f"  Port {info['port']} ({name:9s}): {tag:7s}  {info['detail']}")
    conns = p1["local_connections"]
    if conns:
        lines.append(f"  *** {len(conns)} lingering connection(s) from this PC ***")
        for c in conns:
            lines.append(f"      {c}")
    else:
        note = p1.get("local_connections_note")
        if note:
            lines.append(f"  {note}")
        else:
            lines.append("  No lingering connections from this PC.")

    lines.append("")
    lines.append("[Phase 2] Application")
    if p2 is None:
        lines.append("  Skipped.")
    elif p2.get("connect_error"):
        lines.append(f"  Connect error: {p2['connect_error']}")
    else:
        version = p2.get("version")
        lines.append(f"  Version:   {version if version else 'unavailable'}")
        mode_raw  = p2.get("mode_raw")
        mode_name = p2.get("mode_name", "N/A")
        label_m = f"{mode_raw} = {mode_name}" if mode_raw is not None else mode_name
        lines.append(f"  Mode:      {label_m}")
        err_ids = p2.get("error_ids")
        if err_ids is None:
            lines.append("  Error IDs: unavailable (payload parse failed)")
        else:
            lines.append(f"  Error IDs: {err_ids if err_ids else 'none'}")
        pose = p2.get("pose")
        if pose:
            x, y, z, r = pose
            lines.append(f"  Pose:      X={x:.1f}  Y={y:.1f}  Z={z:.1f}  R={r:.1f}  mm/°")
        angles = p2.get("angles")
        if angles:
            j1, j2, j3, j4 = angles
            lines.append(f"  Joints:    J1={j1:.1f}  J2={j2:.1f}  J3={j3:.1f}  J4={j4:.1f}  deg")
        query_errors = p2.get("query_errors", [])
        if query_errors:
            lines.append("  API payload issues:")
            for issue in query_errors:
                lines.append(f"    - {issue}")
            raw_payloads = p2.get("raw", {})
            if raw_payloads:
                lines.append("  Raw payload snippets:")
                for cmd, payload in raw_payloads.items():
                    lines.append(f"    - {cmd}: {_payload_snippet(payload)!r}")

    lines.append("")
    lines.append("[Phase 3] Remediation")
    for action in actions:
        lines.append(f"  {action}")

    if sdk_log.strip():
        lines.append("")
        lines.append("[SDK log]")
        for ln in sdk_log.strip().splitlines():
            lines.append(f"  {ln}")

    return "\n".join(lines)


def _append_section(existing: str, new_text: str) -> str:
    sep = "\n" + "─" * 54 + "\n" if existing.strip() else ""
    return existing + sep + new_text


# ---------------------------------------------------------------------------
# Shared: clear errors via dashboard, return new mode
# ---------------------------------------------------------------------------

def _fix_errors(dashboard: DobotApiDashboard, log: list[str],
                buf: io.StringIO) -> int:
    with contextlib.redirect_stdout(buf):
        err_resp = _cc._normalize_response(dashboard.GetErrorID())
    nums    = re.findall(r"\d+", err_resp)
    err_ids = [int(n) for n in nums if int(n) != 0]
    log.append(f"  Error IDs: {err_ids if err_ids else 'none'}")

    with contextlib.redirect_stdout(buf):
        mode_resp = _cc._normalize_response(dashboard.RobotMode())
    mode = _cc.parse_robot_mode(mode_resp)
    log.append(f"  Mode: {mode} = {_cc.ROBOT_MODE.get(mode, '?')}")

    if mode == 9 or err_ids:
        log.append("  Calling ClearError() + Continue()...")
        with contextlib.redirect_stdout(buf):
            dashboard.ClearError()
            dashboard.Continue()
        time.sleep(1.5)

        with contextlib.redirect_stdout(buf):
            mode_resp = _cc._normalize_response(dashboard.RobotMode())
        mode = _cc.parse_robot_mode(mode_resp)
        with contextlib.redirect_stdout(buf):
            err_resp2 = _cc._normalize_response(dashboard.GetErrorID())
        remaining = [int(n) for n in re.findall(r"\d+", err_resp2) if int(n) != 0]

        log.append(f"  Mode after clear: {mode} = {_cc.ROBOT_MODE.get(mode, '?')}")
        log.append(f"  Remaining errors: {remaining if remaining else 'none'}")
        if mode == 9:
            log.append(
                "  Robot still in ERROR — hardware intervention needed.\n"
                "  Check: E-stop button, collision guards, joint limits.\n"
                "  If those are clear, power-cycle the robot."
            )
    else:
        log.append("  No errors — nothing to clear.")

    return mode


# ---------------------------------------------------------------------------
# Workers
# ---------------------------------------------------------------------------

class CheckWorker(QThread):
    done = pyqtSignal(int, dict, str)   # robot_id, result, detail_text

    def __init__(self, robot_id: int, ip: str, label: str):
        super().__init__()
        self.robot_id = robot_id
        self.ip       = ip
        self.label    = label

    def run(self) -> None:
        buf = io.StringIO()
        with contextlib.redirect_stdout(buf):
            p1      = _cc.phase1_network(self.ip)
            dash_ok = p1["ports"]["dashboard"]["ok"]
            p2      = _cc.phase2_application(self.ip, dash_ok)
            actions = _cc.phase3_remediation(self.ip, p1, p2)
        result = {"p1": p1, "p2": p2, "actions": actions}
        text   = _build_check_detail(
            self.robot_id, self.ip, self.label, result, buf.getvalue()
        )
        self.done.emit(self.robot_id, result, text)


class FixWorker(QThread):
    done = pyqtSignal(int, bool, str)

    def __init__(self, robot_id: int, ip: str, label: str):
        super().__init__()
        self.robot_id = robot_id
        self.ip       = ip
        self.label    = label

    def run(self) -> None:
        log: list[str] = [f"[Fix Errors] {self.label}  ({self.ip})"]
        dashboard = None
        success   = False
        buf       = io.StringIO()
        try:
            with contextlib.redirect_stdout(buf):
                dashboard = DobotApiDashboard(self.ip, DASHBOARD_PORT)
            log.append("  Connected to dashboard.")
            mode = _fix_errors(dashboard, log, buf)
            success = (mode != 9)
        except Exception as exc:
            log.append(f"  EXCEPTION: {exc}")
        finally:
            sdk = buf.getvalue().strip()
            if sdk:
                log.append("  [SDK log]")
                for ln in sdk.splitlines():
                    log.append(f"    {ln}")
            if dashboard is not None:
                try:
                    dashboard.close()
                except Exception:
                    pass
        self.done.emit(self.robot_id, success, "\n".join(log))


class DemoWorker(QThread):
    done      = pyqtSignal(int, bool, str)
    step_done = pyqtSignal(int, int, int)   # robot_id, step, total

    def __init__(self, robot_id: int, ip: str, label: str):
        super().__init__()
        self.robot_id = robot_id
        self.ip       = ip
        self.label    = label

    def run(self) -> None:
        log: list[str] = [f"[Demo Move] {self.label}  ({self.ip})"]
        dashboard = None
        move_api  = None
        success   = False
        buf       = io.StringIO()
        try:
            with contextlib.redirect_stdout(buf):
                dashboard = DobotApiDashboard(self.ip, DASHBOARD_PORT)
                move_api  = DobotApiMove(self.ip, MOVE_PORT)
            log.append("  Connected.")

            mode = _fix_errors(dashboard, log, buf)
            if mode == 9:
                log.append("  Cannot proceed — robot still in ERROR.")
                self.done.emit(self.robot_id, False, "\n".join(log))
                return

            if mode == 4:
                log.append("  Enabling robot...")
                with contextlib.redirect_stdout(buf):
                    dashboard.EnableRobot()
                time.sleep(2.0)
                with contextlib.redirect_stdout(buf):
                    mode_resp = _cc._normalize_response(dashboard.RobotMode())
                mode = _cc.parse_robot_mode(mode_resp)
                log.append(f"  Mode after enable: {mode} = {_cc.ROBOT_MODE.get(mode, '?')}")

            total = len(DEMO_SEQUENCE)
            for i, (x, y, z, r) in enumerate(DEMO_SEQUENCE, start=1):
                lbl = _DEMO_LABELS.get(i, f"step {i}")
                log.append(f"  [{i}/{total}] {lbl}: X={x}  Y={y}  Z={z}  R={r}")
                with contextlib.redirect_stdout(buf):
                    move_api.MovJ(x, y, z, r)
                    move_api.Sync()
                self.step_done.emit(self.robot_id, i, total)

            log.append("  Demo complete.")
            with contextlib.redirect_stdout(buf):
                dashboard.DisableRobot()
            log.append("  Robot disabled.")
            success = True

        except Exception as exc:
            log.append(f"  EXCEPTION: {exc}")
        finally:
            sdk = buf.getvalue().strip()
            if sdk:
                log.append("  [SDK log]")
                for ln in sdk.splitlines():
                    log.append(f"    {ln}")
            for obj in (move_api, dashboard):
                if obj is not None:
                    try:
                        obj.close()
                    except Exception:
                        pass
        self.done.emit(self.robot_id, success, "\n".join(log))


# ---------------------------------------------------------------------------
# Per-robot row widget
# ---------------------------------------------------------------------------

class RobotRow(QWidget):
    check_requested = pyqtSignal(int)
    fix_requested   = pyqtSignal(int)
    demo_requested  = pyqtSignal(int)

    def __init__(self, robot_id: int, ip: str, label: str,
                 parent: Optional[QWidget] = None):
        super().__init__(parent)
        self.robot_id = robot_id
        self.ip       = ip
        self.label    = label

        layout = QHBoxLayout(self)
        layout.setContentsMargins(8, 4, 8, 4)

        mono      = QFont("monospace", 10)
        mono_bold = QFont("monospace", 10, QFont.Bold)

        lbl_name = QLabel(label)
        lbl_name.setFixedWidth(80)
        lbl_name.setFont(mono_bold)
        layout.addWidget(lbl_name)

        lbl_ip = QLabel(ip)
        lbl_ip.setFixedWidth(112)
        lbl_ip.setFont(mono)
        layout.addWidget(lbl_ip)

        self.status_lbl = QLabel("—")
        self.status_lbl.setFixedWidth(100)
        self.status_lbl.setAlignment(Qt.AlignCenter)
        self.status_lbl.setFont(QFont("monospace", 9, QFont.Bold))
        self._apply_status("—")
        layout.addWidget(self.status_lbl)

        layout.addStretch()

        self.check_btn = QPushButton("Check")
        self.check_btn.setFixedWidth(68)
        self.check_btn.clicked.connect(lambda: self.check_requested.emit(self.robot_id))
        layout.addWidget(self.check_btn)

        self.fix_btn = QPushButton("Fix")
        self.fix_btn.setFixedWidth(52)
        self.fix_btn.setToolTip("Read errors, ClearError + Continue (no motion)")
        self.fix_btn.clicked.connect(lambda: self.fix_requested.emit(self.robot_id))
        layout.addWidget(self.fix_btn)

        self.demo_btn = QPushButton("Demo")
        self.demo_btn.setFixedWidth(62)
        self.demo_btn.setToolTip(
            "Fix errors → Enable → lift + sweep demo → Disable"
        )
        self.demo_btn.clicked.connect(lambda: self.demo_requested.emit(self.robot_id))
        layout.addWidget(self.demo_btn)

    def _apply_status(self, status: str) -> None:
        bg, fg = _STATUS_STYLE.get(status, ("#95a5a6", "white"))
        self.status_lbl.setText(status)
        self.status_lbl.setStyleSheet(
            f"background-color:{bg}; color:{fg}; "
            "border-radius:4px; padding:2px 6px;"
        )

    def _set_busy(self, status: str) -> None:
        self._apply_status(status)
        for btn in (self.check_btn, self.fix_btn, self.demo_btn):
            btn.setEnabled(False)

    def _set_idle(self) -> None:
        for btn in (self.check_btn, self.fix_btn, self.demo_btn):
            btn.setEnabled(True)

    def set_checking(self) -> None:
        self._set_busy("CHECKING")

    def set_fixing(self) -> None:
        self._set_busy("FIXING")

    def set_moving(self) -> None:
        self._set_busy("MOVING")

    def set_check_done(self, result: dict) -> None:
        self._apply_status(_determine_status(result))
        self._set_idle()

    def set_fix_done(self) -> None:
        self._apply_status("—")
        self._set_idle()

    def set_demo_done(self) -> None:
        self._apply_status("—")
        self._set_idle()

    def update_step(self, step: int, total: int) -> None:
        self.status_lbl.setText(f"{step}/{total}")


# ---------------------------------------------------------------------------
# Main window
# ---------------------------------------------------------------------------

class MainWindow(QMainWindow):
    def __init__(self, robots: dict[int, str]) -> None:
        """
        robots: ordered dict of {robot_id: ip} to display initially.
        """
        super().__init__()
        self.setWindowTitle("MG400 Connectivity & Control")
        self.setMinimumSize(700, 500)

        # Live robot registry — extended when user adds custom IPs
        self._robot_ips: dict[int, str] = dict(robots)
        self._robot_labels: dict[int, str] = {}
        self._custom_counter = 100   # IDs for user-added custom rows

        self._check_workers: dict[int, CheckWorker] = {}
        self._fix_workers:   dict[int, FixWorker]   = {}
        self._demo_workers:  dict[int, DemoWorker]  = {}
        self._rows:          dict[int, RobotRow]    = {}

        central = QWidget()
        self.setCentralWidget(central)
        root = QVBoxLayout(central)
        root.setSpacing(8)
        root.setContentsMargins(12, 12, 12, 12)

        # ── Top bar ──────────────────────────────────────────────────
        top = QHBoxLayout()
        title = QLabel("MG400 Connectivity & Control")
        title.setFont(QFont("sans-serif", 12, QFont.Bold))
        top.addWidget(title)
        top.addStretch()
        self.check_all_btn = QPushButton("Check All")
        self.check_all_btn.setFixedWidth(88)
        self.check_all_btn.clicked.connect(self._check_all)
        top.addWidget(self.check_all_btn)
        self.fix_all_btn = QPushButton("Fix All")
        self.fix_all_btn.setFixedWidth(74)
        self.fix_all_btn.setToolTip("Clear errors on all visible robots (no motion)")
        self.fix_all_btn.clicked.connect(self._fix_all)
        top.addWidget(self.fix_all_btn)
        root.addLayout(top)

        # ── Add-robot bar ─────────────────────────────────────────────
        add_bar = QHBoxLayout()
        add_lbl = QLabel("Direct connect:")
        add_lbl.setFont(QFont("sans-serif", 9))
        add_lbl.setStyleSheet("color:#7f8c8d;")
        add_bar.addWidget(add_lbl)

        self.ip_input = QLineEdit()
        self.ip_input.setPlaceholderText("192.168.2.x")
        self.ip_input.setFixedWidth(130)
        self.ip_input.setFont(QFont("monospace", 10))
        self.ip_input.returnPressed.connect(self._add_custom)
        add_bar.addWidget(self.ip_input)

        self.add_btn = QPushButton("Add")
        self.add_btn.setFixedWidth(52)
        self.add_btn.clicked.connect(self._add_custom)
        add_bar.addWidget(self.add_btn)

        self.add_status = QLabel("")
        self.add_status.setFont(QFont("sans-serif", 9))
        self.add_status.setStyleSheet("color:#c0392b;")
        add_bar.addWidget(self.add_status)
        add_bar.addStretch()
        root.addLayout(add_bar)

        # ── Column headers ───────────────────────────────────────────
        hdr = QHBoxLayout()
        hdr.setContentsMargins(8, 0, 8, 0)
        for text, width in [("Robot", 80), ("IP", 112), ("Status", 100)]:
            lbl = QLabel(text)
            lbl.setFixedWidth(width)
            lbl.setFont(QFont("sans-serif", 9))
            lbl.setStyleSheet("color:#7f8c8d;")
            hdr.addWidget(lbl)
        hdr.addStretch()
        for text in ("Check", "Fix", "Demo"):
            lbl = QLabel(text)
            lbl.setFont(QFont("sans-serif", 9))
            lbl.setStyleSheet("color:#7f8c8d;")
            lbl.setAlignment(Qt.AlignCenter)
            hdr.addWidget(lbl)
        root.addLayout(hdr)

        sep1 = QFrame()
        sep1.setFrameShape(QFrame.HLine)
        sep1.setFrameShadow(QFrame.Sunken)
        root.addWidget(sep1)

        # ── Robot rows ───────────────────────────────────────────────
        self._rows_layout = QVBoxLayout()
        self._rows_layout.setSpacing(2)
        for rid, ip in robots.items():
            self._add_row(rid, ip)
        root.addLayout(self._rows_layout)

        sep2 = QFrame()
        sep2.setFrameShape(QFrame.HLine)
        sep2.setFrameShadow(QFrame.Sunken)
        root.addWidget(sep2)

        # ── Detail panel ─────────────────────────────────────────────
        detail_hdr = QHBoxLayout()
        detail_hdr.addWidget(QLabel("Details"))
        detail_hdr.addStretch()
        clear_btn = QPushButton("Clear")
        clear_btn.setFixedWidth(58)
        clear_btn.clicked.connect(lambda: self.detail.clear())
        detail_hdr.addWidget(clear_btn)
        root.addLayout(detail_hdr)

        self.detail = QTextEdit()
        self.detail.setReadOnly(True)
        self.detail.setFont(QFont("monospace", 9))
        self.detail.setMinimumHeight(200)
        self.detail.setPlaceholderText(
            "Check  — connectivity diagnostics\n"
            "Fix    — read & clear errors (no motion)\n"
            "Demo   — fix → enable → lift + sweep → disable\n\n"
            "Tip: type an IP in 'Direct connect' above and click Add to\n"
            "connect to a robot not in the list."
        )
        root.addWidget(self.detail)

    # ── Row management ────────────────────────────────────────────────

    def _robot_label(self, robot_id: int) -> str:
        return self._robot_labels.get(robot_id, f"Robot {robot_id}")

    def _add_row(self, robot_id: int, ip: str,
                 label: Optional[str] = None) -> RobotRow:
        lbl = label or ROBOT_LABELS.get(robot_id) or (
            f"Robot {robot_id}" if robot_id in ROBOT_IPS else ip
        )
        self._robot_labels[robot_id] = lbl
        row = RobotRow(robot_id, ip, lbl)
        row.check_requested.connect(self._check_one)
        row.fix_requested.connect(self._fix_one)
        row.demo_requested.connect(self._demo_one)
        self._rows_layout.addWidget(row)
        self._rows[robot_id] = row
        return row

    def _add_custom(self) -> None:
        ip = self.ip_input.text().strip()
        if not _is_valid_ip(ip):
            self.add_status.setText("Invalid IP")
            return
        # Already present?
        if ip in self._robot_ips.values():
            self.add_status.setText("Already in list")
            return
        self.add_status.setText("")
        rid = self._custom_counter
        self._custom_counter += 1
        self._robot_ips[rid] = ip
        self._add_row(rid, ip)   # label defaults to the IP string
        self.ip_input.clear()

    # ── Internal helpers ──────────────────────────────────────────────

    def _append(self, text: str) -> None:
        current = self.detail.toPlainText()
        self.detail.setText(_append_section(current, text))
        self.detail.verticalScrollBar().setValue(
            self.detail.verticalScrollBar().maximum()
        )

    # ── Check ─────────────────────────────────────────────────────────

    def _check_one(self, robot_id: int) -> None:
        if robot_id in self._check_workers:
            return
        ip     = self._robot_ips[robot_id]
        label  = self._robot_label(robot_id)
        worker = CheckWorker(robot_id, ip, label)
        worker.done.connect(self._on_check_done)
        self._check_workers[robot_id] = worker
        self._rows[robot_id].set_checking()
        worker.start()

    def _check_all(self) -> None:
        for rid in list(self._rows):
            self._check_one(rid)

    def _on_check_done(self, robot_id: int, result: dict, text: str) -> None:
        self._rows[robot_id].set_check_done(result)
        self._check_workers.pop(robot_id, None)
        self.detail.setText(text)

    # ── Fix ───────────────────────────────────────────────────────────

    def _fix_one(self, robot_id: int) -> None:
        if robot_id in self._fix_workers:
            return
        ip     = self._robot_ips[robot_id]
        label  = self._robot_label(robot_id)
        worker = FixWorker(robot_id, ip, label)
        worker.done.connect(self._on_fix_done)
        self._fix_workers[robot_id] = worker
        self._rows[robot_id].set_fixing()
        worker.start()

    def _fix_all(self) -> None:
        for rid in list(self._rows):
            self._fix_one(rid)

    def _on_fix_done(self, robot_id: int, success: bool, log_text: str) -> None:
        self._rows[robot_id].set_fix_done()
        self._fix_workers.pop(robot_id, None)
        self._append(log_text)
        self._check_one(robot_id)   # auto-refresh badge

    # ── Demo ──────────────────────────────────────────────────────────

    def _demo_one(self, robot_id: int) -> None:
        if robot_id in self._demo_workers:
            return
        ip     = self._robot_ips[robot_id]
        label  = self._robot_label(robot_id)
        worker = DemoWorker(robot_id, ip, label)
        worker.done.connect(self._on_demo_done)
        worker.step_done.connect(self._on_step)
        self._demo_workers[robot_id] = worker
        self._rows[robot_id].set_moving()
        worker.start()

    def _on_step(self, robot_id: int, step: int, total: int) -> None:
        if robot_id in self._rows:
            self._rows[robot_id].update_step(step, total)

    def _on_demo_done(self, robot_id: int, success: bool, log_text: str) -> None:
        self._rows[robot_id].set_demo_done()
        self._demo_workers.pop(robot_id, None)
        self._append(log_text)
        self._check_one(robot_id)   # auto-refresh badge


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------

def main() -> None:
    parser = argparse.ArgumentParser(
        description="MG400 connectivity & control GUI",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=(
            "Examples:\n"
            "  python 00_connectivity_gui.py              # all 4 robots\n"
            "  python 00_connectivity_gui.py --ip 192.168.2.7  # all 4 + extra row\n"
        ),
    )
    parser.add_argument(
        "--ip", type=str, metavar="ADDR",
        help="Also add a row for this IP (e.g. a robot connected directly by cable).",
    )
    args = parser.parse_args()

    # Always start with all 4 lab robots
    robots = dict(ROBOT_IPS)

    app = QApplication(sys.argv)
    app.setStyle("Fusion")
    win = MainWindow(robots)
    # If --ip was given, add the extra row after the window is built
    if args.ip:
        if _is_valid_ip(args.ip):
            win.ip_input.setText(args.ip)
            win._add_custom()
        else:
            print(f"Warning: --ip {args.ip!r} is not a valid IP address, ignored.",
                  file=sys.stderr)
    win.show()
    sys.exit(app.exec_())


if __name__ == "__main__":
    main()

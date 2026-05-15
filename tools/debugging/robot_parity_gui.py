#!/usr/bin/env python3
"""
robot_parity_gui.py - PyQt5 GUI for real/simulation parity diagnostics.

The GUI wraps robot_parity_diagnostic.py so command-line and GUI runs produce
the same JSONL records. Real robot access is disabled until the confirmation
checkbox is enabled and a confirmation dialog is accepted.
"""

from __future__ import annotations

import argparse
from dataclasses import dataclass
from datetime import datetime, timezone
import json
from pathlib import Path
import sys
import traceback
from typing import Any

try:
    import robot_parity_diagnostic as diag
except ImportError:
    sys.path.insert(0, str(Path(__file__).resolve().parent))
    import robot_parity_diagnostic as diag

try:
    from PyQt5.QtCore import Qt, QThread, QTimer, QUrl, pyqtSignal
    from PyQt5.QtGui import QDesktopServices, QFont, QImage, QPixmap
    from PyQt5.QtWidgets import (
        QApplication,
        QCheckBox,
        QComboBox,
        QDoubleSpinBox,
        QFileDialog,
        QFormLayout,
        QFrame,
        QGridLayout,
        QGroupBox,
        QHBoxLayout,
        QHeaderView,
        QLabel,
        QLineEdit,
        QMainWindow,
        QMessageBox,
        QPushButton,
        QRadioButton,
        QSpinBox,
        QSplitter,
        QStackedWidget,
        QTabWidget,
        QTableWidget,
        QTableWidgetItem,
        QTextEdit,
        QVBoxLayout,
        QWidget,
    )
    PYQT_IMPORT_ERROR = None
except ImportError as exc:  # --help should still work without PyQt5 installed.
    PYQT_IMPORT_ERROR = exc


RESULT_COLUMNS = [
    "Case",
    "Command",
    "Expected Pose",
    "Sim Status",
    "Sim Error",
    "Real Status",
    "Real Error",
    "Sim vs Real",
]

MG400_ROBOT_IPS = {
    1: "192.168.2.7",
    2: "192.168.2.10",
    3: "192.168.2.9",
    4: "192.168.2.6",
}


@dataclass(frozen=True)
class RunRequest:
    robot: str
    target: str
    suite: str
    backend: str | None
    output_path: Path
    samples: int
    settle: float
    sample_interval: float
    speed: int
    keep_going: bool
    no_return_home: bool
    confirm_real: bool
    probe_only: bool
    gui: bool
    port: str | None = None
    ip: str | None = None
    robot_id: int | None = None

    def to_namespace(self) -> argparse.Namespace:
        return argparse.Namespace(
            robot=self.robot,
            target=self.target,
            suite=self.suite,
            case=None,
            backend=self.backend,
            gui=self.gui,
            confirm_real=self.confirm_real,
            dry_run=False,
            probe_only=self.probe_only,
            output=str(self.output_path),
            samples=self.samples,
            settle=self.settle,
            sample_interval=self.sample_interval,
            keep_going=self.keep_going,
            no_return_home=self.no_return_home,
            port=self.port,
            ip=self.ip,
            robot_id=self.robot_id,
            speed=self.speed,
        )


def _stamp() -> str:
    return datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")


def _format_pose(values: Any) -> str:
    if values is None:
        return "-"
    try:
        x, y, z, r = (float(v) for v in values)
    except Exception:
        return str(values)
    return f"X={x:.1f} Y={y:.1f} Z={z:.1f} R={r:.1f}"


def _format_error(error: dict[str, Any] | None) -> str:
    if not error:
        return "-"
    return f"{float(error.get('max_abs_xyz_mm', 0.0)):.2f} mm / {float(error.get('rotation_deg', 0.0)):.2f} deg"


def _record_summary(record: dict[str, Any]) -> str:
    if record.get("type") == "case_result":
        lines = [
            f"[{record.get('target')}] {record.get('case')} - {record.get('status')}",
            f"  command: {record.get('command')} values={record.get('values')}",
            f"  expected: {_format_pose(record.get('expected_pose'))}",
            f"  after:    {_format_pose(record.get('pose_after'))}",
            f"  error:    {_format_error(record.get('pose_error_vs_expected'))}",
            f"  output record timestamp: {record.get('timestamp')}",
        ]
        raw = ((record.get("after") or {}).get("raw") or {})
        if raw:
            lines.append("  raw:")
            for key, value in raw.items():
                text = str(value).replace("\n", "\\n")
                if len(text) > 180:
                    text = text[:177] + "..."
                lines.append(f"    {key}: {text}")
        if record.get("error"):
            lines.append(f"  error: {record['error']}")
        return "\n".join(lines)
    if record.get("type") == "probe_result":
        snap = record.get("snapshot") or {}
        return "\n".join(
            [
                f"[probe:{record.get('target')}] {record.get('status')}",
                f"  pose:   {_format_pose(snap.get('pose'))}",
                f"  joints: {snap.get('joints', '-')}",
            ]
        )
    if record.get("type") == "parallel_case_result":
        sim_r = record.get("sim") or {}
        real_r = record.get("real") or {}
        diff_mm = record.get("pose_diff_mm")
        diff_deg = record.get("pose_diff_deg")
        diff_text = f"{diff_mm:.2f} mm / {diff_deg:.2f} deg" if diff_mm is not None else "N/A"
        joint_diff = record.get("joint_diff_deg")
        if joint_diff is not None:
            diff_text = f"{diff_text} | joint {joint_diff:.2f} deg"
        lines = [
            f"[parallel] {record.get('case')} — {record.get('status')}",
            f"  sim:  {record.get('sim_error') or sim_r.get('status', '?')}  pose={_format_pose(sim_r.get('pose_after'))}",
            f"  real: {record.get('real_error') or real_r.get('status', '?')}  pose={_format_pose(real_r.get('pose_after'))}",
            f"  diff: {diff_text}",
        ]
        return "\n".join(lines)
    if record.get("type") == "run_error":
        return f"[run error] {record.get('error')}"
    return json.dumps(record, indent=2, sort_keys=True)


def _successful_case_records(path: Path) -> dict[tuple[str, str, str], dict[str, Any]]:
    return diag.successful_case_records(diag.iter_jsonl(path))


if PYQT_IMPORT_ERROR is None:

    class DiagnosticRunWorker(QThread):
        record_ready = pyqtSignal(dict)
        log_ready = pyqtSignal(str)
        done = pyqtSignal(str, bool, str, str)  # target, success, message, output_path

        def __init__(self, request: RunRequest) -> None:
            super().__init__()
            self.request = request
            self._stop_requested = False
            self.client = None

        def request_stop(self) -> None:
            self._stop_requested = True

        def run(self) -> None:
            args = self.request.to_namespace()
            output = self.request.output_path
            rid = diag.run_id(args.robot, args.target)
            failures = 0
            close_error = None
            client = None
            stopped = False

            try:
                diag.validate_run_args(args)
                cases = diag.selected_cases(args)
                with diag.JsonlWriter(output) as writer:
                    writer.write(diag.run_metadata(args, output, rid))
                    try:
                        client = diag.build_client(args)
                        self.client = client
                        self.log_ready.emit(f"Connecting to {args.robot} {args.target}...")
                        client.connect()
                        if args.probe_only:
                            record = diag.run_probe(client, args, rid)
                            writer.write(record)
                            self.record_ready.emit(record)
                            failures += int(record.get("status") != "ok")
                        else:
                            for case in cases:
                                if self._stop_requested:
                                    stopped = True
                                    break
                                self.log_ready.emit(f"Running {args.target}: {case.name}")
                                record = diag.run_case(client, args, case, rid)
                                writer.write(record)
                                self.record_ready.emit(record)
                                if record.get("status") != "ok":
                                    failures += 1
                                    if not args.keep_going:
                                        break
                    except Exception as exc:
                        failures += 1
                        record = {
                            "type": "run_error",
                            "timestamp": diag.utc_now(),
                            "run_id": rid,
                            "robot": args.robot,
                            "target": args.target,
                            "status": "error",
                            "error": f"{type(exc).__name__}: {exc}",
                            "traceback": traceback.format_exc(),
                        }
                        writer.write(record)
                        self.record_ready.emit(record)
                    finally:
                        if client is not None:
                            try:
                                client.close()
                            except Exception as exc:
                                close_error = f"{type(exc).__name__}: {exc}"
                                failures += 1
                        writer.write(
                            {
                                "type": "run_end",
                                "timestamp": diag.utc_now(),
                                "run_id": rid,
                                "status": "stopped" if stopped else "failed" if failures else "ok",
                                "failures": failures,
                                "close_error": close_error,
                            }
                        )
                success = failures == 0 and not stopped
                if stopped:
                    msg = f"{args.target} run stopped after current case: {output}"
                else:
                    msg = f"{args.target} run {'completed' if success else 'finished with errors'}: {output}"
                self.done.emit(args.target, success, msg, str(output))
            except BaseException as exc:
                msg = f"{type(exc).__name__}: {exc}"
                self.done.emit(args.target, False, msg, str(output))


    class CompareWorker(QThread):
        diff_ready = pyqtSignal(str, str, dict)  # suite, case, diff
        done = pyqtSignal(bool, str, str)        # success, message, summary_path

        def __init__(
            self,
            sim_path: Path,
            real_path: Path,
            summary_path: Path,
            pos_tol: float = 10.0,
            rot_tol: float = 5.0,
        ) -> None:
            super().__init__()
            self.sim_path = sim_path
            self.real_path = real_path
            self.summary_path = summary_path
            self.pos_tol = pos_tol
            self.rot_tol = rot_tol

        def run(self) -> None:
            try:
                sim_records = _successful_case_records(self.sim_path)
                real_records = _successful_case_records(self.real_path)
                common = sorted(set(sim_records) & set(real_records))
                if not common:
                    self.done.emit(False, "No matching successful case_result records found.", "")
                    return

                rows = []
                failures = 0
                for robot, suite, case in common:
                    diff = diag.compare_pose_records(sim_records[(robot, suite, case)], real_records[(robot, suite, case)])
                    diff["robot"] = robot
                    diff["suite"] = suite
                    diff["case"] = case
                    diff["pass"] = (
                        diff["max_position_error_mm"] <= self.pos_tol
                        and diff["rotation_error_deg"] <= self.rot_tol
                    )
                    failures += int(not diff["pass"])
                    rows.append(diff)
                    self.diff_ready.emit(suite, case, diff)

                summary = {
                    "timestamp": diag.utc_now(),
                    "sim_jsonl": str(self.sim_path),
                    "real_jsonl": str(self.real_path),
                    "pos_tol_mm": self.pos_tol,
                    "rot_tol_deg": self.rot_tol,
                    "matched_cases": len(common),
                    "failures": failures,
                    "rows": rows,
                    "missing_from_sim_or_failed": [list(k) for k in sorted(set(real_records) - set(sim_records))],
                    "missing_from_real_or_failed": [list(k) for k in sorted(set(sim_records) - set(real_records))],
                }
                self.summary_path.parent.mkdir(parents=True, exist_ok=True)
                self.summary_path.write_text(json.dumps(summary, indent=2, sort_keys=True), encoding="utf-8")
                msg = f"Compared {len(common)} case(s), failures={failures}: {self.summary_path}"
                self.done.emit(failures == 0, msg, str(self.summary_path))
            except Exception as exc:
                self.done.emit(False, f"{type(exc).__name__}: {exc}", "")


    class ParallelRunWorker(QThread):
        record_ready = pyqtSignal(dict)
        log_ready = pyqtSignal(str)
        done = pyqtSignal(str, bool, str, str)  # "parallel", success, message, output_path

        def __init__(self, sim_request: RunRequest, real_request: RunRequest) -> None:
            super().__init__()
            self.sim_request = sim_request
            self.real_request = real_request
            self._stop_requested = False
            self.sim_client = None

        def request_stop(self) -> None:
            self._stop_requested = True

        def run(self) -> None:
            import copy, threading as _threading
            sim_args = self.sim_request.to_namespace()
            real_args = self.real_request.to_namespace()
            robot = sim_args.robot
            rid = diag.run_id(robot, "parallel")
            output = self.sim_request.output_path
            failures = 0
            stopped = False

            try:
                diag.validate_run_args(sim_args)
                cases = diag.selected_cases(sim_args)
                sim_client = diag.build_client(sim_args)
                self.sim_client = sim_client
                real_client = diag.build_client(real_args)

                output.parent.mkdir(parents=True, exist_ok=True)
                self.log_ready.emit(f"Connecting sim ({sim_args.backend}) and real (id={real_args.robot_id})…")

                with diag.JsonlWriter(output) as writer:
                    writer.write(diag.run_metadata(sim_args, output, rid))
                    try:
                        sim_client.connect()
                        real_client.connect()
                        self.log_ready.emit(f"Running {len(cases)} case(s) in lockstep parallel…")

                        for case in cases:
                            if self._stop_requested:
                                stopped = True
                                break
                            self.log_ready.emit(f"Parallel: {case.name}")
                            s_slot: list = [None, None]
                            r_slot: list = [None, None]

                            def _sim(slot=s_slot, _c=case):
                                try:
                                    slot[0] = diag.run_case(sim_client, sim_args, _c, rid)
                                except Exception as exc:
                                    slot[1] = f"{type(exc).__name__}: {exc}"

                            def _real(slot=r_slot, _c=case):
                                try:
                                    slot[0] = diag.run_case(real_client, real_args, _c, rid)
                                except Exception as exc:
                                    slot[1] = f"{type(exc).__name__}: {exc}"

                            t_s = _threading.Thread(target=_sim, daemon=True)
                            t_r = _threading.Thread(target=_real, daemon=True)
                            t_s.start(); t_r.start()
                            t_s.join(timeout=120)
                            t_r.join(timeout=120)
                            if t_s.is_alive():
                                s_slot[1] = "timeout: sim move did not complete within 120 s"
                            if t_r.is_alive():
                                r_slot[1] = "timeout: real move did not complete within 120 s"

                            sim_result, sim_err = s_slot
                            real_result, real_err = r_slot
                            diff = diag._pose_diff(
                                sim_result.get("pose_after") if sim_result else None,
                                real_result.get("pose_after") if real_result else None,
                            )
                            joint_diffs = diag._parallel_joint_diffs(case, sim_result, real_result)
                            ok = bool(
                                sim_result and real_result and not sim_err and not real_err
                                and sim_result.get("status") == "ok"
                                and real_result.get("status") == "ok"
                            )
                            record = {
                                "type": "parallel_case_result",
                                "timestamp": diag.utc_now(),
                                "run_id": rid,
                                "case": case.name,
                                "suite": case.suite,
                                "robot": case.robot,
                                "sim": sim_result,
                                "sim_error": sim_err,
                                "real": real_result,
                                "real_error": real_err,
                                "pose_diff_mm": diff.get("mm") if diff else None,
                                "pose_diff_deg": diff.get("deg") if diff else None,
                                "joint_diff_deg": joint_diffs.get("joint_diff_deg"),
                                "firmware_joint_diff_deg": joint_diffs.get("firmware_joint_diff_deg"),
                                "status": "ok" if ok else "error",
                            }
                            writer.write(record)
                            self.record_ready.emit(record)
                            if not ok:
                                failures += 1
                                if not sim_args.keep_going:
                                    break

                    except Exception as exc:
                        failures += 1
                        err_record = {
                            "type": "run_error",
                            "timestamp": diag.utc_now(),
                            "run_id": rid,
                            "error": f"{type(exc).__name__}: {exc}",
                            "traceback": traceback.format_exc(),
                        }
                        writer.write(err_record)
                        self.record_ready.emit(err_record)
                    finally:
                        for cl in (sim_client, real_client):
                            try:
                                cl.close()
                            except Exception:
                                pass
                        writer.write({
                            "type": "run_end",
                            "timestamp": diag.utc_now(),
                            "run_id": rid,
                            "status": "stopped" if stopped else "failed" if failures else "ok",
                            "failures": failures,
                        })

                success = failures == 0 and not stopped
                if stopped:
                    msg = f"Parallel run stopped after current case: {output}"
                else:
                    msg = f"Parallel run {'completed' if success else 'with errors'}: {output}"
                self.done.emit("parallel", success, msg, str(output))
            except BaseException as exc:
                self.done.emit("parallel", False, f"{type(exc).__name__}: {exc}", "")


    class FeedPollerThread(QThread):
        pose_ready = pyqtSignal(dict)
        error = pyqtSignal(str)

        def __init__(self, client) -> None:
            super().__init__()
            self._client = client
            self._stop = False

        def request_interrupt(self) -> None:
            self._stop = True

        def run(self) -> None:
            import math, time
            try:
                self._client.start_feed()
            except Exception as exc:
                self.error.emit(str(exc))
                self._stop = True
                return
            while not self._stop:
                try:
                    snap = self._client.snapshot()
                    feed = self._client.read_feed_pose()
                    feed_diff = None
                    if feed and snap.get("pose"):
                        dash = snap["pose"]
                        if all(v is not None for v in (dash + feed)):
                            feed_diff = math.sqrt(
                                sum((a - b) ** 2 for a, b in zip(dash[:3], feed[:3]))
                            )
                    snap["feed_diff_mm"] = feed_diff
                    self.pose_ready.emit(snap)
                except Exception as exc:
                    self.error.emit(str(exc))
                time.sleep(0.05)


    class CommandWorker(QThread):
        done = pyqtSignal(str, bool, str)   # target_name, success, message

        def __init__(self, cmd_fn, targets) -> None:
            super().__init__()
            self._cmd_fn = cmd_fn
            self._targets = targets  # list[tuple[str, BaseRobotClient]]

        def run(self) -> None:
            import threading
            if len(self._targets) == 1:
                name, client = self._targets[0]
                try:
                    self._cmd_fn(client)
                    self.done.emit(name, True, "OK")
                except Exception as exc:
                    self.done.emit(name, False, str(exc))
                return
            results: dict = {}
            def _run_one(name: str, client) -> None:
                try:
                    self._cmd_fn(client)
                    results[name] = (True, "OK")
                except Exception as exc:
                    results[name] = (False, str(exc))
            threads = [threading.Thread(target=_run_one, args=(n, c))
                       for n, c in self._targets]
            for t in threads:
                t.start()
            for t in threads:
                t.join(timeout=30)
            for name, (success, msg) in results.items():
                self.done.emit(name, success, msg)


    class QuickActionWorker(QThread):
        """Connect to the real robot, run a single action, then disconnect."""
        done = pyqtSignal(bool, str)  # success, message

        def __init__(self, request: RunRequest, action: str) -> None:
            super().__init__()
            self.request = request  # action: "clear_errors" | "go_home"
            self.action = action

        def run(self) -> None:
            args = self.request.to_namespace()
            client = None
            try:
                client = diag.build_client(args)
                client.connect()
                if self.action == "clear_errors":
                    success, detail = _do_fix_errors(client)
                    if not success:
                        raise RuntimeError(detail)
                    msg = f"fix errors: {detail}"
                elif self.action == "go_home":
                    client.move_cartesian((300.0, 0.0, 50.0, 0.0))
                    msg = f"{self.action} OK"
                else:
                    msg = f"{self.action} OK"
                self.done.emit(True, msg)
            except Exception as exc:
                self.done.emit(False, str(exc))
            finally:
                if client is not None:
                    try:
                        client.close()
                    except Exception:
                        pass


    def _do_fix_errors(client) -> tuple[bool, str]:
        """Read errors/mode, clear only if needed, re-verify. Matches Fix-button logic."""
        import time as _time
        lines = []
        try:
            error_raw = client.dashboard.GetErrorID()
            err_ids = diag.parse_mg400_error_ids(error_raw) or []
        except Exception as exc:
            return False, f"could not read errors: {type(exc).__name__}: {exc}"
        lines.append(f"errors={err_ids if err_ids else 'none'}")
        try:
            mode = diag.parse_mg400_mode(client.dashboard.RobotMode())
        except Exception as exc:
            return False, " | ".join(lines + [f"could not read mode: {type(exc).__name__}: {exc}"])
        lines.append(f"mode={_MODE_STRINGS.get(mode, mode)}")
        if mode is None:
            return False, " | ".join(lines + ["could not determine robot mode"])
        if mode == 9 or err_ids:
            diag.require_mg400_success(client.dashboard.ClearError(), "ClearError")
            diag.require_mg400_success(client.dashboard.Continue(), "continue")
            _time.sleep(1.5)
            try:
                mode = diag.parse_mg400_mode(client.dashboard.RobotMode())
            except Exception as exc:
                return False, " | ".join(lines + [f"could not read mode after clear: {type(exc).__name__}: {exc}"])
            try:
                remaining = diag.parse_mg400_error_ids(client.dashboard.GetErrorID()) or []
            except Exception as exc:
                return False, " | ".join(lines + [f"could not read errors after clear: {type(exc).__name__}: {exc}"])
            lines.append(f"after: mode={_MODE_STRINGS.get(mode, mode)} errors={remaining if remaining else 'none'}")
            if mode == 9:
                lines.append("still ERROR — hardware intervention needed")
                return False, " | ".join(lines)
        else:
            lines.append("no errors to clear")
        return True, " | ".join(lines)


    class _ViewportLabel(QLabel):
        """QLabel subclass that supports left-drag (rotate) and scroll (zoom)."""
        def __init__(self, live_tab, *args, **kwargs):
            super().__init__(*args, **kwargs)
            self._tab = live_tab
            self.setMouseTracking(True)

        def mousePressEvent(self, ev):
            if ev.button() == Qt.LeftButton:
                self._tab._drag_last = ev.pos()

        def mouseReleaseEvent(self, ev):
            if ev.button() == Qt.LeftButton:
                self._tab._drag_last = None

        def mouseMoveEvent(self, ev):
            last = self._tab._drag_last
            if last is None:
                return
            dx = ev.x() - last.x()
            dy = ev.y() - last.y()
            self._tab._cam_yaw   = (self._tab._cam_yaw + dx * 0.4) % 360.0
            self._tab._cam_pitch = max(-89.0, min(-5.0, self._tab._cam_pitch + dy * 0.3))
            self._tab._drag_last = ev.pos()

        def wheelEvent(self, ev):
            delta = ev.angleDelta().y()
            factor = 0.9 if delta > 0 else 1.1
            self._tab._cam_distance = max(0.2, min(3.0, self._tab._cam_distance * factor))

    _MODE_STRINGS: dict = {
        1: "INIT", 2: "BRAKE_OPEN", 3: "DISABLED", 4: "ENABLED",
        5: "RUNNING", 6: "RECORDING", 7: "REPLAYING", 9: "ERROR",
    }

    class LiveControlTab(QWidget):
        def __init__(self, parent=None) -> None:
            super().__init__(parent)
            self._sim_client = None
            self._real_client = None
            self._feed_thread = None
            self._sim_timer = None
            self._viewport_timer = None
            self._command_worker = None
            self._sim_tool_state: bool = False
            self._real_tool_state: bool = False
            self._pending_commands: int = 0
            self._cam_yaw: float      = 45.0
            self._cam_pitch: float    = -45.0
            self._cam_distance: float = 0.8
            self._drag_last           = None
            self._build_ui()

        # ------------------------------------------------------------------ #
        #  UI construction                                                     #
        # ------------------------------------------------------------------ #
        def _build_ui(self) -> None:
            root = QVBoxLayout(self)
            root.setSpacing(6)
            root.setContentsMargins(8, 8, 8, 8)
            root.addWidget(self._make_conn_bar())
            root.addWidget(self._make_middle(), stretch=2)
            root.addWidget(self._make_cmd_panel())

        def _make_conn_bar(self) -> QGroupBox:
            box = QGroupBox("Connections")
            lay = QHBoxLayout(box)
            lay.addWidget(QLabel("Backend:"))
            self.backend_combo = QComboBox()
            self.backend_combo.addItems(["pybullet", "mujoco"])
            lay.addWidget(self.backend_combo)
            self.connect_sim_btn = QPushButton("Connect Sim")
            self.connect_sim_btn.clicked.connect(self._connect_sim)
            lay.addWidget(self.connect_sim_btn)
            self.disconnect_sim_btn = QPushButton("Disconnect Sim")
            self.disconnect_sim_btn.clicked.connect(self._disconnect_sim)
            self.disconnect_sim_btn.setEnabled(False)
            lay.addWidget(self.disconnect_sim_btn)
            self.sim_status_lbl = QLabel("● SIM: OFF")
            self.sim_status_lbl.setStyleSheet("color:#c0392b; font-weight:bold;")
            lay.addWidget(self.sim_status_lbl)
            sep = QFrame()
            sep.setFrameShape(QFrame.VLine)
            sep.setFrameShadow(QFrame.Sunken)
            lay.addSpacing(12)
            lay.addWidget(sep)
            lay.addSpacing(12)
            lay.addWidget(QLabel("Robot:"))
            self.robot_id_combo = QComboBox()
            for rid, ip in MG400_ROBOT_IPS.items():
                self.robot_id_combo.addItem(f"Robot {rid}  ({ip})", userData=rid)
            lay.addWidget(self.robot_id_combo)
            self.connect_real_btn = QPushButton("Connect Real")
            self.connect_real_btn.clicked.connect(self._connect_real)
            lay.addWidget(self.connect_real_btn)
            self.disconnect_real_btn = QPushButton("Disconnect Real")
            self.disconnect_real_btn.clicked.connect(self._disconnect_real)
            self.disconnect_real_btn.setEnabled(False)
            lay.addWidget(self.disconnect_real_btn)
            self.real_status_lbl = QLabel("● REAL: OFF")
            self.real_status_lbl.setStyleSheet("color:#c0392b; font-weight:bold;")
            lay.addWidget(self.real_status_lbl)
            lay.addSpacing(8)
            self.clear_errors_btn = QPushButton("Clear Errors")
            self.clear_errors_btn.setEnabled(False)
            self.clear_errors_btn.clicked.connect(self._clear_real_errors)
            lay.addWidget(self.clear_errors_btn)
            self.go_home_real_btn = QPushButton("Go Home")
            self.go_home_real_btn.setEnabled(False)
            self.go_home_real_btn.clicked.connect(self._go_home_real)
            lay.addWidget(self.go_home_real_btn)
            lay.addStretch()
            return box

        def _make_middle(self) -> QSplitter:
            splitter = QSplitter(Qt.Horizontal)
            self.viewport_lbl = _ViewportLabel(self, "No Sim Connected")
            self.viewport_lbl.setAlignment(Qt.AlignCenter)
            self.viewport_lbl.setMinimumSize(320, 240)
            self.viewport_lbl.setStyleSheet(
                "background:#1a1a1a; color:#555; font-size:14px;"
            )
            splitter.addWidget(self.viewport_lbl)
            params_widget = QWidget()
            params_lay = QHBoxLayout(params_widget)
            params_lay.setSpacing(6)
            self._sim_labels: dict = {}
            self._real_labels: dict = {}
            sim_box = QGroupBox("Simulation")
            sim_form = QFormLayout(sim_box)
            for key in ["X (mm)", "Y (mm)", "Z (mm)", "R (°)",
                        "J1 (°)", "J2 (°)", "J3 (°)", "J4 (°)", "Mode", "Tool"]:
                lbl = QLabel("—")
                lbl.setFont(QFont("monospace", 10))
                sim_form.addRow(key + ":", lbl)
                self._sim_labels[key] = lbl
            params_lay.addWidget(sim_box)
            real_box = QGroupBox("Real Robot")
            real_form = QFormLayout(real_box)
            for key in ["X (mm)", "Y (mm)", "Z (mm)", "R (°)",
                        "J1 (°)", "J2 (°)", "J3 (°)", "J4 (°)",
                        "Mode", "Tool", "Feed Δ (mm)", "Δ Sim (mm)"]:
                lbl = QLabel("—")
                lbl.setFont(QFont("monospace", 10))
                real_form.addRow(key + ":", lbl)
                self._real_labels[key] = lbl
            params_lay.addWidget(real_box)
            splitter.addWidget(params_widget)
            splitter.setSizes([480, 420])
            return splitter

        def _make_cmd_panel(self) -> QGroupBox:
            box = QGroupBox("Commands")
            lay = QVBoxLayout(box)
            type_row = QHBoxLayout()
            type_row.addWidget(QLabel("Type:"))
            self.cmd_type_combo = QComboBox()
            self.cmd_type_combo.addItems(["Joint", "Cartesian", "End-Effector"])
            type_row.addWidget(self.cmd_type_combo)
            type_row.addStretch()
            lay.addLayout(type_row)
            self.cmd_stack = QStackedWidget()
            self.cmd_type_combo.currentIndexChanged.connect(self.cmd_stack.setCurrentIndex)
            self.cmd_stack.addWidget(self._make_joint_panel())
            self.cmd_stack.addWidget(self._make_cart_panel())
            self.cmd_stack.addWidget(self._make_ee_panel())
            lay.addWidget(self.cmd_stack)
            send_row = QHBoxLayout()
            self.send_sim_btn  = QPushButton("Send to Sim")
            self.send_real_btn = QPushButton("Send to Real")
            self.send_both_btn = QPushButton("Send to Both")
            self._send_btns = [self.send_sim_btn, self.send_real_btn, self.send_both_btn]
            self.send_sim_btn.clicked.connect(lambda: self._send_command("sim"))
            self.send_real_btn.clicked.connect(lambda: self._send_command("real"))
            self.send_both_btn.clicked.connect(lambda: self._send_command("both"))
            for btn in self._send_btns:
                send_row.addWidget(btn)
            self.cmd_status_lbl = QLabel("")
            send_row.addSpacing(16)
            send_row.addWidget(self.cmd_status_lbl)
            send_row.addStretch()
            lay.addLayout(send_row)
            self._update_send_buttons()
            return box

        def _make_joint_panel(self) -> QWidget:
            w = QWidget()
            lay = QVBoxLayout(w)
            spin_row = QHBoxLayout()
            self._joint_spins = []
            for name, lo, hi in [("J1", -160.0, 160.0), ("J2", -25.0, 85.0),
                                  ("J3", -25.0, 105.0), ("J4", -180.0, 180.0)]:
                spin_row.addWidget(QLabel(f"{name} (°):"))
                sb = QDoubleSpinBox()
                sb.setRange(lo, hi)
                sb.setSingleStep(1.0)
                sb.setDecimals(1)
                sb.setMinimumWidth(80)
                spin_row.addWidget(sb)
                self._joint_spins.append(sb)
            lay.addLayout(spin_row)
            mode_row = QHBoxLayout()
            self.joint_abs_radio = QRadioButton("Absolute")
            self.joint_rel_radio = QRadioButton("Relative")
            self.joint_abs_radio.setChecked(True)
            mode_row.addWidget(self.joint_abs_radio)
            mode_row.addWidget(self.joint_rel_radio)
            mode_row.addSpacing(20)
            STEP = 5.0
            for idx, name in enumerate(["J1", "J2", "J3", "J4"]):
                btn_m = QPushButton(f"{name}−")
                btn_m.setFixedWidth(48)
                btn_p = QPushButton(f"{name}+")
                btn_p.setFixedWidth(48)
                btn_m.clicked.connect(
                    lambda _, i=idx: self._joint_spins[i].setValue(
                        self._joint_spins[i].value() - STEP))
                btn_p.clicked.connect(
                    lambda _, i=idx: self._joint_spins[i].setValue(
                        self._joint_spins[i].value() + STEP))
                mode_row.addWidget(btn_m)
                mode_row.addWidget(btn_p)
            mode_row.addStretch()
            lay.addLayout(mode_row)
            return w

        def _make_cart_panel(self) -> QWidget:
            w = QWidget()
            lay = QVBoxLayout(w)
            spin_row = QHBoxLayout()
            self._cart_spins = []
            for name, lo, hi, step in [("X", 60.0, 400.0, 5.0),
                                        ("Y", -220.0, 220.0, 5.0),
                                        ("Z", 5.0, 140.0, 5.0),
                                        ("R", -170.0, 170.0, 1.0)]:
                spin_row.addWidget(QLabel(f"{name}:"))
                sb = QDoubleSpinBox()
                sb.setRange(lo, hi)
                sb.setSingleStep(step)
                sb.setDecimals(1)
                sb.setMinimumWidth(80)
                spin_row.addWidget(sb)
                self._cart_spins.append(sb)
            lay.addLayout(spin_row)
            mode_row = QHBoxLayout()
            self.cart_movj_radio = QRadioButton("MovJ")
            self.cart_movl_radio = QRadioButton("MovL")
            self.cart_movj_radio.setChecked(True)
            mode_row.addWidget(self.cart_movj_radio)
            mode_row.addWidget(self.cart_movl_radio)
            mode_row.addStretch()
            lay.addLayout(mode_row)
            return w

        def _make_ee_panel(self) -> QWidget:
            w = QWidget()
            lay = QHBoxLayout(w)
            lay.addWidget(QLabel("Tool index:"))
            self.ee_tool_combo = QComboBox()
            self.ee_tool_combo.addItems(["0", "1"])
            lay.addWidget(self.ee_tool_combo)
            self.ee_toggle_btn = QPushButton("Suction: OFF")
            self.ee_toggle_btn.setCheckable(True)
            self.ee_toggle_btn.clicked.connect(
                lambda checked: self.ee_toggle_btn.setText(
                    "Suction: ON" if checked else "Suction: OFF"
                )
            )
            lay.addWidget(self.ee_toggle_btn)
            lay.addStretch()
            return w

        # ------------------------------------------------------------------ #
        #  Live update helpers                                                 #
        # ------------------------------------------------------------------ #
        def _update_param_labels(
            self,
            labels: dict,
            snap: dict,
            tool_state: bool,
            feed_diff=None,
        ) -> None:
            pose = snap.get("pose") or (None,) * 4
            joints = snap.get("joints") or (None,) * 4
            mode = (snap.get("raw") or {}).get("mode")
            error_ids = (snap.get("raw") or {}).get("error_ids") or []
            values = {
                "X (mm)": f"{pose[0]:.2f}"   if pose[0]   is not None else "—",
                "Y (mm)": f"{pose[1]:.2f}"   if pose[1]   is not None else "—",
                "Z (mm)": f"{pose[2]:.2f}"   if pose[2]   is not None else "—",
                "R (°)":  f"{pose[3]:.2f}"   if pose[3]   is not None else "—",
                "J1 (°)": f"{joints[0]:.2f}" if joints[0] is not None else "—",
                "J2 (°)": f"{joints[1]:.2f}" if joints[1] is not None else "—",
                "J3 (°)": f"{joints[2]:.2f}" if joints[2] is not None else "—",
                "J4 (°)": f"{joints[3]:.2f}" if joints[3] is not None else "—",
                "Mode": _MODE_STRINGS.get(mode, str(mode)) if mode is not None else "—",
                "Tool": "ON" if tool_state else "OFF",
            }
            if "Feed Δ (mm)" in labels:
                values["Feed Δ (mm)"] = (
                    f"{feed_diff:.3f}" if feed_diff is not None else "—"
                )
            err_style = "color:#cc2200; font-weight:bold;"
            for key, val in values.items():
                if key not in labels:
                    continue
                lbl = labels[key]
                if lbl.text() != val:
                    lbl.setText(val)
                    if error_ids and key == "Mode":
                        lbl.setStyleSheet(err_style)
                    else:
                        self._flash_label(lbl)

        @staticmethod
        def _flash_label(lbl: QLabel) -> None:
            lbl.setStyleSheet("color:#00cc44;")
            def _clear(ref=lbl):
                try:
                    ref.setStyleSheet("")
                except RuntimeError:
                    pass  # widget already destroyed
            QTimer.singleShot(300, _clear)

        # ------------------------------------------------------------------ #
        #  Connection management                                               #
        # ------------------------------------------------------------------ #
        def _connect_sim(self) -> None:
            backend = self.backend_combo.currentText()
            client = diag.MG400SimClient(backend=backend, gui=False)
            try:
                client.connect()
            except Exception as exc:
                QMessageBox.critical(self, "Sim Connect Error", str(exc))
                return
            self._sim_client = client
            self._sim_timer = QTimer(self)
            self._sim_timer.setInterval(50)
            self._sim_timer.timeout.connect(self._poll_sim)
            self._sim_timer.start()
            self._viewport_timer = QTimer(self)
            self._viewport_timer.setInterval(66)
            self._viewport_timer.timeout.connect(self._refresh_viewport)
            self._viewport_timer.start()
            self.sim_status_lbl.setText("● SIM: CONNECTED")
            self.sim_status_lbl.setStyleSheet("color:#27ae60; font-weight:bold;")
            self.connect_sim_btn.setEnabled(False)
            self.disconnect_sim_btn.setEnabled(True)
            self.backend_combo.setEnabled(False)
            self._update_send_buttons()

        def _disconnect_sim(self) -> None:
            if self._sim_timer:
                self._sim_timer.stop()
                self._sim_timer = None
            if self._viewport_timer:
                self._viewport_timer.stop()
                self._viewport_timer = None
            if self._sim_client:
                try:
                    self._sim_client.close()
                except Exception:
                    pass
                self._sim_client = None
            for lbl in self._sim_labels.values():
                lbl.setText("—")
            self.viewport_lbl.clear()
            self.viewport_lbl.setText("No Sim Connected")
            self.sim_status_lbl.setText("● SIM: OFF")
            self.sim_status_lbl.setStyleSheet("color:#c0392b; font-weight:bold;")
            self.connect_sim_btn.setEnabled(True)
            self.disconnect_sim_btn.setEnabled(False)
            self.backend_combo.setEnabled(True)
            self._update_send_buttons()

        def _connect_real(self) -> None:
            rid = self.robot_id_combo.currentData()
            ip = MG400_ROBOT_IPS[rid]
            client = diag.MG400RealClient(
                ip=ip, robot_id=None, speed=30, return_home=False
            )
            try:
                client.connect()
            except Exception as exc:
                QMessageBox.critical(self, "Real Connect Error", str(exc))
                return
            self._real_client = client
            thread = FeedPollerThread(client)
            thread.pose_ready.connect(self._on_real_pose)
            thread.error.connect(self._on_real_error)
            thread.start()
            self._feed_thread = thread
            self.real_status_lbl.setText("● REAL: CONNECTED")
            self.real_status_lbl.setStyleSheet("color:#27ae60; font-weight:bold;")
            self.connect_real_btn.setEnabled(False)
            self.disconnect_real_btn.setEnabled(True)
            self.robot_id_combo.setEnabled(False)
            self._update_send_buttons()

        def _disconnect_real(self) -> None:
            if self._feed_thread:
                self._feed_thread.request_interrupt()
                self._feed_thread.wait(2000)
                self._feed_thread = None
            if self._real_client:
                try:
                    self._real_client.close()
                except Exception:
                    pass
                self._real_client = None
            for lbl in self._real_labels.values():
                lbl.setText("—")
            self.real_status_lbl.setText("● REAL: OFF")
            self.real_status_lbl.setStyleSheet("color:#c0392b; font-weight:bold;")
            self.connect_real_btn.setEnabled(True)
            self.disconnect_real_btn.setEnabled(False)
            self.robot_id_combo.setEnabled(True)
            self._update_send_buttons()

        def _clear_real_errors(self) -> None:
            if self._real_client is None:
                return
            def _fn(client):
                success, detail = _do_fix_errors(client)
                if not success:
                    raise RuntimeError(detail)
            self._pending_commands = 1
            self._update_send_buttons()
            worker = CommandWorker(_fn, [("real", self._real_client)])
            worker.done.connect(self._on_command_done)
            worker.finished.connect(worker.deleteLater)
            worker.start()
            self._command_worker = worker

        def _go_home_real(self) -> None:
            if self._real_client is None:
                return
            fn = lambda client: client.move_cartesian((300.0, 0.0, 50.0, 0.0))
            self._pending_commands = 1
            self._update_send_buttons()
            worker = CommandWorker(fn, [("real", self._real_client)])
            worker.done.connect(self._on_command_done)
            worker.finished.connect(worker.deleteLater)
            worker.start()
            self._command_worker = worker

        # ------------------------------------------------------------------ #
        #  Polling callbacks                                                   #
        # ------------------------------------------------------------------ #
        def _poll_sim(self) -> None:
            if self._sim_client is None:
                return
            try:
                snap = self._sim_client.snapshot()
                self._update_param_labels(
                    self._sim_labels, snap, tool_state=self._sim_tool_state
                )
            except Exception as exc:
                self.cmd_status_lbl.setText(f"Sim poll error: {str(exc)[:60]}")

        def _on_real_pose(self, snap: dict) -> None:
            self._update_param_labels(
                self._real_labels,
                snap,
                tool_state=self._real_tool_state,
                feed_diff=snap.get("feed_diff_mm"),
            )
            if self._sim_client is not None:
                try:
                    import math as _math
                    sim_snap = self._sim_client.snapshot()
                    sp = sim_snap.get("pose")
                    rp = snap.get("pose")
                    if sp and rp and all(v is not None for v in list(sp[:3]) + list(rp[:3])):
                        delta = _math.sqrt(sum((a - b) ** 2 for a, b in zip(sp[:3], rp[:3])))
                        lbl = self._real_labels.get("Δ Sim (mm)")
                        if lbl:
                            lbl.setText(f"{delta:.2f}")
                except Exception:
                    pass

        def _on_real_error(self, msg: str) -> None:
            self.real_status_lbl.setText(f"● REAL: ERR — {msg[:40]}")
            self.real_status_lbl.setStyleSheet("color:#cc2200; font-weight:bold;")

        def _refresh_viewport(self) -> None:
            if self._sim_client is None:
                return
            try:
                frame = self._sim_client.get_camera_frame(
                    480, 360,
                    yaw=self._cam_yaw, pitch=self._cam_pitch,
                    distance=self._cam_distance,
                )
                if frame is None:
                    return
                img = QImage(frame, 480, 360, 480 * 3, QImage.Format_RGB888)
                pix = QPixmap.fromImage(img)
                self.viewport_lbl.setPixmap(
                    pix.scaled(
                        self.viewport_lbl.size(),
                        Qt.KeepAspectRatio,
                        Qt.SmoothTransformation,
                    )
                )
            except Exception as exc:
                self.viewport_lbl.setText(f"Render error: {str(exc)[:50]}")

        # ------------------------------------------------------------------ #
        #  Command dispatch                                                    #
        # ------------------------------------------------------------------ #
        def _update_send_buttons(self) -> None:
            sim_ok  = self._sim_client is not None
            real_ok = self._real_client is not None
            busy = self._pending_commands > 0
            self.send_sim_btn.setEnabled(sim_ok and not busy)
            self.send_real_btn.setEnabled(real_ok and not busy)
            self.send_both_btn.setEnabled(sim_ok and real_ok and not busy)
            self.clear_errors_btn.setEnabled(real_ok and not busy)
            self.go_home_real_btn.setEnabled(real_ok and not busy)

        def _build_command_fn(self):
            cmd_type = self.cmd_type_combo.currentText()
            if cmd_type == "Joint":
                raw = tuple(sb.value() for sb in self._joint_spins)
                if self.joint_rel_radio.isChecked():
                    def fn_rel(client):
                        snap = client.snapshot()
                        cur = snap.get("joints") or (0.0, 0.0, 0.0, 0.0)
                        client.move_joint(tuple(c + d for c, d in zip(cur, raw)))
                    return fn_rel
                return lambda client: client.move_joint(raw)
            if cmd_type == "Cartesian":
                pose = tuple(sb.value() for sb in self._cart_spins)
                if self.cart_movl_radio.isChecked():
                    return lambda client: client.move_cartesian_linear(pose)
                return lambda client: client.move_cartesian(pose)
            if cmd_type == "End-Effector":
                index = int(self.ee_tool_combo.currentText())
                state = self.ee_toggle_btn.isChecked()
                return lambda client: client.set_tool(index, state)
            return None

        def _send_command(self, target: str) -> None:
            fn = self._build_command_fn()
            if fn is None:
                return
            targets = []
            if target in ("sim", "both") and self._sim_client is not None:
                targets.append(("sim", self._sim_client))
            if target in ("real", "both") and self._real_client is not None:
                targets.append(("real", self._real_client))
            if not targets:
                self.cmd_status_lbl.setText("No client connected for this target.")
                return
            self._pending_commands = len(targets)
            self._update_send_buttons()
            worker = CommandWorker(fn, targets)
            worker.done.connect(self._on_command_done)
            worker.finished.connect(worker.deleteLater)
            worker.start()
            self._command_worker = worker

        def _on_command_done(self, target: str, success: bool, message: str) -> None:
            self._pending_commands = max(0, self._pending_commands - 1)
            if self._pending_commands == 0:
                self._update_send_buttons()
            if success:
                self.cmd_status_lbl.setStyleSheet("color:#27ae60;")
                self.cmd_status_lbl.setText(f"{target}: ✓ {message}")
                if self.cmd_type_combo.currentText() == "End-Effector":
                    state = self.ee_toggle_btn.isChecked()
                    if target in ("sim", "both"):
                        self._sim_tool_state = state
                    if target in ("real", "both"):
                        self._real_tool_state = state
            else:
                self.cmd_status_lbl.setStyleSheet("color:#cc2200;")
                self.cmd_status_lbl.setText(f"{target}: ✗ {message[:60]}")

        def shutdown(self) -> None:
            if self._sim_timer:
                self._sim_timer.stop()
            if self._viewport_timer:
                self._viewport_timer.stop()
            if self._feed_thread:
                self._feed_thread.request_interrupt()
                self._feed_thread.wait(3000)
            if self._command_worker and self._command_worker.isRunning():
                self._command_worker.wait(5000)
            if self._sim_client:
                try:
                    self._sim_client.close()
                except Exception:
                    pass
            if self._real_client:
                try:
                    self._real_client.close()
                except Exception:
                    pass


    class DiagnosticWindow(QMainWindow):
        def __init__(self, preset_robot: str = "mg400", output_dir: Path | None = None) -> None:
            super().__init__()
            self.setWindowTitle("Robot Parity Diagnostics")
            self.setMinimumSize(1120, 720)

            self._run_worker: DiagnosticRunWorker | None = None
            self._parallel_worker: ParallelRunWorker | None = None
            self._compare_worker: CompareWorker | None = None
            self._quick_worker: QuickActionWorker | None = None
            self._busy = False
            self._case_rows: dict[tuple[str, str], int] = {}
            self._latest_sim_path: Path | None = None
            self._latest_real_path: Path | None = None
            self._sequence_mode: str | None = None
            self._sequence_stamp: str | None = None
            self._output_dir = output_dir or diag.DEFAULT_RESULTS_DIR
            self._cam_yaw: float      = 45.0
            self._cam_pitch: float    = -45.0
            self._cam_distance: float = 0.8
            self._drag_last           = None
            self._diag_viewport_timer: QTimer | None = None

            self._build_ui()

            idx = self.robot_combo.findData(preset_robot)
            if idx >= 0:
                self.robot_combo.setCurrentIndex(idx)
            self.output_dir_edit.setText(str(self._output_dir))
            self._refresh_ports()
            self._update_robot_controls()
            self._preview_cases()
            self._update_button_state()

        def _build_ui(self) -> None:
            central = QWidget()
            self.setCentralWidget(central)
            root = QVBoxLayout(central)
            root.setSpacing(8)
            root.setContentsMargins(10, 10, 10, 10)

            title = QLabel("Robot Parity Diagnostics")
            title.setFont(QFont("sans-serif", 13, QFont.Bold))
            root.addWidget(title)

            config_box = QGroupBox("Configuration")
            grid = QGridLayout(config_box)
            grid.setHorizontalSpacing(10)
            grid.setVerticalSpacing(6)

            self.robot_combo = QComboBox()
            self.robot_combo.addItem("MG400", userData="mg400")
            self.robot_combo.addItem("Magician", userData="magician")
            self.robot_combo.currentIndexChanged.connect(self._update_robot_controls)
            grid.addWidget(QLabel("Robot:"), 0, 0)
            grid.addWidget(self.robot_combo, 0, 1)

            self.suite_combo = QComboBox()
            suite_labels = {
                "joint": "Joint (safe default)",
                "cartesian": "Cartesian",
                "axis": "Axis Sign",
                "repeatability": "Repeatability",
                "equivalence": "Joint vs Cartesian",
                "modes": "Motion Modes",
                "workspace": "Workspace Conservative",
                "io": "End-Effector IO",
                "connection": "Connection Snapshots",
                "pick_place": "Pick-and-Place",
                "trajectory": "Trajectory Sweep",
                "speed_factor": "Speed Factor",
                "feed_parity": "Feed vs Dashboard",
                "all": "All Suites",
            }
            for suite in diag.SUITE_CHOICES:
                self.suite_combo.addItem(suite_labels.get(suite, suite), userData=suite)
            self.suite_combo.currentIndexChanged.connect(self._preview_cases)
            grid.addWidget(QLabel("Suite:"), 0, 2)
            grid.addWidget(self.suite_combo, 0, 3)

            self.backend_combo = QComboBox()
            grid.addWidget(QLabel("Sim backend:"), 0, 4)
            grid.addWidget(self.backend_combo, 0, 5)

            self.samples_spin = QSpinBox()
            self.samples_spin.setRange(1, 20)
            self.samples_spin.setValue(3)
            grid.addWidget(QLabel("Samples:"), 1, 0)
            grid.addWidget(self.samples_spin, 1, 1)

            self.settle_spin = QDoubleSpinBox()
            self.settle_spin.setRange(0.0, 10.0)
            self.settle_spin.setDecimals(2)
            self.settle_spin.setSingleStep(0.1)
            self.settle_spin.setValue(0.2)
            grid.addWidget(QLabel("Settle s:"), 1, 2)
            grid.addWidget(self.settle_spin, 1, 3)

            self.interval_spin = QDoubleSpinBox()
            self.interval_spin.setRange(0.0, 5.0)
            self.interval_spin.setDecimals(2)
            self.interval_spin.setSingleStep(0.05)
            self.interval_spin.setValue(0.05)
            grid.addWidget(QLabel("Sample interval s:"), 1, 4)
            grid.addWidget(self.interval_spin, 1, 5)

            self.speed_spin = QSpinBox()
            self.speed_spin.setRange(1, 100)
            self.speed_spin.setValue(20)
            grid.addWidget(QLabel("MG400 speed %:"), 2, 0)
            grid.addWidget(self.speed_spin, 2, 1)

            self.sim_gui_check = QCheckBox("Open simulator GUI")
            grid.addWidget(self.sim_gui_check, 2, 2)

            self.keep_going_check = QCheckBox("Keep going after case error")
            grid.addWidget(self.keep_going_check, 2, 3, 1, 2)

            self.return_home_check = QCheckBox("Return home on close")
            self.return_home_check.setChecked(True)
            grid.addWidget(self.return_home_check, 2, 5)

            self.output_dir_edit = QLineEdit()
            self.output_dir_edit.setFont(QFont("monospace", 9))
            self.output_dir_btn = QPushButton("Browse")
            self.output_dir_btn.clicked.connect(self._choose_output_dir)
            grid.addWidget(QLabel("Output dir:"), 3, 0)
            grid.addWidget(self.output_dir_edit, 3, 1, 1, 4)
            grid.addWidget(self.output_dir_btn, 3, 5)

            root.addWidget(config_box)

            target_row = QHBoxLayout()
            self.magician_box = QGroupBox("Magician Real Target")
            mag_layout = QHBoxLayout(self.magician_box)
            mag_layout.addWidget(QLabel("Port:"))
            self.port_combo = QComboBox()
            self.port_combo.setMinimumWidth(180)
            self.port_combo.setFont(QFont("monospace", 9))
            mag_layout.addWidget(self.port_combo)
            self.refresh_ports_btn = QPushButton("Refresh")
            self.refresh_ports_btn.clicked.connect(self._refresh_ports)
            mag_layout.addWidget(self.refresh_ports_btn)
            target_row.addWidget(self.magician_box)

            self.mg400_box = QGroupBox("MG400 Real Target")
            mg_layout = QHBoxLayout(self.mg400_box)
            mg_layout.addWidget(QLabel("Robot:"))
            self.mg400_robot_combo = QComboBox()
            for rid, ip in MG400_ROBOT_IPS.items():
                self.mg400_robot_combo.addItem(f"Robot {rid} ({ip})", userData=rid)
            self.mg400_robot_combo.setMinimumWidth(170)
            mg_layout.addWidget(self.mg400_robot_combo)
            mg_layout.addWidget(QLabel("or IP:"))
            self.mg400_ip_edit = QLineEdit()
            self.mg400_ip_edit.setPlaceholderText("optional 192.168.2.x")
            self.mg400_ip_edit.setFont(QFont("monospace", 9))
            self.mg400_ip_edit.setMinimumWidth(150)
            mg_layout.addWidget(self.mg400_ip_edit)
            target_row.addWidget(self.mg400_box)
            root.addLayout(target_row)

            safety = QHBoxLayout()
            self.confirm_real_check = QCheckBox("Confirm real robot access/motion")
            self.confirm_real_check.setStyleSheet("font-weight:bold; color:#c0392b;")
            self.confirm_real_check.stateChanged.connect(self._update_button_state)
            safety.addWidget(self.confirm_real_check)
            self.status_badge = QLabel("READY")
            self.status_badge.setAlignment(Qt.AlignCenter)
            self.status_badge.setFixedWidth(130)
            self.status_badge.setFont(QFont("monospace", 9, QFont.Bold))
            self._apply_badge("READY")
            safety.addStretch()
            safety.addWidget(self.status_badge)
            root.addLayout(safety)

            action_row = QHBoxLayout()
            self.preview_btn = QPushButton("Preview Cases")
            self.preview_btn.clicked.connect(self._preview_cases)
            action_row.addWidget(self.preview_btn)

            self.probe_real_btn = QPushButton("Probe Real")
            self.probe_real_btn.clicked.connect(self._probe_real)
            action_row.addWidget(self.probe_real_btn)

            self.run_sim_btn = QPushButton("Run Simulation")
            self.run_sim_btn.clicked.connect(self._run_sim)
            action_row.addWidget(self.run_sim_btn)

            self.run_real_btn = QPushButton("Run Real")
            self.run_real_btn.clicked.connect(self._run_real)
            action_row.addWidget(self.run_real_btn)

            self.parallel_check = QCheckBox("Lockstep")
            self.parallel_check.setChecked(True)
            self.parallel_check.setToolTip("Run sim and real in lockstep threads (parallel). Uncheck for sequential.")
            action_row.addWidget(self.parallel_check)

            self.run_both_btn = QPushButton("Run Sim + Real + Compare")
            self.run_both_btn.clicked.connect(self._run_both)
            action_row.addWidget(self.run_both_btn)

            self.compare_btn = QPushButton("Compare Saved Files")
            self.compare_btn.clicked.connect(self._compare_saved)
            action_row.addWidget(self.compare_btn)

            self.open_results_btn = QPushButton("Open Results Folder")
            self.open_results_btn.clicked.connect(self._open_results_folder)
            action_row.addWidget(self.open_results_btn)

            self.stop_btn = QPushButton("Stop Test")
            self.stop_btn.setStyleSheet("color:#c0392b; font-weight:bold;")
            self.stop_btn.clicked.connect(self._stop_test)
            self.stop_btn.setEnabled(False)
            action_row.addWidget(self.stop_btn)

            self.clear_errors_btn = QPushButton("Clear Errors")
            self.clear_errors_btn.clicked.connect(self._clear_errors_real)
            self.clear_errors_btn.setEnabled(False)
            action_row.addWidget(self.clear_errors_btn)

            self.go_home_btn = QPushButton("Go Home")
            self.go_home_btn.clicked.connect(self._go_home_real)
            self.go_home_btn.setEnabled(False)
            action_row.addWidget(self.go_home_btn)

            root.addLayout(action_row)

            sep = QFrame()
            sep.setFrameShape(QFrame.HLine)
            sep.setFrameShadow(QFrame.Sunken)
            root.addWidget(sep)

            self._main_tabs = QTabWidget()

            # --- Diagnostics tab ---
            diag_tab = QWidget()
            diag_lay = QVBoxLayout(diag_tab)
            diag_lay.setContentsMargins(0, 6, 0, 0)

            self.results_table = QTableWidget(0, len(RESULT_COLUMNS))
            self.results_table.setHorizontalHeaderLabels(RESULT_COLUMNS)
            self.results_table.verticalHeader().setVisible(False)
            self.results_table.setAlternatingRowColors(True)
            self.results_table.setSelectionBehavior(QTableWidget.SelectRows)
            self.results_table.horizontalHeader().setSectionResizeMode(QHeaderView.ResizeToContents)
            self.results_table.horizontalHeader().setStretchLastSection(True)

            self._diag_viewport_lbl = _ViewportLabel(self, "No Sim Running")
            self._diag_viewport_lbl.setAlignment(Qt.AlignCenter)
            self._diag_viewport_lbl.setMinimumSize(240, 180)
            self._diag_viewport_lbl.setStyleSheet(
                "background:#1a1a1a; color:#555; font-size:13px;"
            )

            table_viewport_split = QSplitter(Qt.Horizontal)
            table_viewport_split.addWidget(self.results_table)
            table_viewport_split.addWidget(self._diag_viewport_lbl)
            table_viewport_split.setSizes([560, 320])
            diag_lay.addWidget(table_viewport_split, stretch=3)

            log_hdr = QHBoxLayout()
            log_hdr.addWidget(QLabel("Details"))
            log_hdr.addStretch()
            clear_log_btn = QPushButton("Clear")
            clear_log_btn.setFixedWidth(60)
            clear_log_btn.clicked.connect(lambda: self.detail.clear())
            log_hdr.addWidget(clear_log_btn)
            diag_lay.addLayout(log_hdr)

            self.detail = QTextEdit()
            self.detail.setReadOnly(True)
            self.detail.setFont(QFont("monospace", 9))
            self.detail.setMinimumHeight(170)
            self.detail.setPlaceholderText(
                "Preview cases, run simulation, then run real robot.\n"
                "Results are saved as JSONL files in the selected output directory."
            )
            diag_lay.addWidget(self.detail, stretch=2)

            self._main_tabs.addTab(diag_tab, "Diagnostics")

            # --- Live Control tab ---
            self._live_tab = LiveControlTab(parent=self)
            self._main_tabs.addTab(self._live_tab, "Live Control")

            root.addWidget(self._main_tabs, stretch=5)

        def _selected_robot(self) -> str:
            return self.robot_combo.currentData()

        def _selected_suite(self) -> str:
            return self.suite_combo.currentData()

        def _selected_backend(self) -> str | None:
            return self.backend_combo.currentData()

        def _output_dir_path(self) -> Path:
            text = self.output_dir_edit.text().strip()
            return Path(text).expanduser().resolve() if text else diag.DEFAULT_RESULTS_DIR

        def _append_log(self, text: str) -> None:
            current = self.detail.toPlainText()
            sep = "\n" + "-" * 72 + "\n" if current.strip() else ""
            self.detail.setText(current + sep + text)
            self.detail.verticalScrollBar().setValue(self.detail.verticalScrollBar().maximum())

        def _apply_badge(self, status: str) -> None:
            colors = {
                "READY": ("#27ae60", "white"),
                "RUNNING": ("#8e44ad", "white"),
                "ERROR": ("#c0392b", "white"),
                "DONE": ("#2980b9", "white"),
                "COMPARE": ("#16a085", "white"),
            }
            bg, fg = colors.get(status, ("#95a5a6", "white"))
            self.status_badge.setText(status)
            self.status_badge.setStyleSheet(
                f"background-color:{bg}; color:{fg}; border-radius:4px; padding:3px 8px;"
            )

        def _choose_output_dir(self) -> None:
            path = QFileDialog.getExistingDirectory(self, "Select diagnostics output directory", str(self._output_dir_path()))
            if path:
                self.output_dir_edit.setText(path)

        def _refresh_ports(self) -> None:
            current = self.port_combo.currentText() if hasattr(self, "port_combo") else ""
            self.port_combo.clear()
            self.port_combo.addItem("Auto", userData=None)
            try:
                from serial.tools import list_ports

                for port in list_ports.comports():
                    label = f"{port.device}  {port.description or ''}".strip()
                    self.port_combo.addItem(label, userData=port.device)
            except Exception as exc:
                self.port_combo.addItem(f"Port list unavailable: {exc}", userData=None)
            idx = self.port_combo.findText(current)
            if idx >= 0:
                self.port_combo.setCurrentIndex(idx)

        def _update_robot_controls(self) -> None:
            robot = self._selected_robot()
            self.backend_combo.blockSignals(True)
            self.backend_combo.clear()
            if robot == "magician":
                for backend in diag.MAGICIAN_BACKENDS:
                    self.backend_combo.addItem(backend, userData=backend)
                self.backend_combo.setCurrentIndex(0)  # pybullet
            else:
                for backend in diag.MG400_BACKENDS:
                    self.backend_combo.addItem(backend, userData=backend)
                self.backend_combo.setCurrentIndex(0)  # pybullet
            self.backend_combo.blockSignals(False)
            self.magician_box.setVisible(robot == "magician")
            self.mg400_box.setVisible(robot == "mg400")
            self.speed_spin.setEnabled(robot == "mg400")
            self._preview_cases()

        def _update_button_state(self) -> None:
            real_enabled = (not self._busy) and (self._parallel_worker is None) and self.confirm_real_check.isChecked()
            self.preview_btn.setEnabled(not self._busy)
            self.run_sim_btn.setEnabled(not self._busy)
            self.probe_real_btn.setEnabled(real_enabled)
            self.run_real_btn.setEnabled(real_enabled)
            self.run_both_btn.setEnabled(real_enabled)
            self.compare_btn.setEnabled(not self._busy)
            self.open_results_btn.setEnabled(not self._busy)
            running = self._busy or (self._parallel_worker is not None)
            self.stop_btn.setEnabled(running)
            quick_ok = (
                not self._busy
                and self._parallel_worker is None
                and self._quick_worker is None
                and self.confirm_real_check.isChecked()
            )
            self.clear_errors_btn.setEnabled(quick_ok)
            self.go_home_btn.setEnabled(quick_ok)

        def _set_busy(self, busy: bool, status: str = "RUNNING") -> None:
            self._busy = busy
            self._apply_badge(status)
            self._update_button_state()

        def _case_args(self) -> argparse.Namespace:
            return argparse.Namespace(robot=self._selected_robot(), suite=self._selected_suite(), case=None)

        def _preview_cases(self) -> None:
            if not hasattr(self, "results_table"):
                return
            try:
                cases = diag.selected_cases(self._case_args())
            except Exception as exc:
                self._append_log(f"Cannot list cases: {exc}")
                return
            self.results_table.setRowCount(0)
            self._case_rows.clear()
            for case in cases:
                row = self.results_table.rowCount()
                self.results_table.insertRow(row)
                key = (case.suite, case.name)
                self._case_rows[key] = row
                values = {
                    0: case.name,
                    1: case.command,
                    2: _format_pose(diag.expected_pose(case)),
                    3: "-",
                    4: "-",
                    5: "-",
                    6: "-",
                    7: "-",
                }
                for col, text in values.items():
                    item = QTableWidgetItem(text)
                    if col in (3, 5):
                        item.setTextAlignment(Qt.AlignCenter)
                    self.results_table.setItem(row, col, item)
            self.results_table.resizeColumnsToContents()

        def _request_output_path(self, target: str) -> Path:
            stamp = self._sequence_stamp or _stamp()
            robot = self._selected_robot()
            suite = self._selected_suite()
            return self._output_dir_path() / f"{robot}_{suite}_{target}_{stamp}.jsonl"

        def _build_request(self, target: str, probe_only: bool = False, output_path: Path | None = None) -> RunRequest:
            robot = self._selected_robot()
            port = None
            ip = None
            robot_id = None
            if robot == "magician":
                port = self.port_combo.currentData()
            else:
                custom_ip = self.mg400_ip_edit.text().strip()
                if custom_ip:
                    ip = custom_ip
                else:
                    robot_id = self.mg400_robot_combo.currentData()
            return RunRequest(
                robot=robot,
                target=target,
                suite=self._selected_suite(),
                backend=self._selected_backend(),
                output_path=output_path or self._request_output_path(target),
                samples=self.samples_spin.value(),
                settle=self.settle_spin.value(),
                sample_interval=self.interval_spin.value(),
                speed=self.speed_spin.value(),
                keep_going=self.keep_going_check.isChecked(),
                no_return_home=not self.return_home_check.isChecked(),
                confirm_real=self.confirm_real_check.isChecked(),
                probe_only=probe_only,
                gui=self.sim_gui_check.isChecked(),
                port=port,
                ip=ip,
                robot_id=robot_id,
            )

        def _confirm_real_dialog(self, title: str) -> bool:
            if not self.confirm_real_check.isChecked():
                QMessageBox.warning(self, "Real robot not confirmed", "Check 'Confirm real robot access/motion' first.")
                return False
            result = QMessageBox.question(
                self,
                title,
                "This will access the real robot and may move it.\n\n"
                "Make sure the workspace is clear, the correct robot is selected, and DobotStudio is closed.\n\n"
                "Continue?",
                QMessageBox.Yes | QMessageBox.No,
                QMessageBox.No,
            )
            return result == QMessageBox.Yes

        def _start_run(self, request: RunRequest) -> None:
            if self._run_worker is not None:
                return
            request.output_path.parent.mkdir(parents=True, exist_ok=True)
            self._set_busy(True, "RUNNING")
            self._append_log(f"Starting {request.target} run -> {request.output_path}")
            worker = DiagnosticRunWorker(request)
            worker.record_ready.connect(self._on_record_ready)
            worker.log_ready.connect(self._append_log)
            worker.done.connect(self._on_run_done)
            self._run_worker = worker
            worker.start()
            self._start_diag_viewport_timer()

        def _run_sim(self) -> None:
            self._sequence_mode = None
            self._sequence_stamp = None
            self._preview_cases()
            self._start_run(self._build_request("sim"))

        def _run_real(self) -> None:
            if not self._confirm_real_dialog("Run real diagnostics"):
                return
            self._sequence_mode = None
            self._sequence_stamp = None
            self._preview_cases()
            self._start_run(self._build_request("real"))

        def _probe_real(self) -> None:
            if not self._confirm_real_dialog("Probe real robot"):
                return
            self._sequence_mode = None
            self._sequence_stamp = None
            self._preview_cases()
            self._start_run(self._build_request("real", probe_only=True))

        def _run_both(self) -> None:
            if not self._confirm_real_dialog("Run simulation, real robot, then compare"):
                return
            if self.parallel_check.isChecked():
                self._run_parallel()
            else:
                self._sequence_mode = "sim_real_compare"
                self._sequence_stamp = _stamp()
                self._latest_sim_path = None
                self._latest_real_path = None
                self._preview_cases()
                self._start_run(self._build_request("sim", output_path=self._request_output_path("sim")))

        def _run_parallel(self) -> None:
            if self._run_worker is not None or self._parallel_worker is not None:
                return
            stamp = _stamp()
            robot = self._selected_robot()
            suite = self._selected_suite()
            output = self._output_dir_path() / f"{robot}_{suite}_parallel_{stamp}.jsonl"
            output.parent.mkdir(parents=True, exist_ok=True)
            sim_request = self._build_request("sim", output_path=output)
            real_request = self._build_request("real", output_path=output)
            self._preview_cases()
            self._set_busy(True, "RUNNING")
            self._append_log(f"Starting lockstep parallel run → {output}")
            worker = ParallelRunWorker(sim_request, real_request)
            worker.record_ready.connect(self._on_record_ready)
            worker.log_ready.connect(self._append_log)
            worker.done.connect(self._on_parallel_done)
            self._parallel_worker = worker
            worker.start()
            self._start_diag_viewport_timer()

        def _on_parallel_done(self, target: str, success: bool, message: str, output_path: str) -> None:
            self._parallel_worker = None
            self._stop_diag_viewport_timer()
            self._append_log(message)
            self._set_busy(False, "DONE" if success else "ERROR")

        def _on_record_ready(self, record: dict[str, Any]) -> None:
            self._append_log(_record_summary(record))
            rtype = record.get("type")
            if rtype == "case_result":
                key = (record.get("suite"), record.get("case"))
                row = self._case_rows.get(key)
                if row is None:
                    return
                target = record.get("target")
                status_col, err_col = (3, 4) if target == "sim" else (5, 6)
                status = "PASS" if record.get("status") == "ok" else "FAIL"
                self.results_table.setItem(row, status_col, QTableWidgetItem(status))
                self.results_table.setItem(row, err_col, QTableWidgetItem(_format_error(record.get("pose_error_vs_expected"))))
                if status == "FAIL" and record.get("error"):
                    self.results_table.item(row, err_col).setToolTip(record["error"])
            elif rtype == "parallel_case_result":
                key = (record.get("suite"), record.get("case"))
                row = self._case_rows.get(key)
                if row is None:
                    return
                sim_r = record.get("sim") or {}
                real_r = record.get("real") or {}
                sim_ok = not record.get("sim_error") and sim_r.get("status") == "ok"
                real_ok = not record.get("real_error") and real_r.get("status") == "ok"
                self.results_table.setItem(row, 3, QTableWidgetItem("PASS" if sim_ok else "FAIL"))
                self.results_table.setItem(row, 4, QTableWidgetItem(_format_error(sim_r.get("pose_error_vs_expected"))))
                self.results_table.setItem(row, 5, QTableWidgetItem("PASS" if real_ok else "FAIL"))
                self.results_table.setItem(row, 6, QTableWidgetItem(_format_error(real_r.get("pose_error_vs_expected"))))
                diff_mm = record.get("pose_diff_mm")
                diff_deg = record.get("pose_diff_deg")
                diff_text = f"{diff_mm:.2f} mm / {diff_deg:.2f} deg" if diff_mm is not None else "N/A"
                joint_diff = record.get("joint_diff_deg")
                if joint_diff is not None:
                    diff_text = f"{diff_text} | joint {joint_diff:.2f} deg"
                self.results_table.setItem(row, 7, QTableWidgetItem(diff_text))

        def _on_run_done(self, target: str, success: bool, message: str, output_path: str) -> None:
            self._run_worker = None
            self._stop_diag_viewport_timer()
            path = Path(output_path) if output_path else None
            if target == "sim" and path is not None:
                self._latest_sim_path = path
            if target == "real" and path is not None:
                self._latest_real_path = path
            self._append_log(message)

            if self._sequence_mode == "sim_real_compare" and target == "sim":
                if success:
                    self._start_run(self._build_request("real", output_path=self._request_output_path("real")))
                    return
                self._sequence_mode = None
            elif self._sequence_mode == "sim_real_compare" and target == "real":
                self._sequence_mode = None
                if success:
                    self._compare_latest()
                    return

            self._sequence_stamp = None
            self._set_busy(False, "DONE" if success else "ERROR")

        def _compare_summary_path(self) -> Path:
            stamp = _stamp()
            return self._output_dir_path() / f"compare_{self._selected_robot()}_{self._selected_suite()}_{stamp}.json"

        def _start_compare(self, sim_path: Path, real_path: Path) -> None:
            if self._compare_worker is not None:
                return
            self._set_busy(True, "COMPARE")
            self._append_log(f"Comparing:\n  sim:  {sim_path}\n  real: {real_path}")
            worker = CompareWorker(sim_path, real_path, self._compare_summary_path())
            worker.diff_ready.connect(self._on_compare_diff)
            worker.done.connect(self._on_compare_done)
            self._compare_worker = worker
            worker.start()

        def _compare_latest(self) -> None:
            if self._latest_sim_path is None or self._latest_real_path is None:
                QMessageBox.warning(self, "Missing outputs", "Run both simulation and real diagnostics first.")
                return
            self._start_compare(self._latest_sim_path, self._latest_real_path)

        def _compare_saved(self) -> None:
            sim_path, _ = QFileDialog.getOpenFileName(
                self,
                "Select simulation JSONL",
                str(self._output_dir_path()),
                "JSONL files (*.jsonl);;All files (*)",
            )
            if not sim_path:
                return
            real_path, _ = QFileDialog.getOpenFileName(
                self,
                "Select real robot JSONL",
                str(self._output_dir_path()),
                "JSONL files (*.jsonl);;All files (*)",
            )
            if not real_path:
                return
            self._start_compare(Path(sim_path), Path(real_path))

        def _on_compare_diff(self, suite: str, case: str, diff: dict[str, Any]) -> None:
            row = self._case_rows.get((suite, case))
            if row is None:
                return
            tag = "PASS" if diff.get("pass") else "FAIL"
            text = f"{tag}: {diff['max_position_error_mm']:.2f} mm / {diff['rotation_error_deg']:.2f} deg"
            item = QTableWidgetItem(text)
            item.setToolTip(json.dumps(diff, indent=2, sort_keys=True))
            self.results_table.setItem(row, 7, item)

        def _on_compare_done(self, success: bool, message: str, summary_path: str) -> None:
            self._compare_worker = None
            self._append_log(message)
            if summary_path:
                self._append_log(f"Comparison summary saved: {summary_path}")
            self._sequence_stamp = None
            self._set_busy(False, "DONE" if success else "ERROR")

        def _open_results_folder(self) -> None:
            path = self._output_dir_path()
            path.mkdir(parents=True, exist_ok=True)
            QDesktopServices.openUrl(QUrl.fromLocalFile(str(path)))

        # ------------------------------------------------------------------ #
        #  Stop / Quick-action buttons                                         #
        # ------------------------------------------------------------------ #
        def _stop_test(self) -> None:
            worker = self._run_worker or self._parallel_worker
            if worker is None:
                return
            self._append_log("Stop requested — finishing current case then aborting…")
            worker.request_stop()
            self.stop_btn.setEnabled(False)
            def _report_still_running(ref=worker):
                if ref.isRunning():
                    self._append_log(
                        "Stop is cooperative: current motion/socket call is still running. "
                        "Waiting instead of force-terminating to avoid broken MG400 sockets."
                    )
            QTimer.singleShot(2000, _report_still_running)

        def _clear_errors_real(self) -> None:
            if not self.confirm_real_check.isChecked():
                return
            self._run_quick_action("clear_errors")

        def _go_home_real(self) -> None:
            if not self.confirm_real_check.isChecked():
                return
            self._run_quick_action("go_home")

        def _run_quick_action(self, action: str) -> None:
            if self._quick_worker is not None:
                return
            request = RunRequest(
                robot=self._selected_robot(),
                target="real",
                suite=self._selected_suite(),
                backend=self._selected_backend(),
                output_path=self._request_output_path("real"),
                samples=1,
                settle=0.0,
                sample_interval=0.0,
                speed=self.speed_spin.value(),
                keep_going=True,
                no_return_home=True,
                confirm_real=True,
                probe_only=False,
                gui=False,
                port=self.port_combo.currentData() if self._selected_robot() == "magician" else None,
                ip=self.mg400_ip_edit.text().strip() or None,
                robot_id=self.mg400_robot_combo.currentData() if not self.mg400_ip_edit.text().strip() else None,
            )
            self._append_log(f"Quick action: {action}…")
            worker = QuickActionWorker(request, action)
            worker.done.connect(self._on_quick_done)
            worker.finished.connect(worker.deleteLater)
            self._quick_worker = worker
            self._update_button_state()
            worker.start()

        def _on_quick_done(self, success: bool, message: str) -> None:
            self._quick_worker = None
            self._append_log(f"{'✓' if success else '✗'} {message}")
            self._update_button_state()

        # ------------------------------------------------------------------ #
        #  Diagnostics tab viewport                                            #
        # ------------------------------------------------------------------ #
        def _start_diag_viewport_timer(self) -> None:
            if self._diag_viewport_timer is not None:
                return
            t = QTimer(self)
            t.setInterval(66)
            t.timeout.connect(self._refresh_diag_viewport)
            t.start()
            self._diag_viewport_timer = t

        def _stop_diag_viewport_timer(self) -> None:
            if self._diag_viewport_timer:
                self._diag_viewport_timer.stop()
                self._diag_viewport_timer = None
            self._diag_viewport_lbl.clear()
            self._diag_viewport_lbl.setText("No Sim Running")

        def _refresh_diag_viewport(self) -> None:
            client = None
            if self._run_worker is not None:
                client = self._run_worker.client
            elif self._parallel_worker is not None:
                client = self._parallel_worker.sim_client
            if client is None:
                return
            try:
                frame = client.get_camera_frame(
                    320, 240,
                    yaw=self._cam_yaw,
                    pitch=self._cam_pitch,
                    distance=self._cam_distance,
                )
                if frame is None:
                    return
                img = QImage(frame, 320, 240, 320 * 3, QImage.Format_RGB888)
                pix = QPixmap.fromImage(img)
                self._diag_viewport_lbl.setPixmap(
                    pix.scaled(
                        self._diag_viewport_lbl.size(),
                        Qt.KeepAspectRatio,
                        Qt.SmoothTransformation,
                    )
                )
            except Exception as exc:
                self._diag_viewport_lbl.setText(f"Render err: {str(exc)[:40]}")

        def closeEvent(self, event) -> None:  # noqa: N802 - Qt method name
            if (self._run_worker is not None
                    or self._compare_worker is not None
                    or self._parallel_worker is not None):
                QMessageBox.warning(
                    self, "Worker running",
                    "Wait for the current run to finish before closing."
                )
                event.ignore()
                return
            self._stop_diag_viewport_timer()
            if self._quick_worker is not None:
                self._quick_worker.wait(3000)
            self._live_tab.shutdown()
            event.accept()


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Open the robot parity diagnostics GUI.")
    parser.add_argument("--robot", choices=("magician", "mg400"), default="mg400", help="Preselect robot family")
    parser.add_argument("--output-dir", help="Default output directory for JSONL results")
    return parser


def main(argv: list[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    if PYQT_IMPORT_ERROR is not None:
        raise SystemExit(
            "PyQt5 is required for the diagnostics GUI. Install PyQt5 or use "
            f"robot_parity_diagnostic.py from the terminal. Import error: {PYQT_IMPORT_ERROR}"
        )
    output_dir = Path(args.output_dir).expanduser().resolve() if args.output_dir else None
    app = QApplication([sys.argv[0]])
    win = DiagnosticWindow(preset_robot=args.robot, output_dir=output_dir)
    win.show()
    return app.exec_()


if __name__ == "__main__":
    raise SystemExit(main())

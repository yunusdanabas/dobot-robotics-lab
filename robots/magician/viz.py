"""Small live visualizer for Dobot Magician hardware scripts."""

from __future__ import annotations

import multiprocessing as mp
import os


def _viz_enabled() -> bool:
    return os.environ.get("DOBOT_VIZ", "1").lower() not in {"0", "false", "no"}


class RobotViz:
    """Forward commanded poses to a separate Qt/pyqtgraph process."""

    def __init__(self, enabled: bool = True) -> None:
        self._enabled = False
        if not enabled or not _viz_enabled():
            return
        try:
            ctx = mp.get_context("spawn")
            self._queue: mp.Queue = ctx.Queue(maxsize=500)
            self._proc = ctx.Process(target=_viz_process, args=(self._queue,), daemon=True)
            self._proc.start()
            self._enabled = True
        except Exception as exc:
            print(f"[viz] Visualizer disabled: {exc}")

    def attach(self, bot) -> None:
        """Patch bot.move_to so each commanded pose is sent to the plot process."""
        if not self._enabled:
            return
        original = bot.move_to
        queue = self._queue

        def _patched(x, y, z, r, *args, **kwargs):
            result = original(x, y, z, r, *args, **kwargs)
            try:
                queue.put_nowait((float(x), float(y), float(z), float(r)))
            except Exception:
                pass
            return result

        bot.move_to = _patched

    def send(self, x: float, y: float, z: float, r: float) -> None:
        if not self._enabled:
            return
        try:
            self._queue.put_nowait((float(x), float(y), float(z), float(r)))
        except Exception:
            pass

    def close(self) -> None:
        if not self._enabled:
            return
        try:
            self._queue.put_nowait(None)
        except Exception:
            pass
        self._proc.join(timeout=2)
        if self._proc.is_alive():
            self._proc.terminate()


def _viz_process(queue) -> None:
    import sys
    from collections import deque

    from PyQt5.QtCore import Qt, QTimer
    from PyQt5.QtWidgets import QApplication, QMainWindow, QSplitter
    import pyqtgraph as pg

    trail_maxlen = int(os.environ.get("DOBOT_TRAIL", "500"))
    hard_limits = {"x": (115, 320), "y": (-160, 160), "z": (0, 160)}
    safe_bounds = {"x": (120, 315), "y": (-158, 158), "z": (5, 155)}

    class _Window(QMainWindow):
        def __init__(self, q):
            super().__init__()
            self._q = q
            self._trail: deque[tuple[float, float, float]] = deque(maxlen=trail_maxlen)
            self.setWindowTitle("Dobot Magician Live Visualizer")
            self.resize(1200, 600)

            splitter = QSplitter(Qt.Horizontal)
            self.setCentralWidget(splitter)

            self._xy = pg.PlotWidget(title="Top View (XY) - C clears trail")
            self._xy.setAspectLocked(True)
            self._xy.setXRange(90, 340)
            self._xy.setYRange(-180, 180)
            self._xy.setLabel("bottom", "X (mm)")
            self._xy.setLabel("left", "Y (mm)")
            splitter.addWidget(self._xy)

            self._xz = pg.PlotWidget(title="Front View (XZ)")
            self._xz.setXRange(90, 340)
            self._xz.setYRange(-15, 180)
            self._xz.setLabel("bottom", "X (mm)")
            self._xz.setLabel("left", "Z (mm)")
            splitter.addWidget(self._xz)

            self._draw_bounds()
            self._trail_xy = self._xy.plot([], [], pen=pg.mkPen((0, 200, 255), width=2))
            self._trail_xz = self._xz.plot([], [], pen=pg.mkPen((0, 200, 255), width=2))
            self._dot_xy = self._xy.plot([], [], pen=None, symbol="o", symbolBrush="r", symbolSize=10)
            self._dot_xz = self._xz.plot([], [], pen=None, symbol="o", symbolBrush="r", symbolSize=10)

            self._timer = QTimer()
            self._timer.timeout.connect(self._poll)
            self._timer.start(50)

        def _draw_bounds(self) -> None:
            dashed = Qt.DashLine
            hx, hy, hz = hard_limits["x"], hard_limits["y"], hard_limits["z"]
            sx, sy, sz = safe_bounds["x"], safe_bounds["y"], safe_bounds["z"]
            self._xy.plot([hx[0], hx[1], hx[1], hx[0], hx[0]], [hy[0], hy[0], hy[1], hy[1], hy[0]], pen=pg.mkPen("w"))
            self._xy.plot([sx[0], sx[1], sx[1], sx[0], sx[0]], [sy[0], sy[0], sy[1], sy[1], sy[0]], pen=pg.mkPen("y", style=dashed))
            self._xz.plot([hx[0], hx[1], hx[1], hx[0], hx[0]], [hz[0], hz[0], hz[1], hz[1], hz[0]], pen=pg.mkPen("w"))
            self._xz.plot([sx[0], sx[1], sx[1], sx[0], sx[0]], [sz[0], sz[0], sz[1], sz[1], sz[0]], pen=pg.mkPen("y", style=dashed))

        def keyPressEvent(self, event) -> None:
            if event.key() == Qt.Key_C:
                self._trail.clear()
                self._update_plots(None)
            else:
                super().keyPressEvent(event)

        def _poll(self) -> None:
            while True:
                try:
                    item = self._q.get_nowait()
                except Exception:
                    break
                if item is None:
                    self.close()
                    return
                self._update_plots(item)

        def _update_plots(self, item) -> None:
            if item is not None:
                x, y, z, r = item
                self._trail.append((x, y, z))
                self.statusBar().showMessage(f"X={x:.1f} Y={y:.1f} Z={z:.1f} R={r:.1f}")
            xs = [p[0] for p in self._trail]
            ys = [p[1] for p in self._trail]
            zs = [p[2] for p in self._trail]
            self._trail_xy.setData(xs, ys)
            self._trail_xz.setData(xs, zs)
            if xs:
                self._dot_xy.setData([xs[-1]], [ys[-1]])
                self._dot_xz.setData([xs[-1]], [zs[-1]])
            else:
                self._dot_xy.setData([], [])
                self._dot_xz.setData([], [])

    app = QApplication(sys.argv)
    window = _Window(queue)
    window.show()
    app.exec_()

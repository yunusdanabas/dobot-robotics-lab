"""Cross-platform terminal key reader for teleop scripts."""

from __future__ import annotations

import sys
import time


class TerminalKeyReader:
    """Context manager that provides non-blocking keyboard reads."""

    _WIN_EXTENDED = {
        0x48: "up",
        0x50: "down",
        0x4B: "left",
        0x4D: "right",
    }

    def __init__(self) -> None:
        self._fd: int | None = None
        self._old_term = None

    @staticmethod
    def require_tty() -> bool:
        """Return True when stdin is an interactive terminal."""
        return sys.stdin.isatty()

    def __enter__(self) -> "TerminalKeyReader":
        if sys.platform != "win32":
            import termios
            import tty

            self._fd = sys.stdin.fileno()
            self._old_term = termios.tcgetattr(self._fd)
            tty.setraw(self._fd)
        return self

    def __exit__(self, exc_type, exc, tb) -> None:
        self.restore()

    def restore(self) -> None:
        """Restore terminal state on POSIX platforms."""
        if sys.platform == "win32" or self._fd is None or self._old_term is None:
            return
        import termios

        termios.tcsetattr(self._fd, termios.TCSADRAIN, self._old_term)
        self._fd = None
        self._old_term = None

    def read_key(self, timeout_s: float = 0.02) -> str | None:
        """Return one normalized key token or None when no key is ready."""
        if sys.platform == "win32":
            return self._read_windows(timeout_s)
        return self._read_posix(timeout_s)

    def _read_windows(self, timeout_s: float) -> str | None:
        import msvcrt

        deadline = time.perf_counter() + timeout_s
        while time.perf_counter() < deadline:
            if msvcrt.kbhit():
                ch = msvcrt.getch()
                if ch in (b"\x00", b"\xe0"):
                    ch2 = msvcrt.getch()
                    return self._WIN_EXTENDED.get(ch2[0])
                if ch == b"\x1b":
                    return "esc"
                try:
                    text = ch.decode("utf-8", errors="replace")
                except Exception:
                    return None
                if text in "[]{}":
                    return text
                return text.lower() if text else None
            time.sleep(min(timeout_s, 0.005))
        return None

    def _read_posix(self, timeout_s: float) -> str | None:
        import select

        ready, _, _ = select.select([sys.stdin], [], [], timeout_s)
        if not ready:
            return None

        ch = sys.stdin.read(1)
        if not ch:
            return None

        if ch == "\x1b":
            r2, _, _ = select.select([sys.stdin], [], [], 0.05)
            if not r2:
                return "esc"
            c1 = sys.stdin.read(1)
            if c1 != "[":
                return "esc"
            r3, _, _ = select.select([sys.stdin], [], [], 0.02)
            if not r3:
                return "esc"
            c2 = sys.stdin.read(1)
            if c2 == "A":
                return "up"
            if c2 == "B":
                return "down"
            if c2 == "C":
                return "right"
            if c2 == "D":
                return "left"
            if c2 == "5":
                if select.select([sys.stdin], [], [], 0.02)[0]:
                    sys.stdin.read(1)
                return "page_up"
            if c2 == "6":
                if select.select([sys.stdin], [], [], 0.02)[0]:
                    sys.stdin.read(1)
                return "page_down"
            return "esc"

        if ch in "[]{}":
            return ch
        return ch.lower() if ch else None

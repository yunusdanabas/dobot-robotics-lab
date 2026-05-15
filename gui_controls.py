"""Shared Tk helpers for GUI-first teleop scripts.

Lightweight wrappers keep repetitive button/slider/status boilerplate in one
place and avoid pulling in any non-stdlib dependency.
"""

from __future__ import annotations

import tkinter as tk
from tkinter import ttk
from typing import Callable


def bind_hold_button(
    button: ttk.Button,
    *,
    on_press: Callable[[], None],
    on_release: Callable[[], None],
) -> None:
    """Bind press/release handlers so a button can drive hold-to-move actions."""
    button.bind("<ButtonPress-1>", lambda _event: on_press())
    button.bind("<ButtonRelease-1>", lambda _event: on_release())
    button.bind("<Leave>", lambda _event: on_release())


def labeled_value(parent: tk.Misc, label: str, width: int = 12) -> tuple[ttk.Frame, ttk.Label]:
    """Create a simple `Label: value` row and return row frame + value label."""
    row = ttk.Frame(parent)
    ttk.Label(row, text=label).pack(side="left")
    value = ttk.Label(row, text="-", width=width)
    value.pack(side="left", padx=(8, 0))
    return row, value


def schedule_periodic(root: tk.Tk, every_ms: int, callback: Callable[[], None]) -> None:
    """Run callback periodically until the Tk window closes."""

    def _tick() -> None:
        try:
            callback()
            root.after(every_ms, _tick)
        except tk.TclError:
            return

    root.after(every_ms, _tick)


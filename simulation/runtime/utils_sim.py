"""Shim: re-export Magician helpers for legacy ``import utils_sim`` callers."""

from __future__ import annotations

import os
import sys


_HERE = os.path.dirname(os.path.abspath(__file__))
_MAGICIAN_UTILS_DIR = os.path.normpath(os.path.join(_HERE, "..", "..", "robots", "magician"))
if os.path.exists(os.path.join(_MAGICIAN_UTILS_DIR, "utils.py")):
    if _MAGICIAN_UTILS_DIR not in sys.path:
        sys.path.insert(0, _MAGICIAN_UTILS_DIR)
else:
    raise ImportError(f"Magician utils.py not found at {_MAGICIAN_UTILS_DIR}")

from utils import *  # noqa: F401,F403,E402

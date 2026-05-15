"""Resolve simulation URDF asset paths.

The runtime and student teleops share these helpers so environment overrides
and default vendor locations stay consistent.
"""

from __future__ import annotations

import os
from pathlib import Path


PACKAGE_ROOT = Path(__file__).resolve().parents[2]
WORKSPACE_ROOT = PACKAGE_ROOT.parent

MAGICIAN_URDF_BY_TOOL = {
    "none": "magician_none.urdf",
    "motor": "magician_motor.urdf",
    "suction": "magician_suction.urdf",
}


def _override_path(env_keys: tuple[str, ...], urdf_name: str) -> Path | None:
    """Return the first existing env override, accepting file or folder paths."""
    for env_key in env_keys:
        raw = os.environ.get(env_key)
        if not raw:
            continue
        path = Path(raw).expanduser().resolve()
        if path.is_dir():
            path = path / "urdf" / urdf_name
        if path.exists():
            return path
    return None


def resolve_magician_urdf_path(tool: str = "none") -> Path:
    """Return the Magician URDF path for ``tool`` (none, motor, or suction)."""
    try:
        urdf_name = MAGICIAN_URDF_BY_TOOL[tool]
    except KeyError as exc:
        raise ValueError(
            f"Unknown Magician tool {tool!r}; expected one of {sorted(MAGICIAN_URDF_BY_TOOL)}"
        ) from exc

    override = _override_path(("DOBOT_MAGICIAN_URDF_PATH", "DOBOT_MAGICIAN_URDF"), urdf_name)
    if override is not None:
        return override

    candidates = [
        PACKAGE_ROOT / "vendor" / "magician_ros2_urdf" / "urdf" / urdf_name,
        WORKSPACE_ROOT / "vendor" / "magician_ros2_urdf" / "urdf" / urdf_name,
    ]
    for candidate in candidates:
        if candidate.exists():
            return candidate.resolve()

    expected = PACKAGE_ROOT / "vendor" / "magician_ros2_urdf"
    raise FileNotFoundError(
        "Magician URDF not found.\n"
        "Set up assets from ME403_LabFiles root:\n"
        "  python3 scripts/bootstrap.py --simulation\n"
        "or:\n"
        "  python3 scripts/fetch_assets.py --all\n\n"
        "If you use a modified URDF, point directly at one URDF file, or at a\n"
        "description folder containing urdf/magician_none.urdf,\n"
        "urdf/magician_motor.urdf, and urdf/magician_suction.urdf:\n"
        "  DOBOT_MAGICIAN_URDF_PATH=/absolute/path/to/magician_none.urdf\n"
        "  DOBOT_MAGICIAN_URDF_PATH=/absolute/path/to/magician_ros2_urdf\n\n"
        f"Expected default vendor location: {expected.resolve()}"
    )


def resolve_mg400_urdf_path() -> Path:
    """Return the MG400 URDF path."""
    override = _override_path(("DOBOT_MG400_URDF_PATH", "DOBOT_MG400_URDF"), "mg400.urdf")
    if override is not None:
        return override

    candidates = [
        PACKAGE_ROOT / "vendor" / "mg400_description" / "urdf" / "mg400.urdf",
        WORKSPACE_ROOT / "vendor" / "mg400_description" / "urdf" / "mg400.urdf",
    ]
    for candidate in candidates:
        if candidate.exists():
            return candidate.resolve()

    expected = PACKAGE_ROOT / "vendor" / "mg400_description"
    raise FileNotFoundError(
        "MG400 URDF not found.\n"
        "Set up assets from ME403_LabFiles root:\n"
        "  python3 scripts/bootstrap.py --simulation\n"
        "or:\n"
        "  python3 scripts/fetch_assets.py --all\n\n"
        "If you use a modified URDF, point directly with:\n"
        "  DOBOT_MG400_URDF_PATH=/absolute/path/to/mg400.urdf\n"
        "or a description folder:\n"
        "  DOBOT_MG400_URDF_PATH=/absolute/path/to/mg400_description\n\n"
        f"Expected default vendor location: {expected.resolve()}"
    )

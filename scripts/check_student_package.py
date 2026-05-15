"""Validate a public ME403_LabFiles student package tree."""

from __future__ import annotations

import argparse
import shutil
from pathlib import Path


DEFAULT_ROOT = Path(__file__).resolve().parents[1]

REQUIRED_PATHS = (
    "README.md",
    "GETTING_STARTED.md",
    ".gitignore",
    "gui_controls.py",
    "terminal_keys.py",
    "docs/README.md",
    "docs/setup.md",
    "docs/simulation.md",
    "docs/troubleshooting.md",
    "requirements/base.txt",
    "requirements/simulation_mujoco.txt",
    "requirements/simulation_pybullet.txt",
    "scripts/bootstrap.py",
    "scripts/fetch_assets.py",
    "scripts/check_install.py",
    "scripts/check_student_package.py",
    "labs/README.md",
    "labs/lab01_fk/README.md",
    "labs/lab01_fk/myCode.py",
    "labs/lab01_fk/interface.py",
    "labs/lab01_fk/utils.py",
    "labs/lab02_jacobian_ik/README.md",
    "labs/lab02_jacobian_ik/myCode.py",
    "labs/lab02_jacobian_ik/interface.py",
    "labs/lab02_jacobian_ik/utils.py",
    "robots/README.md",
    "robots/magician/README.md",
    "robots/mg400/README.md",
    "simulation/README.md",
    "simulation/runtime/sim_dobot.py",
    "simulation/runtime/sim_mg400.py",
    "simulation/student/README.md",
    "simulation/tests/README.md",
    "tools/README.md",
    "vendor/README.md",
)

GENERATED_DIR_NAMES = {"__pycache__", ".pytest_cache", ".venv", "build", "install", "log", "move"}
GENERATED_SUFFIXES = {
    ".pyc",
    ".pyo",
    ".aux",
    ".bbl",
    ".bcf",
    ".blg",
    ".fdb_latexmk",
    ".fls",
    ".lof",
    ".log",
    ".lot",
    ".nav",
    ".out",
    ".snm",
    ".synctex",
    ".gz",
    ".toc",
    ".vrb",
    ".xdv",
    ".zip",
}
TEXT_SUFFIXES = {".py", ".md", ".txt", ".ps1", ".tex", ".yml", ".yaml", ".toml"}
TEXT_FILENAMES = {".gitignore", "Makefile"}
INSTRUCTOR_DIR = "_" + "instructor"
BLANK_SUFFIX = "_" + "blank"
BANNED_TEXT = (
    INSTRUCTOR_DIR,
    "Students/" + "_shared",
    "import " + "unified_lab_utils",
    "from " + "interface_common",
    "Solution" + " Key",
    "reference " + "solution",
    "solution " + "key with full " + "implementation",
    "full " + "implementation",
    "lab01_fk" + BLANK_SUFFIX,
    "lab02_jacobian_ik" + BLANK_SUFFIX,
)


def _clean_generated(root: Path) -> int:
    removed = 0
    for path in sorted(root.rglob("*"), key=lambda p: len(p.parts), reverse=True):
        rel = path.relative_to(root)
        if rel.parts[:1] == (".git",):
            continue
        if path.is_dir() and path.name in {"__pycache__", ".pytest_cache"}:
            shutil.rmtree(path)
            removed += 1
        elif path.is_file() and path.suffix in {".pyc", ".pyo"}:
            path.unlink()
            removed += 1
    return removed


def _should_skip_text_scan(root: Path, path: Path, *, allow_private: bool) -> bool:
    rel = path.relative_to(root)
    if path.resolve() == Path(__file__).resolve():
        return True
    if rel.parts[:1] == ("vendor",):
        return True
    if allow_private and rel.parts[:1] == (INSTRUCTOR_DIR,):
        return True
    return False


def _check_text(root: Path, path: Path, errors: list[str], *, allow_private: bool) -> None:
    is_text_file = path.suffix in TEXT_SUFFIXES or path.name in TEXT_FILENAMES or path.suffix == ""
    if not is_text_file or _should_skip_text_scan(root, path, allow_private=allow_private):
        return
    try:
        text = path.read_text(encoding="utf-8")
    except UnicodeDecodeError:
        return
    for token in BANNED_TEXT:
        if token in text:
            errors.append(f"banned text {token!r}: {path.relative_to(root)}")


def _is_allowed_private_path(rel: Path) -> bool:
    return rel.parts[:1] == (INSTRUCTOR_DIR,)


def validate(root: Path, *, allow_private: bool) -> list[str]:
    errors: list[str] = []
    for rel in REQUIRED_PATHS:
        if not (root / rel).exists():
            errors.append(f"missing required path: {rel}")

    readme = root / "README.md"
    if readme.exists() and "## Acknowledgements" not in readme.read_text(encoding="utf-8"):
        errors.append("missing README acknowledgements section")

    for path in root.rglob("*"):
        rel = path.relative_to(root)
        if rel.parts[:1] == (".git",):
            continue

        if ".git" in rel.parts:
            errors.append(f"nested git metadata: {rel}")
        if any(part in GENERATED_DIR_NAMES for part in rel.parts):
            errors.append(f"generated/cache path: {rel}")
        if path.is_file() and (path.suffix in GENERATED_SUFFIXES or path.name.endswith(".run.xml")):
            errors.append(f"generated file: {rel}")
        if any(part.endswith(BLANK_SUFFIX) for part in rel.parts):
            errors.append(f"blank-suffixed path leaked into public tree: {rel}")
        if rel.parts[:1] == (INSTRUCTOR_DIR,):
            if not allow_private:
                errors.append(f"instructor-only path in public tree: {rel}")
            elif not _is_allowed_private_path(rel):
                errors.append(f"unexpected instructor path: {rel}")
        elif rel.parts[:1] != ("vendor",) and any("solution" in part.lower() for part in rel.parts):
            errors.append(f"solution-named path outside instructor area: {rel}")

        if path.is_file():
            _check_text(root, path, errors, allow_private=allow_private)

    return errors


def main() -> int:
    parser = argparse.ArgumentParser(description="Check that a ME403_LabFiles tree is safe for public student release.")
    parser.add_argument("root", nargs="?", default=str(DEFAULT_ROOT), help="Package/export root to check")
    parser.add_argument(
        "--allow-instructor",
        dest="allow_private",
        action="store_true",
        help="Allow private maintainer files in this source tree",
    )
    parser.add_argument("--clean", action="store_true", help="Remove Python cache files before checking")
    parser.add_argument("--clean-only", action="store_true", help="Remove Python cache files and exit without checking")
    args = parser.parse_args()

    root = Path(args.root).expanduser().resolve()
    if args.clean or args.clean_only:
        removed = _clean_generated(root)
        print(f"Removed {removed} generated cache path(s).")
        if args.clean_only:
            return 0

    errors = validate(root, allow_private=args.allow_private)
    if errors:
        print("Student package check FAILED:\n")
        for error in errors:
            print(f"- {error}")
        return 1

    print("Student package check OK.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

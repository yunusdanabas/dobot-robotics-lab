"""Shared waypoint checks and pick/place stepping for sim_pick_place and viz scripts."""

from __future__ import annotations

import time
from typing import Callable, Iterable, Mapping


def check_bounds(
    *,
    bounds: Mapping[str, tuple[float, float]],
    label: str,
    x: float,
    y: float,
    z: float,
) -> None:
    issues = []
    if not (bounds["x"][0] <= x <= bounds["x"][1]):
        issues.append(f"X={x} outside {bounds['x']}")
    if not (bounds["y"][0] <= y <= bounds["y"][1]):
        issues.append(f"Y={y} outside {bounds['y']}")
    if not (bounds["z"][0] <= z <= bounds["z"][1]):
        issues.append(f"Z={z} outside {bounds['z']}")
    if issues:
        print(f"[Warning] {label}: {', '.join(issues)}")


def check_waypoints(
    *,
    bounds: Mapping[str, tuple[float, float]],
    points: Iterable[tuple[str, float, float, float]],
) -> None:
    for label, x, y, z in points:
        check_bounds(bounds=bounds, label=label, x=x, y=y, z=z)


def run_single_pick_place(
    *,
    go_home: Callable[[], None],
    pick_up: Callable[[], None],
    place_down: Callable[[], None],
    home_message: str = "Going home ...",
    return_message: str = "Returning home ...",
    done_message: str = "Pick-and-place simulation complete.",
) -> None:
    print(home_message)
    go_home()

    print("\n--- PICK ---")
    pick_up()

    print("\n--- PLACE ---")
    place_down()

    print(f"\n{return_message}")
    go_home()

    print(f"\n{done_message}")


def run_visual_pick_place_cycles(
    *,
    cycles: int,
    is_running: Callable[[], bool],
    go_home: Callable[[], None],
    pick_up: Callable[[], None],
    place_down: Callable[[], None],
) -> int:
    completed = 0
    print("Going home ...")
    go_home()

    while True:
        if cycles > 0 and completed >= cycles:
            break
        if not is_running():
            print("Viewer closed - exiting.")
            break

        completed += 1
        print(f"\n=== Cycle {completed} ===")

        print("--- PICK ---")
        pick_up()
        if not is_running():
            break

        print("--- PLACE ---")
        place_down()
        if not is_running():
            break

        print("Returning home ...")
        go_home()

        if cycles == 0:
            print("(Close the viewer window to stop, or press Ctrl-C)")
            time.sleep(0.5)

    return completed

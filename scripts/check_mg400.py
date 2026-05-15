"""Basic MG400 network connectivity check."""

from __future__ import annotations

import argparse
import socket
import subprocess
import sys


ROBOT_IPS = {
    1: "192.168.2.7",
    2: "192.168.2.10",
    3: "192.168.2.9",
    4: "192.168.2.6",
}
PORTS = (29999, 30003, 30004)


def _ping_command(ip: str) -> list[str]:
    if sys.platform == "win32":
        return ["ping", "-n", "2", "-w", "1000", ip]
    return ["ping", "-c", "2", "-W", "1", ip]


def _ping(ip: str) -> bool:
    try:
        result = subprocess.run(_ping_command(ip), stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, check=False)
        return result.returncode == 0
    except Exception:
        return False


def _tcp(ip: str, port: int) -> bool:
    try:
        with socket.create_connection((ip, port), timeout=1.5):
            return True
    except OSError:
        return False


def main() -> int:
    parser = argparse.ArgumentParser(description="Check MG400 ping and TCP ports.")
    parser.add_argument("--robot", type=int, choices=ROBOT_IPS, help="Robot number 1-4")
    parser.add_argument("--ip", help="Explicit IP address")
    args = parser.parse_args()

    ip = args.ip or ROBOT_IPS.get(args.robot or 1)
    print(f"Checking MG400 at {ip}")
    ping_ok = _ping(ip)
    print(f"ping                 {'OK' if ping_ok else 'FAILED'}")
    all_ok = ping_ok
    for port in PORTS:
        ok = _tcp(ip, port)
        print(f"tcp/{port:<5}           {'OK' if ok else 'FAILED'}")
        all_ok &= ok

    if not all_ok:
        print("\nChecklist:")
        print("1. Robot powered on and Ethernet connected.")
        print("2. PC Ethernet set to 192.168.2.100/24.")
        print("3. Correct robot number/IP selected.")
    return 0 if all_ok else 1


if __name__ == "__main__":
    raise SystemExit(main())

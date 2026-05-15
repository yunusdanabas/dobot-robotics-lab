"""
00_raw_dashboard_probe.py — Raw dashboard socket lines for debugging MG400 framing/protocol (no motion).

Examples: python 00_raw_dashboard_probe.py --robot 4
          python 00_raw_dashboard_probe.py --ip 192.168.2.6 --timeout 2
"""

from __future__ import annotations

import argparse
import socket
import time

from utils_mg400 import DASHBOARD_PORT, MG400_IP, ROBOT_IPS


COMMANDS = ("RobotMode", "GetErrorID", "GetPose", "GetAngle", "GetVersion")
VARIANTS = (
    ("plain",    "{cmd}()"),
    ("semi",     "{cmd}();"),
    ("newline",  "{cmd}()\n"),
    ("semi+nl",  "{cmd}();\n"),
)


def _resolve_ip(robot: int | None, ip: str) -> str:
    return ROBOT_IPS[robot] if robot else ip


def _escape_text(text: str) -> str:
    return (
        text.replace("\\", "\\\\")
        .replace("\r", "\\r")
        .replace("\n", "\\n")
        .replace("\t", "\\t")
    )


def _clip(text: str, limit: int = 180) -> str:
    return text if len(text) <= limit else text[: limit - 1] + "…"


def _probe_once(ip: str, payload: str, timeout: float) -> tuple[bytes, str]:
    with socket.create_connection((ip, DASHBOARD_PORT), timeout=timeout) as sock:
        sock.settimeout(timeout)
        sock.sendall(payload.encode("utf-8"))
        data = sock.recv(4096)
    return data, data.decode("utf-8", errors="replace")


def _probe_with_retries(
    ip: str,
    payload: str,
    timeout: float,
    retries: int,
    retry_delay: float,
) -> tuple[bytes | None, str | None, Exception | None]:
    last_exc: Exception | None = None
    for _ in range(retries + 1):
        try:
            raw, text = _probe_once(ip, payload, timeout)
            return raw, text, None
        except Exception as exc:  # noqa: BLE001 - diagnostic script
            last_exc = exc
            time.sleep(retry_delay)
    return None, None, last_exc


def run_probe(
    ip: str,
    timeout: float,
    retries: int,
    retry_delay: float,
    post_delay: float,
) -> None:
    print(f"\n=== Probe target: {ip}:{DASHBOARD_PORT} ===")
    for cmd in COMMANDS:
        print(f"\n[{cmd}]")
        for variant_name, template in VARIANTS:
            payload = template.format(cmd=cmd)
            raw, text, exc = _probe_with_retries(
                ip=ip,
                payload=payload,
                timeout=timeout,
                retries=retries,
                retry_delay=retry_delay,
            )
            send_repr = _escape_text(payload)
            if exc is not None:
                print(
                    f"  {variant_name:8s} send={send_repr!r} "
                    f"-> ERROR {type(exc).__name__}: {exc}"
                )
            else:
                recv_text = _clip(_escape_text(text or ""))
                recv_bytes = _clip(repr(raw))
                print(
                    f"  {variant_name:8s} send={send_repr!r} "
                    f"-> recv_text={recv_text!r} recv_bytes={recv_bytes}"
                )
            time.sleep(post_delay)


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Raw MG400 dashboard probe (no motion commands)."
    )
    parser.add_argument(
        "--ip",
        default=MG400_IP,
        help="Primary target IP (default: robot 1). Overridden by --robot.",
    )
    parser.add_argument(
        "--robot",
        type=int,
        choices=list(ROBOT_IPS.keys()),
        metavar="N",
        help="Primary target robot number 1-4.",
    )
    parser.add_argument(
        "--compare-ip",
        default=None,
        help="Optional second target IP for side-by-side comparison.",
    )
    parser.add_argument(
        "--compare-robot",
        type=int,
        choices=list(ROBOT_IPS.keys()),
        metavar="N",
        help="Optional second target robot number 1-4 (overrides --compare-ip).",
    )
    parser.add_argument(
        "--timeout",
        type=float,
        default=2.0,
        help="Socket connect/read timeout in seconds (default: 2.0).",
    )
    parser.add_argument(
        "--retries",
        type=int,
        default=5,
        help="Retries per payload on connection/read errors (default: 5).",
    )
    parser.add_argument(
        "--retry-delay",
        type=float,
        default=0.4,
        help="Delay between retries in seconds (default: 0.4).",
    )
    parser.add_argument(
        "--post-delay",
        type=float,
        default=0.2,
        help="Delay between payloads in seconds (default: 0.2).",
    )
    args = parser.parse_args()

    primary_ip = _resolve_ip(args.robot, args.ip)
    compare_ip = None
    if args.compare_robot is not None or args.compare_ip:
        compare_ip = _resolve_ip(args.compare_robot, args.compare_ip or MG400_IP)

    print("MG400 raw dashboard probe (diagnostic, no motion)")
    print(f"Commands: {', '.join(COMMANDS)}")
    print("Variants: plain, semicolon, newline, semicolon+newline")
    print(f"Primary target: {primary_ip}")
    if compare_ip:
        print(f"Compare target: {compare_ip}")

    run_probe(
        ip=primary_ip,
        timeout=args.timeout,
        retries=args.retries,
        retry_delay=args.retry_delay,
        post_delay=args.post_delay,
    )
    if compare_ip:
        run_probe(
            ip=compare_ip,
            timeout=args.timeout,
            retries=args.retries,
            retry_delay=args.retry_delay,
            post_delay=args.post_delay,
        )


if __name__ == "__main__":
    main()

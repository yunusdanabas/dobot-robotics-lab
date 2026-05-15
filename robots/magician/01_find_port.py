"""
01_find_port.py — List serial ports and mark likely Dobot devices. 

Run: python 01_find_port.py
"""

from serial.tools import list_ports
from utils import find_port, DOBOT_KEYWORDS


def main():
    ports = list(list_ports.comports())

    if not ports:
        print("No serial ports found. Is the robot plugged in?")
        return

    print(f"{'Device':<20} {'Description':<40} {'HWID'}")
    print("-" * 90)

    def desc_hwid(port) -> str:
        return f"{(port.description or '')} {port.hwid}".lower()

    for p in ports:
        is_dobot = any(kw.lower() in desc_hwid(p) for kw in DOBOT_KEYWORDS)
        marker = "  <-- DOBOT" if is_dobot else ""
        print(f"{p.device:<20} {(p.description or 'n/a'):<40} {p.hwid}{marker}")

    print()
    likely = find_port()
    if likely and any(p.device == likely for p in ports):
        print(f"[OK] Likely Dobot port: {likely}")
        print(f"     Use PORT = '{likely}' in your connection scripts.")
    else:
        print("[!] No Dobot device found (CP210x/Silicon Labs or CH340/USB2.0-Serial).")
        print("    Check USB cable, power adapter, and driver installation.")
        print("    On Linux: sudo usermod -a -G dialout $USER  (then re-login)")
        print("    On Windows: check Device Manager for a Silicon Labs / USB Serial COM port.")


if __name__ == "__main__":
    main()

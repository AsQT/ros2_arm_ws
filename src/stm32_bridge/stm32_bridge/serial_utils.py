# serial_utils.py
import sys
from serial.tools import list_ports

def choose_serial_port(prompt: str = "Chọn cổng serial", default: str | None = None) -> str:

    ports = list(list_ports.comports())

    if not ports:
        print("Không tìm thấy cổng serial nào.")
        sys.exit(1)

    print(f"\n{prompt}:")
    for i, p in enumerate(ports):
        print(f"  [{i}] {p.device}  ({p.description})")

    if default and any(p.device == default for p in ports):
        print(f"\nNhấn Enter để dùng mặc định: {default}, hoặc nhập số index khác.")
    else:
        default = None

    while True:
        s = input("Chọn index cổng: ").strip()
        if s == "" and default:
            return default

        if not s.isdigit():
            print("Vui lòng nhập số index.")
            continue

        idx = int(s)
        if 0 <= idx < len(ports):
            return ports[idx].device

        print("Index không hợp lệ, thử lại.")

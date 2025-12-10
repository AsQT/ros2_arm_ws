#!/usr/bin/env python3
import serial
import struct
from serial_utils import choose_serial_port

HEADER = b"\xAA\xCC"
TAIL   = b"\xAA\xEE"
FRAME_LEN = 31  # 2 header + 25 payload + 2 crc + 2 tail

def crc16_modbus(data: bytes) -> int:
    crc = 0xFFFF
    for b in data:
        crc ^= b
        for _ in range(8):
            if crc & 1:
                crc = (crc >> 1) ^ 0xA001
            else:
                crc >>= 1
    return crc & 0xFFFF

def bytes_to_hex(bs: bytes) -> str:
    return " ".join(f"{b:02X}" for b in bs)

def main():
    port = choose_serial_port("Chọn cổng serial để MONITOR")
    baud = 115200

    print(f"\nMở cổng {port} @ {baud}...")
    ser = serial.Serial(port, baud, timeout=0.05)

    buf = bytearray()
    print("Đang đọc dữ liệu... Nhấn Ctrl+C để thoát.\n")

    try:
        while True:
            data = ser.read(64)
            if data:
                buf.extend(data)

            # Tìm frame trong buffer
            while True:
                start = buf.find(HEADER)
                if start == -1:
                    # Không có header → chỉ giữ vài byte cuối để tránh phình bộ nhớ
                    if len(buf) > 4:
                        buf = buf[-4:]
                    break

                # Nếu không đủ tối thiểu 1 frame thì chờ thêm
                if len(buf) < start + FRAME_LEN:
                    break

                # Kiểm tra tail theo vị trí cố định
                end = start + FRAME_LEN
                frame = buf[start:end]

                if frame[-2:] != TAIL:
                    # header sai/ lệch → bỏ byte đầu và tìm lại
                    buf.pop(0)
                    continue

                # Lúc này frame có độ dài đúng và có tail
                # Tách phần payload + CRC để kiểm CRC
                payload = frame[2:2+25]           # CMD + 6*int32
                crc_rx = frame[27] | (frame[28] << 8)
                crc_calc = crc16_modbus(payload)

                print("==================================================")
                print("RAW FRAME HEX:")
                print(bytes_to_hex(frame))

                if crc_rx != crc_calc:
                    print(f"CRC ERROR: rx=0x{crc_rx:04X}, calc=0x{crc_calc:04X}")
                else:
                    print(f"CRC OK: 0x{crc_calc:04X}")
                    cmd = payload[0]
                    joints_raw = []
                    joints_deg = []

                    for i in range(6):
                        offset = 1 + i * 4
                        val = struct.unpack("<i", payload[offset:offset+4])[0]
                        joints_raw.append(val)
                        joints_deg.append(val / 1000.0)

                    print(f"CMD       : 0x{cmd:02X}")
                    print("Joints int32:", joints_raw)
                    print("Joints deg  :", ["{:.3f}".format(d) for d in joints_deg])

                # Bỏ frame này khỏi buffer
                del buf[:end]
    except KeyboardInterrupt:
        print("\nThoát.")
    finally:
        ser.close()

if __name__ == "__main__":
    main()

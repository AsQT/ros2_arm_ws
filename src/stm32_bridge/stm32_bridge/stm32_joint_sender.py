#!/usr/bin/env python3
import rclpy
from rclpy.node         import Node

from sensor_msgs.msg    import JointState

import serial
import struct
import threading
import math
from serial.tools       import list_ports


# ================= CONFIG =================
DEFAULT_PORT = "/dev/ttyUSB0"
DEFAULT_BAUD = 115200

JOINT_NAMES = [
    "joint_1",
    "joint_2",
    "joint_3",
    "joint_4",
    "joint_5",
    "joint_6", ]

DEG_SCALE = 1000  # 1 deg = 1000

# FAS-like protocol
HEADER = b"\xAA\xCC"
TAIL   = b"\xAA\xEE"
CMD_JOINT6 = 0x10 


# ================= CRC16 (FAS / MODBUS STYLE) =================

def crc16_modbus(data: bytes) -> int:
    """
    CRC16-IBM / MODBUS: poly 0xA001, init 0xFFFF
    """
    crc = 0xFFFF
    for b in data:
        crc ^= b
        for _ in range(8):
            if crc & 0x0001:
                crc = (crc >> 1) ^ 0xA001
            else:
                crc >>= 1
    return crc & 0xFFFF


# ================= CHỌN CỔNG COM =================

def choose_serial_port(prompt: str = "Chọn cổng serial", default: str | None = None) -> str:

    ports = list(list_ports.comports())
    if not ports:
        print("Không tìm thấy cổng serial nào.")
        raise SystemExit(1)

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


# ================= NODE =================

class STM32Bridge(Node):

    def __init__(self):
        super().__init__("stm32_joint_sender")

        self.declare_parameter("port", "")
        self.declare_parameter("baud", DEFAULT_BAUD)
        self.declare_parameter("joint_topic", "/joint_states")

        port_param  = self.get_parameter("port").get_parameter_value().string_value
        baud_param  = self.get_parameter("baud").get_parameter_value().integer_value
        topic_param = self.get_parameter("joint_topic").get_parameter_value().string_value

        if port_param:
            port = port_param
            print(f"Dùng cổng từ ROS param: {port}")
        else:
            port = choose_serial_port("Chọn cổng serial cho STM32Bridge", default=DEFAULT_PORT)

        baud = baud_param if baud_param > 0 else DEFAULT_BAUD
        self.joint_topic = topic_param or "/joint_states"

        try:
            self.ser = serial.Serial(port, baud, timeout=0.01)
            self.get_logger().info(f"Open serial: {port} @ {baud}")
        except Exception as e:
            self.get_logger().fatal(f"Cannot open {port}: {e}")
            raise SystemExit()

        self.lock = threading.Lock()

        self.last_joints = {name: 0.0 for name in JOINT_NAMES}

        self.sub = self.create_subscription(
            JointState,
            self.joint_topic,
            self.joint_state_cb,
            10
        )
        self.get_logger().info(f"Subscribed to {self.joint_topic}")

    def joint_state_cb(self, msg: JointState):
        updated = False

        for name, pos in zip(msg.name, msg.position):
            if name in JOINT_NAMES:
                deg = math.degrees(pos)
                self.last_joints[name] = deg
                updated = True

        if updated:
            self.send_joints(self.last_joints)

    # -------------------------------------------------
    # Build frame: FAS-style header / tail / CRC16
    # -------------------------------------------------
    def build_frame(self, joint_dict: dict) -> bytes:

        # ---- PAYLOAD: CMD + 6 * int32 ----
        payload = bytearray()
        payload.append(CMD_JOINT6)

        for name in JOINT_NAMES:
            deg = float(joint_dict.get(name, 0.0))
            val = int(deg * DEG_SCALE)  # int32

            # clamp int32
            if val > 0x7FFFFFFF:
                val = 0x7FFFFFFF
            elif val < -0x80000000:
                val = -0x80000000

            payload.extend(struct.pack("<i", val))

        # ---- CRC16 trên PAYLOAD ----
        crc = crc16_modbus(bytes(payload))
        crc_bytes = struct.pack("<H", crc)

        # ---- FULL FRAME: HEADER + PAYLOAD + CRC + TAIL ----
        frame = bytearray()
        frame.extend(HEADER)
        frame.extend(payload)
        frame.extend(crc_bytes)
        frame.extend(TAIL)

        return bytes(frame)

    # -------------------------------------------------
    # Gửi 6 joint xuống STM32
    # -------------------------------------------------
    def send_joints(self, joint_dict: dict):
        frame = self.build_frame(joint_dict)

        with self.lock:
            self.ser.write(frame)

        log_str = " | ".join(
            f"{n}:{joint_dict.get(n, 0.0):.3f}deg"
            for n in JOINT_NAMES
        )
        self.get_logger().info(f"TX -> STM32 ({len(frame)} bytes): {log_str}")

    # -------------------------------------------------
    # DEMO: gửi sóng sin 6 khớp để test
    # -------------------------------------------------
    def run_demo_sender(self):
        t = 0.0
        dt = 0.02
        while rclpy.ok():
            joints = {}
            for i, n in enumerate(JOINT_NAMES):
                joints[n] = 30.0 * math.sin(t + i * 0.5)

            self.send_joints(joints)

            t += dt
            self.get_clock().sleep_for(rclpy.duration.Duration(seconds=dt))


# ================= MAIN =================

def main():
    rclpy.init()
    node = STM32Bridge()

    node.get_logger().info("STM32Bridge is running (listening to /joint_states)...")
    try:
        rclpy.spin(node)
        # node.run_demo_sender()
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

#!/usr/bin/env python3
import sys
import math
import rclpy
from rclpy.node import Node

from std_msgs.msg import Float64MultiArray
from std_srvs.srv import Trigger, SetBool

from PyQt5.QtWidgets import (
    QApplication, QWidget, QVBoxLayout, QHBoxLayout,
    QLabel, QPushButton, QSlider, QTextEdit
)
from PyQt5.QtCore import Qt, QTimer


# 6 joint + gripper
JOINT_NAMES = ["joint_1","joint_2","joint_3","joint_4","joint_5","joint_6","gripper"]

def deg2rad(d): return d * math.pi / 180.0


class FakeGuiNode(Node):
    def __init__(self):
        super().__init__("fake_gui_node")

        self.pub_pos = self.create_publisher(Float64MultiArray, "/robot_hardware/cmd_pos", 10)
        self.pub_vel = self.create_publisher(Float64MultiArray, "/robot_hardware/cmd_vel", 10)

        self.cli_home  = self.create_client(Trigger, "/robot_hardware/home")
        self.cli_stop  = self.create_client(Trigger, "/robot_hardware/stop")
        self.cli_estop = self.create_client(SetBool, "/robot_hardware/estop")

        # GUI làm việc theo độ (deg)
        self.cmd_pos_deg = [0.0] * len(JOINT_NAMES)
        self.cmd_vel_deg = [0.0] * len(JOINT_NAMES)


class FakeGui(QWidget):
    def __init__(self, ros_node: FakeGuiNode):
        super().__init__()
        self.node = ros_node
        self.setWindowTitle("Fake Robot Control GUI (deg) + Gripper")

        layout = QVBoxLayout()

        self.sliders = []
        self.value_labels = []

        for i, name in enumerate(JOINT_NAMES):
            row = QHBoxLayout()
            label = QLabel(name)

            val_label = QLabel("0°")
            val_label.setFixedWidth(50)

            slider = QSlider(Qt.Horizontal)

            # gripper thường không cần -180..180, ví dụ 0..100 (%)
            if name == "gripper":
                slider.setMinimum(0)
                slider.setMaximum(100)   # %
                slider.setValue(0)
            else:
                slider.setMinimum(-180)
                slider.setMaximum(180)
                slider.setValue(0)

            slider.valueChanged.connect(lambda v, idx=i: self.on_slider(idx, v))

            self.sliders.append(slider)
            self.value_labels.append(val_label)

            row.addWidget(label)
            row.addWidget(slider)
            row.addWidget(val_label)
            layout.addLayout(row)

        # Buttons
        row_btn = QHBoxLayout()
        btn_pos = QPushButton("Send Position")
        btn_vel = QPushButton("Send Velocity")
        btn_rst = QPushButton("RST = 0")

        btn_pos.clicked.connect(self.send_pos)
        btn_vel.clicked.connect(self.send_vel)
        btn_rst.clicked.connect(self.reset_all)

        row_btn.addWidget(btn_pos)
        row_btn.addWidget(btn_vel)
        row_btn.addWidget(btn_rst)
        layout.addLayout(row_btn)

        self.log = QTextEdit()
        self.log.setReadOnly(True)
        layout.addWidget(self.log)

        row_srv = QHBoxLayout()
        btn_home  = QPushButton("HOME")
        btn_stop  = QPushButton("STOP")
        btn_estop = QPushButton("ESTOP")

        btn_home.clicked.connect(self.call_home)
        btn_stop.clicked.connect(self.call_stop)
        btn_estop.clicked.connect(self.call_estop)

        row_srv.addWidget(btn_home)
        row_srv.addWidget(btn_stop)
        row_srv.addWidget(btn_estop)
        layout.addLayout(row_srv)

        self.setLayout(layout)

    def on_slider(self, idx, value):
        name = JOINT_NAMES[idx]

        if name == "gripper":
            # value là % (0..100). Bạn có thể map sang deg nếu gripper là revolute.
            self.node.cmd_pos_deg[idx] = float(value)
            self.node.cmd_vel_deg[idx] = 0.0
            self.value_labels[idx].setText(f"{value}%")
        else:
            self.node.cmd_pos_deg[idx] = float(value)
            self.node.cmd_vel_deg[idx] = float(value) * 0.5  # ví dụ vel = 0.5 * pos (deg/s)
            self.value_labels[idx].setText(f"{value}°")

    def reset_all(self):
        # set slider về 0 (sẽ auto update cmd_pos_deg/cmd_vel_deg)
        for s in self.sliders:
            s.setValue(0)
        self.log.append("RST: all = 0")

    def send_pos(self):
        # Publish rad cho 6 joint, gripper publish theo % (tạm thời)
        data = []
        for i, name in enumerate(JOINT_NAMES):
            if name == "gripper":
                data.append(self.node.cmd_pos_deg[i])   # % (tạm)
            else:
                data.append(deg2rad(self.node.cmd_pos_deg[i]))  # rad

        msg = Float64MultiArray()
        msg.data = data
        self.node.pub_pos.publish(msg)
        self.log.append(f"CMD_POS (deg): {self.node.cmd_pos_deg}")
        self.log.append(f"CMD_POS (pub): {msg.data}")

    def send_vel(self):
        data = []
        for i, name in enumerate(JOINT_NAMES):
            if name == "gripper":
                data.append(0.0)
            else:
                data.append(deg2rad(self.node.cmd_vel_deg[i]))  # rad/s

        msg = Float64MultiArray()
        msg.data = data
        self.node.pub_vel.publish(msg)
        self.log.append(f"CMD_VEL (deg/s): {self.node.cmd_vel_deg}")
        self.log.append(f"CMD_VEL (pub): {msg.data}")

    def call_home(self):
        self.log.append("CALL HOME")
        self.node.cli_home.call_async(Trigger.Request())

    def call_stop(self):
        self.log.append("CALL STOP")
        self.node.cli_stop.call_async(Trigger.Request())

    def call_estop(self):
        self.log.append("CALL ESTOP = true")
        req = SetBool.Request()
        req.data = True
        self.node.cli_estop.call_async(req)


def main():
    rclpy.init()
    node = FakeGuiNode()

    app = QApplication(sys.argv)
    gui = FakeGui(node)
    gui.show()

    timer = QTimer()
    timer.timeout.connect(lambda: rclpy.spin_once(node, timeout_sec=0.01))
    timer.start(10)

    sys.exit(app.exec_())


if __name__ == "__main__":
    main()

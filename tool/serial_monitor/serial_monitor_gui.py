#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import sys
import struct
import time
from dataclasses import dataclass
from typing import Optional, List

import serial
from serial.tools import list_ports

from PyQt5.QtCore import Qt, QThread, pyqtSignal
from PyQt5.QtWidgets import (
    QApplication, QWidget, QVBoxLayout, QHBoxLayout, QGroupBox,
    QLabel, QPushButton, QComboBox, QSpinBox, QDoubleSpinBox,
    QTextEdit, QTableWidget, QTableWidgetItem, QHeaderView, QMessageBox
)

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


@dataclass
class ParsedFrame:
    cmd: int
    joints_raw: List[int]
    joints_deg: List[float]
    crc_rx: int
    crc_calc: int
    raw_hex: str
    ts: float


def try_extract_one_frame(buf: bytearray) -> Optional[bytes]:
    """Extract exactly one full frame from buf if available; otherwise None.
    Removes leading junk safely.
    """
    while True:
        start = buf.find(HEADER)
        if start == -1:
            # keep last few bytes to avoid unlimited growth
            if len(buf) > 4:
                del buf[:-4]
            return None

        # drop bytes before header
        if start > 0:
            del buf[:start]

        if len(buf) < FRAME_LEN:
            return None

        frame = bytes(buf[:FRAME_LEN])
        if frame[-2:] != TAIL:
            # header was a false positive or stream misaligned → drop 1 byte and retry
            del buf[0]
            continue

        # valid length + tail
        del buf[:FRAME_LEN]
        return frame


def parse_frame(frame: bytes) -> ParsedFrame:
    payload = frame[2:2+25]  # 25 bytes
    crc_rx = frame[27] | (frame[28] << 8)
    crc_calc = crc16_modbus(payload)

    cmd = payload[0]
    joints_raw = []
    joints_deg = []
    for i in range(6):
        offset = 1 + i * 4
        val = struct.unpack("<i", payload[offset:offset+4])[0]
        joints_raw.append(val)
        joints_deg.append(val / 1000.0)

    return ParsedFrame(
        cmd=cmd,
        joints_raw=joints_raw,
        joints_deg=joints_deg,
        crc_rx=crc_rx,
        crc_calc=crc_calc,
        raw_hex=bytes_to_hex(frame),
        ts=time.time()
    )


class SerialReaderThread(QThread):
    frame_ok = pyqtSignal(object)     # ParsedFrame
    frame_bad = pyqtSignal(str)       # error string / raw
    status = pyqtSignal(str)          # status log line

    def __init__(self, port: str, baud: int, read_chunk: int = 64, timeout_s: float = 0.05):
        super().__init__()
        self.port = port
        self.baud = baud
        self.read_chunk = read_chunk
        self.timeout_s = timeout_s
        self._stop = False
        self._ser: Optional[serial.Serial] = None
        self._buf = bytearray()

        self.frames_ok_count = 0
        self.frames_crc_err_count = 0

    def stop(self):
        self._stop = True

    def run(self):
        try:
            self.status.emit(f"Opening {self.port} @ {self.baud} ...")
            self._ser = serial.Serial(self.port, self.baud, timeout=self.timeout_s)
            self.status.emit("Serial opened.")
        except Exception as e:
            self.status.emit(f"Open failed: {e}")
            return

        try:
            while not self._stop:
                try:
                    data = self._ser.read(self.read_chunk)
                except Exception as e:
                    self.status.emit(f"Read error: {e}")
                    break

                if data:
                    self._buf.extend(data)

                # extract as many frames as possible
                while True:
                    frame = try_extract_one_frame(self._buf)
                    if frame is None:
                        break

                    pf = parse_frame(frame)
                    if pf.crc_rx != pf.crc_calc:
                        self.frames_crc_err_count += 1
                        self.frame_bad.emit(
                            f"CRC ERROR rx=0x{pf.crc_rx:04X} calc=0x{pf.crc_calc:04X} | RAW: {pf.raw_hex}"
                        )
                    else:
                        self.frames_ok_count += 1
                        self.frame_ok.emit(pf)

        finally:
            try:
                if self._ser and self._ser.is_open:
                    self._ser.close()
            except Exception:
                pass
            self.status.emit("Serial closed.")


class MonitorGUI(QWidget):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Serial Frame Monitor (AA CC ... AA EE) - 6 Joints")

        self.reader: Optional[SerialReaderThread] = None

        # --- Top: connection controls
        self.cb_port = QComboBox()
        self.btn_refresh = QPushButton("Refresh Ports")
        self.sb_baud = QSpinBox()
        self.sb_baud.setRange(1200, 3000000)
        self.sb_baud.setValue(115200)

        self.sb_read_chunk = QSpinBox()
        self.sb_read_chunk.setRange(1, 4096)
        self.sb_read_chunk.setValue(64)

        self.sb_timeout = QDoubleSpinBox()
        self.sb_timeout.setRange(0.0, 1.0)
        self.sb_timeout.setDecimals(3)
        self.sb_timeout.setSingleStep(0.01)
        self.sb_timeout.setValue(0.05)

        self.btn_connect = QPushButton("Connect")
        self.btn_disconnect = QPushButton("Disconnect")
        self.btn_disconnect.setEnabled(False)

        conn_box = QGroupBox("Connection")
        conn_layout = QHBoxLayout()
        conn_layout.addWidget(QLabel("Port:"))
        conn_layout.addWidget(self.cb_port, 2)
        conn_layout.addWidget(self.btn_refresh)
        conn_layout.addWidget(QLabel("Baud:"))
        conn_layout.addWidget(self.sb_baud)
        conn_layout.addWidget(QLabel("Read chunk:"))
        conn_layout.addWidget(self.sb_read_chunk)
        conn_layout.addWidget(QLabel("Timeout(s):"))
        conn_layout.addWidget(self.sb_timeout)
        conn_layout.addWidget(self.btn_connect)
        conn_layout.addWidget(self.btn_disconnect)
        conn_box.setLayout(conn_layout)

        # --- Middle: joint table
        self.table = QTableWidget(6, 3)
        self.table.setHorizontalHeaderLabels(["Joint", "int32", "deg"])
        self.table.verticalHeader().setVisible(False)
        self.table.horizontalHeader().setSectionResizeMode(QHeaderView.Stretch)
        self.table.setEditTriggers(QTableWidget.NoEditTriggers)

        for i in range(6):
            self.table.setItem(i, 0, QTableWidgetItem(f"J{i+1}"))
            self.table.setItem(i, 1, QTableWidgetItem("-"))
            self.table.setItem(i, 2, QTableWidgetItem("-"))

        data_box = QGroupBox("Latest Telemetry")
        data_layout = QVBoxLayout()
        self.lbl_cmd = QLabel("CMD: --")
        self.lbl_crc = QLabel("CRC: --")
        self.lbl_counts = QLabel("Frames OK: 0 | CRC ERR: 0")
        self.lbl_cmd.setStyleSheet("font-weight: bold; font-size: 14px;")
        data_layout.addWidget(self.lbl_cmd)
        data_layout.addWidget(self.lbl_crc)
        data_layout.addWidget(self.lbl_counts)
        data_layout.addWidget(self.table)
        data_box.setLayout(data_layout)

        # --- Bottom: log
        self.log = QTextEdit()
        self.log.setReadOnly(True)
        self.log.setLineWrapMode(QTextEdit.NoWrap)

        log_box = QGroupBox("Log")
        log_layout = QVBoxLayout()
        log_layout.addWidget(self.log)
        log_box.setLayout(log_layout)

        # --- Main layout
        main = QVBoxLayout()
        main.addWidget(conn_box)
        main.addWidget(data_box, 2)
        main.addWidget(log_box, 1)
        self.setLayout(main)

        # signals
        self.btn_refresh.clicked.connect(self.refresh_ports)
        self.btn_connect.clicked.connect(self.on_connect)
        self.btn_disconnect.clicked.connect(self.on_disconnect)

        self.refresh_ports()

    def append_log(self, s: str):
        self.log.append(s)

    def refresh_ports(self):
        self.cb_port.clear()
        ports = list(list_ports.comports())
        for p in ports:
            # show friendly text
            text = f"{p.device}  ({p.description})"
            self.cb_port.addItem(text, p.device)

        if not ports:
            self.cb_port.addItem("No ports found", "")

    def on_connect(self):
        port = self.cb_port.currentData()
        if not port:
            QMessageBox.warning(self, "No port", "Không có cổng serial hợp lệ.")
            return

        baud = int(self.sb_baud.value())
        read_chunk = int(self.sb_read_chunk.value())
        timeout_s = float(self.sb_timeout.value())

        self.reader = SerialReaderThread(port, baud, read_chunk=read_chunk, timeout_s=timeout_s)
        self.reader.status.connect(self.append_log)
        self.reader.frame_bad.connect(self.append_log)
        self.reader.frame_ok.connect(self.on_frame_ok)

        self.reader.start()
        self.btn_connect.setEnabled(False)
        self.btn_disconnect.setEnabled(True)
        self.btn_refresh.setEnabled(False)

    def on_disconnect(self):
        if self.reader:
            self.reader.stop()
            self.reader.wait(1000)
            self.reader = None

        self.btn_connect.setEnabled(True)
        self.btn_disconnect.setEnabled(False)
        self.btn_refresh.setEnabled(True)
        self.append_log("Disconnected.")

    def on_frame_ok(self, pf: ParsedFrame):
        # update top labels
        self.lbl_cmd.setText(f"CMD: 0x{pf.cmd:02X}")
        self.lbl_crc.setText(f"CRC OK: 0x{pf.crc_calc:04X}")

        # update counts
        if self.reader:
            self.lbl_counts.setText(
                f"Frames OK: {self.reader.frames_ok_count} | CRC ERR: {self.reader.frames_crc_err_count}"
            )

        # update table
        for i in range(6):
            self.table.item(i, 1).setText(str(pf.joints_raw[i]))
            self.table.item(i, 2).setText(f"{pf.joints_deg[i]:.3f}")

        # optional: log raw frame (comment out if too spammy)
        self.append_log(f"[OK] {time.strftime('%H:%M:%S')} CMD=0x{pf.cmd:02X} | RAW: {pf.raw_hex}")

    def closeEvent(self, event):
        try:
            self.on_disconnect()
        finally:
            event.accept()


def main():
    app = QApplication(sys.argv)
    w = MonitorGUI()
    w.resize(1100, 700)
    w.show()
    sys.exit(app.exec_())


if __name__ == "__main__":
    main()

#!/usr/bin/env python3
"""
DM-IMU-L1 可视化上位机
- 3D 姿态显示 (PyOpenGL)
- 实时数据曲线 (QPainter)
- 支持串口直读或管道输入
"""

import sys
import struct
import argparse
import threading
import time
import math
from collections import deque
import numpy as np

from PyQt5 import QtWidgets, QtCore, QtGui
from PyQt5.QtWidgets import (QApplication, QMainWindow, QWidget, QVBoxLayout,
                              QHBoxLayout, QLabel, QSplitter, QGroupBox, QGridLayout)
from PyQt5.QtCore import Qt, QTimer, QMutex, QMutexLocker, QSettings
from PyQt5.QtGui import QColor, QPen, QPalette

# ── OpenGL ──────────────────────────────────────────────────────────────────
try:
    from OpenGL.GL import *
    from OpenGL.GLU import *
    HAS_OPENGL = True
except ImportError:
    HAS_OPENGL = False

# ── Serial ──────────────────────────────────────────────────────────────────
try:
    import serial
    HAS_SERIAL = True
except ImportError:
    HAS_SERIAL = False

# ═══════════════════════════════════════════════════════════════════════════
# CRC16 (from DM-IMU-L1 manual)
# ═══════════════════════════════════════════════════════════════════════════

CRC16_TABLE = [
    0x0000, 0x1021, 0x2042, 0x3063, 0x4084, 0x50A5, 0x60C6, 0x70E7,
    0x8108, 0x9129, 0xA14A, 0xB16B, 0xC18C, 0xD1AD, 0xE1CE, 0xF1EF,
    0x1231, 0x0210, 0x3273, 0x2252, 0x52B5, 0x4294, 0x72F7, 0x62D6,
    0x9339, 0x8318, 0xB37B, 0xA35A, 0xD3BD, 0xC39C, 0xF3FF, 0xE3DE,
    0x2462, 0x3443, 0x0420, 0x1401, 0x64E6, 0x74C7, 0x44A4, 0x5485,
    0xA56A, 0xB54B, 0x8528, 0x9509, 0xE5EE, 0xF5CF, 0xC5AC, 0xD58D,
    0x3653, 0x2672, 0x1611, 0x0630, 0x76D7, 0x66F6, 0x5695, 0x46B4,
    0xB75B, 0xA77A, 0x9719, 0x8738, 0xF7DF, 0xE7FE, 0xD79D, 0xC7BC,
    0x48C4, 0x58E5, 0x6886, 0x78A7, 0x0840, 0x1861, 0x2802, 0x3823,
    0xC9CC, 0xD9ED, 0xE98E, 0xF9AF, 0x8948, 0x9969, 0xA90A, 0xB92B,
    0x5AF5, 0x4AD4, 0x7AB7, 0x6A96, 0x1A71, 0x0A50, 0x3A33, 0x2A12,
    0xDBFD, 0xCBDC, 0xFBBF, 0xEB9E, 0x9B79, 0x8B58, 0xBB3B, 0xAB1A,
    0x6CA6, 0x7C87, 0x4CE4, 0x5CC5, 0x2C22, 0x3C03, 0x0C60, 0x1C41,
    0xEDAE, 0xFD8F, 0xCDEC, 0xDDCD, 0xAD2A, 0xBD0B, 0x8D68, 0x9D49,
    0x7E97, 0x6EB6, 0x5ED5, 0x4EF4, 0x3E13, 0x2E32, 0x1E51, 0x0E70,
    0xFF9F, 0xEFBE, 0xDFDD, 0xCFFC, 0xBF1B, 0xAF3A, 0x9F59, 0x8F78,
    0x9188, 0x81A9, 0xB1CA, 0xA1EB, 0xD10C, 0xC12D, 0xF14E, 0xE16F,
    0x1080, 0x00A1, 0x30C2, 0x20E3, 0x5004, 0x4025, 0x7046, 0x6067,
    0x83B9, 0x9398, 0xA3FB, 0xB3DA, 0xC33D, 0xD31C, 0xE37F, 0xF35E,
    0x02B1, 0x1290, 0x22F3, 0x32D2, 0x4235, 0x5214, 0x6277, 0x7256,
    0xB5EA, 0xA5CB, 0x95A8, 0x8589, 0xF56E, 0xE54F, 0xD52C, 0xC50D,
    0x34E2, 0x24C3, 0x14A0, 0x0481, 0x7466, 0x6447, 0x5424, 0x4405,
    0xA7DB, 0xB7FA, 0x8799, 0x97B8, 0xE75F, 0xF77E, 0xC71D, 0xD73C,
    0x26D3, 0x36F2, 0x0691, 0x16B0, 0x6657, 0x7676, 0x4615, 0x5634,
    0xD94C, 0xC96D, 0xF90E, 0xE92F, 0x99C8, 0x89E9, 0xB98A, 0xA9AB,
    0x5844, 0x4865, 0x7806, 0x6827, 0x18C0, 0x08E1, 0x3882, 0x28A3,
    0xCB7D, 0xDB5C, 0xEB3F, 0xFB1E, 0x8BF9, 0x9BD8, 0xABBB, 0xBB9A,
    0x4A75, 0x5A54, 0x6A37, 0x7A16, 0x0AF1, 0x1AD0, 0x2AB3, 0x3A92,
    0xFD2E, 0xED0F, 0xDD6C, 0xCD4D, 0xBDAA, 0xAD8B, 0x9DE8, 0x8DC9,
    0x7C26, 0x6C07, 0x5C64, 0x4C45, 0x3CA2, 0x2C83, 0x1CE0, 0x0CC1,
    0xEF1F, 0xFF3E, 0xCF5D, 0xDF7C, 0xAF9B, 0xBFBA, 0x8FD9, 0x9FF8,
    0x6E17, 0x7E36, 0x4E55, 0x5E74, 0x2E93, 0x3EB2, 0x0ED1, 0x1EF0
]


def crc16_compute(data: bytes) -> int:
    """Standard CRC-16/XMODEM (crc << 8)"""
    crc = 0xFFFF
    for b in data:
        idx = ((crc >> 8) ^ b) & 0xFF
        crc = ((crc << 8) ^ CRC16_TABLE[idx]) & 0xFFFF
    return crc


def crc16_v1(data: bytes) -> int:
    """As-written in manual (crc << 1)"""
    crc = 0xFFFF
    for b in data:
        idx = ((crc >> 8) ^ b) & 0xFF
        crc = ((crc << 1) ^ CRC16_TABLE[idx]) & 0xFFFF
    return crc


# ═══════════════════════════════════════════════════════════════════════════
# IMU Data structures
# ═══════════════════════════════════════════════════════════════════════════

class IMUData:
    __slots__ = ('dev_id', 'accel', 'gyro', 'euler', 'quat', 'updated', 'timestamp')

    def __init__(self):
        self.dev_id = 0
        self.accel = [0.0, 0.0, 0.0]   # g
        self.gyro = [0.0, 0.0, 0.0]    # deg/s
        self.euler = [0.0, 0.0, 0.0]   # roll, pitch, yaw (deg)
        self.quat = [0.0, 0.0, 0.0, 0.0]  # w, x, y, z
        self.updated = 0  # bitmask
        self.timestamp = time.time()


# ═══════════════════════════════════════════════════════════════════════════
# IMU Frame Parser (state machine, same as C++ version)
# ═══════════════════════════════════════════════════════════════════════════

class IMUParser:
    SYNC1, SYNC2, ID, TYPE, DATA, CRC1, CRC2, END = range(8)

    def __init__(self, skip_crc=False):
        self.state = self.SYNC1
        self._dev_id = 0
        self._frame_type = 0
        self._buf = bytearray()
        self._data_len = 0
        self._data = IMUData()
        self.callback = None
        self.skip_crc = skip_crc
        self.crc_errors = 0
        self.frame_count = 0

    @staticmethod
    def _expected_len(ftype):
        if ftype == 0x01: return 12  # accel
        if ftype == 0x02: return 12  # gyro
        if ftype == 0x03: return 12  # euler
        if ftype == 0x04: return 16  # quaternion
        return 0

    def feed(self, data: bytes):
        for b in data:
            if self.state == self.SYNC1:
                if b == 0x55:
                    self.state = self.SYNC2
            elif self.state == self.SYNC2:
                if b == 0xAA:
                    self.state = self.ID
                elif b != 0x55:
                    self.state = self.SYNC1
            elif self.state == self.ID:
                self._dev_id = b
                self.state = self.TYPE
            elif self.state == self.TYPE:
                self._frame_type = b
                self._data_len = self._expected_len(b)
                if self._data_len == 0:
                    self.state = self.SYNC1
                else:
                    self._buf = bytearray()
                    self.state = self.DATA
            elif self.state == self.DATA:
                self._buf.append(b)
                if len(self._buf) >= self._data_len:
                    self.state = self.CRC1
            elif self.state == self.CRC1:
                self._buf.append(b)  # CRC low
                self.state = self.CRC2
            elif self.state == self.CRC2:
                self._buf.append(b)  # CRC high
                self.state = self.END
            elif self.state == self.END:
                if b == 0x0A:
                    self._process_frame()
                self.state = self.SYNC1

    def _process_frame(self):
        data = bytes(self._buf[:self._data_len])

        if not self.skip_crc:
            crc_lo = self._buf[self._data_len]
            crc_hi = self._buf[self._data_len + 1]
            received_crc = crc_lo | (crc_hi << 8)

            scopes = [
                bytes([self._dev_id, self._frame_type]) + data,
                bytes([self._frame_type]) + data,
                data,
                bytes([0x55, 0xAA, self._dev_id, self._frame_type]) + data,
            ]

            ok = False
            for s in scopes:
                if crc16_compute(s) == received_crc or crc16_v1(s) == received_crc:
                    ok = True
                    break
                swapped = ((received_crc << 8) | (received_crc >> 8)) & 0xFFFF
                if crc16_compute(s) == swapped or crc16_v1(s) == swapped:
                    ok = True
                    break

            if not ok:
                self.crc_errors += 1
                return

        self.frame_count += 1
        d = self._data
        d.dev_id = self._dev_id
        d.timestamp = time.time()

        if self._frame_type == 0x01:  # Acceleration
            d.accel = list(struct.unpack('<fff', data))
            d.updated |= 1
        elif self._frame_type == 0x02:  # Gyro
            d.gyro = list(struct.unpack('<fff', data))
            d.updated |= 2
        elif self._frame_type == 0x03:  # Euler
            d.euler = list(struct.unpack('<fff', data))
            d.updated |= 4
        elif self._frame_type == 0x04:  # Quaternion
            d.quat = list(struct.unpack('<ffff', data))
            d.updated |= 8

        if d.updated == 0x0F and self.callback:
            self.callback(d)
            d.updated = 0


# ═══════════════════════════════════════════════════════════════════════════
# Serial Reader Thread
# ═══════════════════════════════════════════════════════════════════════════

class SerialReader(QtCore.QObject):
    """Reads from serial port or stdin in a worker thread."""
    data_ready = QtCore.pyqtSignal(IMUData)
    error = QtCore.pyqtSignal(str)
    connected = QtCore.pyqtSignal(bool)

    def __init__(self, port=None, baud=921600, skip_crc=False):
        super().__init__()
        self.port_name = port
        self.baud = baud
        self._running = False
        self._parser = IMUParser(skip_crc=skip_crc)
        self._ser = None
        self._write_lock = threading.Lock()

    def start(self):
        self._running = True
        self._parser.callback = lambda d: self.data_ready.emit(d)

        if self.port_name:
            self._thread = threading.Thread(target=self._read_serial, daemon=True)
        else:
            self._thread = threading.Thread(target=self._read_stdin, daemon=True)
        self._thread.start()

    def stop(self):
        self._running = False

    def send_raw(self, data: bytes) -> bool:
        """Thread-safe write to serial port."""
        with self._write_lock:
            if self._ser and self._ser.is_open:
                try:
                    self._ser.write(data)
                    return True
                except Exception:
                    return False
            return False

    def _read_serial(self):
        try:
            self._ser = serial.Serial(self.port_name, self.baud, timeout=0.1)
            self.connected.emit(True)
        except Exception as e:
            self.error.emit(f"Serial open failed: {e}")
            self.connected.emit(False)
            return

        while self._running:
            try:
                data = self._ser.read(4096)
                if data:
                    self._parser.feed(data)
            except Exception as e:
                self.error.emit(f"Serial read error: {e}")
                break
        self._ser.close()
        self._ser = None
        self._running = False
        self.connected.emit(False)

    def _read_stdin(self):
        self.connected.emit(True)
        while self._running:
            data = sys.stdin.buffer.read(4096)
            if not data:
                break
            self._parser.feed(data)
        self._running = False
        self.connected.emit(False)

    def get_stats(self):
        return (self._parser.frame_count, self._parser.crc_errors)


# ═══════════════════════════════════════════════════════════════════════════
# 3D Attitude OpenGL Widget
# ═══════════════════════════════════════════════════════════════════════════

class AttitudeWidget(QtWidgets.QOpenGLWidget):
    # IMU physical dimensions (ratios): 36 x 26 x 9 mm
    # X = longest side (36mm), Y = short side (26mm), Z = thickness (9mm)
    # Z axis passes through the largest face (PCB board surface)
    IMU_X, IMU_Y, IMU_Z = 1.0, 1.4, 0.35

    def __init__(self, parent=None):
        super().__init__(parent)
        self.euler = [0.0, 0.0, 0.0]
        self.quat = [1.0, 0.0, 0.0, 0.0]
        self._rot_x = 30.0
        self._rot_z = -30.0
        self._last_pos = None
        self._zoom = -6.0
        self.setMinimumSize(350, 350)

    def update_attitude(self, data: IMUData):
        self.euler = list(data.euler)
        self.quat = list(data.quat)
        self.update()

    # ── Quaternion to rotation matrix ──
    @staticmethod
    def _quat_to_matrix(q):
        """Return 4x4 column-major rotation matrix from quaternion [w,x,y,z]."""
        w, x, y, z = q[0], q[1], q[2], q[3]
        # Normalize
        n = math.sqrt(w*w + x*x + y*y + z*z)
        if n > 0:
            w, x, y, z = w/n, x/n, y/n, z/n
        return [
            1-2*(y*y+z*z), 2*(x*y+z*w),   2*(x*z-y*w),   0,
            2*(x*y-z*w),   1-2*(x*x+z*z), 2*(y*z+x*w),   0,
            2*(x*z+y*w),   2*(y*z-x*w),   1-2*(x*x+y*y), 0,
            0,             0,             0,             1
        ]

    def initializeGL(self):
        glClearColor(0.15, 0.15, 0.18, 1.0)
        glEnable(GL_DEPTH_TEST)
        glEnable(GL_LIGHTING)
        glEnable(GL_LIGHT0)
        glEnable(GL_COLOR_MATERIAL)
        glColorMaterial(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE)
        glLightfv(GL_LIGHT0, GL_POSITION, [5.0, 10.0, 8.0, 1.0])
        glLightfv(GL_LIGHT0, GL_AMBIENT, [0.2, 0.2, 0.2, 1.0])
        glLightfv(GL_LIGHT0, GL_DIFFUSE, [0.9, 0.9, 0.9, 1.0])
        glLightfv(GL_LIGHT0, GL_SPECULAR, [0.4, 0.4, 0.4, 1.0])
        glEnable(GL_BLEND)
        glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA)
        glEnable(GL_LINE_SMOOTH)
        glHint(GL_LINE_SMOOTH_HINT, GL_NICEST)

    def resizeGL(self, w, h):
        glViewport(0, 0, w, h)
        glMatrixMode(GL_PROJECTION)
        glLoadIdentity()
        aspect = w / max(h, 1)
        gluPerspective(45, aspect, 0.5, 50.0)
        glMatrixMode(GL_MODELVIEW)

    def paintGL(self):
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)
        glLoadIdentity()
        glTranslatef(0, 0, self._zoom)

        # User view rotation
        glRotatef(self._rot_x, 1, 0, 0)
        glRotatef(self._rot_z, 0, 0, 1)

        # Draw world reference grid (XY plane = ground)
        self._draw_grid()

        # Apply IMU attitude using quaternion for accurate rotation
        glMultMatrixf(self._quat_to_matrix(self.quat))

        self._draw_imu_body()
        self._draw_axes()

    def _draw_grid(self):
        glDisable(GL_LIGHTING)
        glColor4f(0.3, 0.3, 0.35, 0.5)
        glBegin(GL_LINES)
        for i in range(-5, 6):
            glVertex3f(i, -5, 0)
            glVertex3f(i, 5, 0)
            glVertex3f(-5, i, 0)
            glVertex3f(5, i, 0)
        glEnd()
        glEnable(GL_LIGHTING)

    def _draw_imu_body(self):
        """PCB-shaped box: Z=thin, top/bottom faces are largest."""
        w, h, d = self.IMU_X, self.IMU_Y, self.IMU_Z
        # Half-extents
        hx, hy, hz = w/2, h/2, d/2

        # Define 6 faces (CCW winding for correct normals)
        faces = [
            # +Z (Top PCB — largest face, green)
            ([(-hx, -hy, hz), (hx, -hy, hz), (hx, hy, hz), (-hx, hy, hz)],
             (0.25, 0.72, 0.30)),
            # -Z (Bottom PCB — largest face, dark)
            ([(-hx, hy, -hz), (hx, hy, -hz), (hx, -hy, -hz), (-hx, -hy, -hz)],
             (0.18, 0.18, 0.20)),
            # +Y (Front, connector side — red)
            ([(-hx, hy, -hz), (hx, hy, -hz), (hx, hy, hz), (-hx, hy, hz)],
             (0.75, 0.28, 0.25)),
            # -Y (Back — blue)
            ([(-hx, -hy, hz), (hx, -hy, hz), (hx, -hy, -hz), (-hx, -hy, -hz)],
             (0.25, 0.30, 0.70)),
            # +X (Right — orange)
            ([(hx, -hy, -hz), (hx, hy, -hz), (hx, hy, hz), (hx, -hy, hz)],
             (0.70, 0.45, 0.20)),
            # -X (Left — purple)
            ([(-hx, -hy, hz), (-hx, hy, hz), (-hx, hy, -hz), (-hx, -hy, -hz)],
             (0.45, 0.22, 0.65)),
        ]

        for verts, color in faces:
            glColor3f(*color)
            glBegin(GL_QUADS)
            u = (verts[1][0]-verts[0][0], verts[1][1]-verts[0][1], verts[1][2]-verts[0][2])
            v = (verts[3][0]-verts[0][0], verts[3][1]-verts[0][1], verts[3][2]-verts[0][2])
            nx = u[1]*v[2] - u[2]*v[1]
            ny = u[2]*v[0] - u[0]*v[2]
            nz = u[0]*v[1] - u[1]*v[0]
            glNormal3f(nx, ny, nz)
            for x, y, z in verts:
                glVertex3f(x, y, z)
            glEnd()

        # White edges
        glDisable(GL_LIGHTING)
        glColor4f(1.0, 1.0, 1.0, 0.5)
        glLineWidth(1.5)
        glBegin(GL_LINES)
        for verts, _ in faces:
            for i in range(4):
                a, b = verts[i], verts[(i+1)%4]
                glVertex3f(*a); glVertex3f(*b)
        glEnd()
        glLineWidth(1.0)
        glEnable(GL_LIGHTING)

    def _draw_axes(self):
        """X=Red, Y=Green, Z=Blue arrows extending from IMU body."""
        glDisable(GL_LIGHTING)
        axes = [
            ((-1, 0, 0), (1.0, 0.25, 0.25)),
            ((0, 1, 0), (0.25, 1.0, 0.25)),
            ((0, 0, 1), (0.30, 0.55, 1.0)),
        ]
        length = 2.0
        for (dx, dy, dz), color in axes:
            glColor3f(*color)
            glLineWidth(3.0)
            glBegin(GL_LINES)
            glVertex3f(0, 0, 0)
            glVertex3f(dx*length, dy*length, dz*length)
            glEnd()
            # Arrow head
            glBegin(GL_TRIANGLES)
            s = 0.18
            tip = (dx*length, dy*length, dz*length)
            if dx:
                glVertex3f(*tip)
                glVertex3f(dx*(length-s), dy*s, dz*s)
                glVertex3f(dx*(length-s), dy*-s, dz*s)
            elif dy:
                glVertex3f(*tip)
                glVertex3f(dx*s, dy*(length-s), dz*s)
                glVertex3f(dx*-s, dy*(length-s), dz*s)
            else:
                glVertex3f(*tip)
                glVertex3f(dx*s, dy*s, dz*(length-s))
                glVertex3f(dx*s, dy*-s, dz*(length-s))
            glEnd()
        glLineWidth(1.0)
        glEnable(GL_LIGHTING)

    def mousePressEvent(self, event):
        self._last_pos = event.pos()

    def mouseMoveEvent(self, event):
        if self._last_pos:
            dx = event.x() - self._last_pos.x()
            dy = event.y() - self._last_pos.y()
            self._rot_z += dx * 0.5
            self._rot_x += dy * 0.5
            self._last_pos = event.pos()
            self.update()

    def wheelEvent(self, event):
        delta = event.angleDelta().y()
        self._zoom += delta * 0.01
        self._zoom = max(-30.0, min(-2.0, self._zoom))
        self.update()


# ═══════════════════════════════════════════════════════════════════════════
# Real-time Curve Chart Widget
# ═══════════════════════════════════════════════════════════════════════════

class ChartWidget(QWidget):
    """Scrollable line chart with pixel-level downsampling for performance."""

    COLORS = [
        Qt.red, Qt.green, Qt.blue,
        Qt.cyan, Qt.magenta, Qt.yellow,
        Qt.white, Qt.gray
    ]

    def __init__(self, title="", labels=None, history_sec=10.0, parent=None):
        super().__init__(parent)
        self.title = title
        self.labels = labels or []
        self._history = history_sec
        self._data = [deque(maxlen=5000) for _ in self.labels]
        self._time = deque(maxlen=5000)
        self._range_min = 0.0
        self._range_max = 1.0
        self._auto_range = True
        self._range_margin = 0.1
        self._frame_skip = 0  # for throttled auto-range
        self.setMinimumHeight(120)
        self.setSizePolicy(QtWidgets.QSizePolicy.Expanding,
                           QtWidgets.QSizePolicy.Expanding)

    def add_data(self, t: float, values: list):
        self._time.append(t)
        for i, v in enumerate(values):
            if i < len(self._data):
                self._data[i].append(v)

    def paintEvent(self, event):
        n = len(self._time)
        if n < 2:
            return

        painter = QtGui.QPainter(self)
        painter.setRenderHint(QtGui.QPainter.Antialiasing)

        w = self.width()
        h = self.height()
        ml, mr, mt, mb = 55, 15, 25, 25
        pw = w - ml - mr
        ph = h - mt - mb
        if pw < 1:
            painter.end()
            return

        # Background
        painter.fillRect(0, 0, w, h, QColor(25, 25, 30))
        painter.fillRect(ml, mt, pw, ph, QColor(18, 18, 22))

        t_now = self._time[-1]
        t_min = t_now - self._history
        t_max = t_now
        t_span = max(t_max - t_min, 0.001)

        # ── Downsampled data extraction ──
        # Extract only as many points as pixel columns (pw)
        step = max(1, n // pw)  # stride through data
        # Use slice views — avoid list() copy
        times_raw = self._time
        data_raw = [d for d in self._data]

        # Build downsampled lists: take one point per pixel column
        if step == 1:
            # Fast path: fewer points than pixels, use all
            times = times_raw
            datasets = data_raw
        else:
            # Downsample: take min/max per bin for envelope, or just stride
            times = [times_raw[i] for i in range(0, n, step)]
            # Also include the last point
            if times[-1] != times_raw[-1]:
                times.append(times_raw[-1])
            datasets = []
            for d in data_raw:
                ds = [d[i] for i in range(0, n, step)]
                if ds[-1] != d[-1]:
                    ds.append(d[-1])
                datasets.append(ds)

        # ── Auto range (throttled to every 10 frames) ──
        self._frame_skip += 1
        if self._auto_range and self._frame_skip % 10 == 0:
            mn, mx = float('inf'), float('-inf')
            for ds in datasets:
                if ds:
                    mn = min(mn, min(ds))
                    mx = max(mx, max(ds))
            if mn < mx:
                span = mx - mn
                if span < 0.001:
                    span = 1.0
                self._range_min = mn - span * self._range_margin
                self._range_max = mx + span * self._range_margin

        v_min, v_max = self._range_min, self._range_max
        v_span = max(v_max - v_min, 0.001)

        # ── Coordinate transforms ──
        def tx(t):
            return ml + (t - t_min) / t_span * pw

        def ty(v):
            return mt + ph - (v - v_min) / v_span * ph

        # ── Grid ──
        painter.setPen(QPen(QColor(50, 50, 55), 1, Qt.DotLine))
        for i in range(6):
            y = mt + ph * i / 5
            painter.drawLine(ml, int(y), ml + pw, int(y))

        # ── Title ──
        painter.setPen(QColor(180, 180, 190))
        font = painter.font()
        font.setPointSize(9)
        font.setBold(True)
        painter.setFont(font)
        painter.drawText(ml, 3, pw, mt - 3, Qt.AlignLeft | Qt.AlignVCenter, self.title)

        # ── Y-axis labels ──
        font.setPointSize(7)
        font.setBold(False)
        painter.setFont(font)
        painter.setPen(QColor(120, 120, 130))
        for i in range(6):
            val = v_min + v_span * (5 - i) / 5
            y = mt + ph * i / 5
            painter.drawText(2, int(y - 7), ml - 4, 14,
                             Qt.AlignRight | Qt.AlignVCenter, f"{val:.1f}")

        # ── X-axis time labels ──
        for i in range(5):
            t_label = t_min + (t_max - t_min) * i / 4
            x = tx(t_label)
            label = f"{max(0, t_now - t_label):.0f}s"
            painter.drawText(int(x - 20), mt + ph + 2, 40, mb - 2,
                             Qt.AlignCenter, label)

        # ── Clipped drawing area ──
        painter.setClipRect(ml, mt, pw, ph)

        # ── Draw traces using QPainterPath (single draw call per trace) ──
        for i, (data, color) in enumerate(zip(datasets, self.COLORS)):
            if len(data) < 2 or len(times) < 2:
                continue
            path = QtGui.QPainterPath()
            x0 = tx(times[0])
            y0 = ty(data[0])
            path.moveTo(x0, y0)
            for j in range(1, len(data)):
                path.lineTo(tx(times[j]), ty(data[j]))
            painter.setPen(QPen(color, 1.5))
            painter.drawPath(path)

        # ── Legend ──
        painter.setClipRect(0, 0, w, h)
        lx, ly = ml + 5, mt + 5
        font.setPointSize(7)
        painter.setFont(font)
        for i, (label, color) in enumerate(zip(self.labels, self.COLORS)):
            if i >= len(self._data):
                break
            painter.fillRect(lx, ly + i * 14, 12, 10, color)
            painter.setPen(QColor(200, 200, 210))
            painter.drawText(lx + 16, ly + i * 14, 80, 10, Qt.AlignLeft, label)

        painter.end()


# ═══════════════════════════════════════════════════════════════════════════
# Data Display Panel (current values)
# ═══════════════════════════════════════════════════════════════════════════

class DataPanel(QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        layout = QGridLayout(self)
        layout.setSpacing(5)
        layout.setContentsMargins(5, 5, 5, 5)

        self._labels = {}

        groups = [
            ("Euler", [
                ("roll",  "Roll:",  "0.00°",   (255, 150, 100)),
                ("pitch", "Pitch:", "0.00°",   (100, 255, 150)),
                ("yaw",   "Yaw:",   "0.00°",   (100, 180, 255)),
            ]),
            ("Accel", [
                ("ax", "X:", "0.000 g", (255, 120, 120)),
                ("ay", "Y:", "0.000 g", (120, 255, 120)),
                ("az", "Z:", "0.000 g", (120, 180, 255)),
            ]),
            ("Gyro", [
                ("gx", "X:", "0.00 °/s", (255, 120, 120)),
                ("gy", "Y:", "0.00 °/s", (120, 255, 120)),
                ("gz", "Z:", "0.00 °/s", (120, 180, 255)),
            ]),
            ("Quat", [
                ("qw", "W:", "0.0000", (220, 220, 220)),
                ("qx", "X:", "0.0000", (255, 150, 150)),
                ("qy", "Y:", "0.0000", (150, 255, 150)),
                ("qz", "Z:", "0.0000", (150, 180, 255)),
            ]),
        ]

        row = 0
        for group_name, items in groups:
            gb = QGroupBox(group_name)
            gb.setStyleSheet(
                "QGroupBox { color: #aaa; font-weight: bold; "
                "border: 1px solid #444; border-radius: 4px; margin-top: 10px; "
                "padding-top: 12px; }"
                "QGroupBox::title { subcontrol-origin: margin; left: 8px; }")
            gl = QGridLayout(gb)
            gl.setSpacing(2)
            gl.setContentsMargins(8, 14, 8, 4)
            for i, (key, name, default, color) in enumerate(items):
                lbl_name = QLabel(name)
                lbl_name.setStyleSheet(f"color: rgb{tuple(color)}; font-weight: bold;")
                lbl_val = QLabel(default)
                lbl_val.setStyleSheet("color: #ddd; font-family: monospace;")
                lbl_val.setAlignment(Qt.AlignRight | Qt.AlignVCenter)
                lbl_val.setMinimumWidth(80)
                gl.addWidget(lbl_name, i, 0)
                gl.addWidget(lbl_val, i, 1)
                self._labels[key] = lbl_val
            layout.addWidget(gb, row, 0)
            row += 1

        layout.setRowStretch(row, 1)

    def update_data(self, data: IMUData):
        self._labels["roll"].setText(f"{data.euler[0]:+.2f}°")
        self._labels["pitch"].setText(f"{data.euler[1]:+.2f}°")
        self._labels["yaw"].setText(f"{data.euler[2]:+.2f}°")
        self._labels["ax"].setText(f"{data.accel[0]:+.4f} g")
        self._labels["ay"].setText(f"{data.accel[1]:+.4f} g")
        self._labels["az"].setText(f"{data.accel[2]:+.4f} g")
        self._labels["gx"].setText(f"{data.gyro[0]:+.2f} °/s")
        self._labels["gy"].setText(f"{data.gyro[1]:+.2f} °/s")
        self._labels["gz"].setText(f"{data.gyro[2]:+.2f} °/s")
        self._labels["qw"].setText(f"{data.quat[0]:+.4f}")
        self._labels["qx"].setText(f"{data.quat[1]:+.4f}")
        self._labels["qy"].setText(f"{data.quat[2]:+.4f}")
        self._labels["qz"].setText(f"{data.quat[3]:+.4f}")


# ═══════════════════════════════════════════════════════════════════════════
# Main Window
# ═══════════════════════════════════════════════════════════════════════════

class MainWindow(QMainWindow):
    def __init__(self, port=None, baud=921600):
        super().__init__()
        self.setWindowTitle("DM-IMU-L1 Viewer")
        self.setMinimumSize(1100, 700)
        self.setStyleSheet("QMainWindow { background-color: #1a1a20; }")

        self._data = IMUData()
        self._data_lock = QMutex()
        self._port_arg = port
        self._baud_arg = baud
        self._reader = None

        # ── Shared styles ──
        btn_style = ("QPushButton { background: #3a3a48; color: #ddd; border: 1px solid #555; "
                      "padding: 4px 14px; border-radius: 3px; }"
                      "QPushButton:hover { background: #4a4a5a; }"
                      "QPushButton:pressed { background: #2a2a38; }"
                      "QPushButton:disabled { color: #666; }")
        edit_style = ("QLineEdit { background: #1a1a24; color: #ddd; border: 1px solid #555; "
                       "padding: 3px 6px; border-radius: 3px; }")

        # ═════════════════════════════════════════════════════════════
        # MAIN CONTENT — widgets created first (toolbar references them)
        # ═════════════════════════════════════════════════════════════
        self.attitude = AttitudeWidget()

        self.euler_chart = ChartWidget("Euler Angles (°)",
                                       ["Roll", "Pitch", "Yaw"], history_sec=5)
        self.accel_chart = ChartWidget("Acceleration (g)",
                                       ["X", "Y", "Z"], history_sec=5)
        self.gyro_chart = ChartWidget("Angular Velocity (°/s)",
                                      ["X", "Y", "Z"], history_sec=5)
        self.data_panel = DataPanel()

        # ═════════════════════════════════════════════════════════════
        # TOP TOOLBAR
        # ═════════════════════════════════════════════════════════════
        toolbar = QWidget()
        toolbar.setStyleSheet("background: #22222c; border-bottom: 1px solid #444;")
        tl = QHBoxLayout(toolbar)
        tl.setContentsMargins(8, 4, 8, 4)
        tl.setSpacing(8)

        conn_label = QLabel("Port:")
        conn_label.setStyleSheet("color: #aaa;")
        self._port_edit = QtWidgets.QLineEdit(port or "/dev/ttyACM0")
        self._port_edit.setStyleSheet(edit_style)
        self._port_edit.setFixedWidth(140)

        btn_connect_style = ("QPushButton { background: #2a6a3a; color: #fff; border: 1px solid #4a4; "
                             "padding: 4px 14px; border-radius: 3px; font-weight: bold; }"
                             "QPushButton:hover { background: #3a8a4a; }")
        btn_disconnect_style = ("QPushButton { background: #6a2a2a; color: #fff; border: 1px solid #a44; "
                                "padding: 4px 14px; border-radius: 3px; font-weight: bold; }"
                                "QPushButton:hover { background: #8a3a3a; }")

        self._btn_toggle = QtWidgets.QPushButton("Connect")
        self._btn_toggle.setStyleSheet(btn_connect_style)
        self._btn_toggle.clicked.connect(self._toggle_connection)
        self._btn_connect_style = btn_connect_style
        self._btn_disconnect_style = btn_disconnect_style

        tl.addWidget(conn_label)
        tl.addWidget(self._port_edit)
        tl.addWidget(self._btn_toggle)

        # Separator
        sep1 = QLabel("│")
        sep1.setStyleSheet("color: #555;")
        tl.addWidget(sep1)

        # ── "View" dropdown (stays open on toggle via QWidgetAction+QCheckBox) ──
        self._view_btn = QtWidgets.QToolButton()
        self._view_btn.setText("View ▾")
        self._view_btn.setPopupMode(QtWidgets.QToolButton.InstantPopup)
        self._view_btn.setStyleSheet(
            "QToolButton { background: #3a3a48; color: #ddd; border: 1px solid #555; "
            "padding: 4px 12px; border-radius: 3px; }"
            "QToolButton::menu-indicator { image: none; }"
            "QToolButton:hover { background: #4a4a5a; }")

        view_menu = QtWidgets.QMenu(self._view_btn)
        view_menu.setStyleSheet(
            "QMenu { background: #2a2a36; color: #ddd; border: 1px solid #555; padding: 4px; }"
            "QMenu::item { padding: 4px 20px 4px 8px; }"
            "QMenu::item:selected { background: #3a3a50; }")

        cb_style = ("QCheckBox { color: #ddd; spacing: 6px; }"
                     "QCheckBox::indicator { width: 14px; height: 14px; }"
                     "QCheckBox::indicator:checked { background: #4a4; border: 1px solid #6c6; "
                     "border-radius: 2px; }"
                     "QCheckBox::indicator:unchecked { background: #333; border: 1px solid #555; "
                     "border-radius: 2px; }")

        def _make_cb(menu, text, widget):
            cb = QtWidgets.QCheckBox(text)
            cb.setChecked(True)
            cb.setStyleSheet(cb_style)
            cb.toggled.connect(widget.setVisible)
            act = QtWidgets.QWidgetAction(menu)
            act.setDefaultWidget(cb)
            menu.addAction(act)
            return cb

        self._cb_euler = _make_cb(view_menu, "Euler Chart", self.euler_chart)
        self._cb_accel = _make_cb(view_menu, "Accel Chart", self.accel_chart)
        self._cb_gyro = _make_cb(view_menu, "Gyro Chart", self.gyro_chart)
        view_menu.addSeparator()
        self._cb_data = _make_cb(view_menu, "Data Panel", self.data_panel)

        self._view_btn.setMenu(view_menu)
        tl.addWidget(self._view_btn)

        # Separator
        sep2 = QLabel("│")
        sep2.setStyleSheet("color: #555;")
        tl.addWidget(sep2)

        # CAN ID setting
        canid_label = QLabel("CAN ID:")
        canid_label.setStyleSheet("color: #aaa;")
        self._settings = QSettings("DM-IMU", "Viewer")
        self._canid_spin = QtWidgets.QSpinBox()
        self._canid_spin.setRange(0, 255)
        self._canid_spin.setValue(self._settings.value("can_id", 1, int))
        self._canid_spin.setFixedWidth(60)
        self._canid_spin.setStyleSheet(
            "QSpinBox { background: #1a1a24; color: #ddd; border: 1px solid #555; "
            "padding: 3px 4px; border-radius: 3px; }")
        self._btn_set_canid = QtWidgets.QPushButton("Set")
        self._btn_set_canid.setStyleSheet(
            "QPushButton { background: #3a3a48; color: #ddd; border: 1px solid #555; "
            "padding: 3px 10px; border-radius: 3px; }"
            "QPushButton:hover { background: #4a4a5a; }")
        self._btn_set_canid.clicked.connect(self._set_can_id)

        tl.addWidget(canid_label)
        tl.addWidget(self._canid_spin)
        tl.addWidget(self._btn_set_canid)

        tl.addStretch()

        right_chart_splitter = QSplitter(Qt.Vertical)
        right_chart_splitter.addWidget(self.euler_chart)
        right_chart_splitter.addWidget(self.accel_chart)
        right_chart_splitter.addWidget(self.gyro_chart)
        right_chart_splitter.setStretchFactor(0, 2)
        right_chart_splitter.setStretchFactor(1, 1)
        right_chart_splitter.setStretchFactor(2, 1)

        right_wrapper = QSplitter(Qt.Horizontal)
        right_wrapper.addWidget(right_chart_splitter)
        right_wrapper.addWidget(self.data_panel)
        right_wrapper.setStretchFactor(0, 3)
        right_wrapper.setStretchFactor(1, 1)
        right_wrapper.setSizes([600, 200])

        self._main_splitter = QSplitter(Qt.Horizontal)
        self._main_splitter.addWidget(self.attitude)
        self._main_splitter.addWidget(right_wrapper)
        self._main_splitter.setStretchFactor(0, 2)
        self._main_splitter.setStretchFactor(1, 4)
        self._main_splitter.setSizes([350, 750])

        # ═════════════════════════════════════════════════════════════
        # STATUS BAR (bottom)
        # ═════════════════════════════════════════════════════════════
        status_bar = QWidget()
        status_bar.setStyleSheet("background: #1c1c26; border-top: 1px solid #444;")
        sbl = QHBoxLayout(status_bar)
        sbl.setContentsMargins(10, 2, 10, 2)
        sbl.setSpacing(16)

        self._status_indicator = QLabel(" ⬤ ")
        self._status_indicator.setStyleSheet("color: #e55; font-size: 10px;")
        self._status_text = QLabel("Disconnected")
        self._status_text.setStyleSheet("color: #aaa;")
        sbl.addWidget(self._status_indicator)
        sbl.addWidget(self._status_text)

        sbl.addStretch()

        self._fps_label = QLabel("Data: -- Hz")
        self._fps_label.setStyleSheet("color: #888;")
        self._crc_label = QLabel("CRC: --")
        self._crc_label.setStyleSheet("color: #888;")
        self._id_label = QLabel("ID: --")
        self._id_label.setStyleSheet("color: #888;")
        sbl.addWidget(self._fps_label)
        sbl.addWidget(self._crc_label)
        sbl.addWidget(self._id_label)

        # ═════════════════════════════════════════════════════════════
        # Assemble into central widget
        # ═════════════════════════════════════════════════════════════
        central = QWidget()
        cl = QVBoxLayout(central)
        cl.setContentsMargins(0, 0, 0, 0)
        cl.setSpacing(0)
        cl.addWidget(toolbar)
        cl.addWidget(self._main_splitter, 1)
        cl.addWidget(status_bar)
        self.setCentralWidget(central)

        # ── Refresh timer (30 FPS) ──
        self._frame_count = 0
        self._refresh_timer = QTimer()
        self._refresh_timer.timeout.connect(self._update_ui)
        self._refresh_timer.start(33)
        self._last_fps_time = time.time()

        # Auto-connect if port was given on command line
        if port:
            self._do_connect()

    # ── CAN ID setting ──

    def _set_can_id(self):
        if not self._reader or not self._reader._running:
            self._status_text.setText("Not connected")
            return

        can_id = self._canid_spin.value()
        # Sequence: enter settings → set ID → save → normal
        # Per manual: must be in settings mode; must save after changes
        cmds = [
            (bytes([0xAA, 0x06, 0x01, 0x0D]), 150),   # enter settings mode
            (bytes([0xAA, 0x08, can_id, 0x0D]), 80),   # set CAN ID
            (bytes([0xAA, 0x03, 0x01, 0x0D]), 120),    # save parameters (keeps after power-off)
            (bytes([0xAA, 0x06, 0x00, 0x0D]), 0),      # enter normal mode
        ]

        def send_next(i=0):
            if i < len(cmds):
                cmd, delay = cmds[i]
                self._reader.send_raw(cmd)
                QTimer.singleShot(delay, lambda: send_next(i + 1))
            else:
                self._settings.setValue("can_id", can_id)
                self._status_text.setText(f"CAN ID set to {can_id}, saved")
                self._status_indicator.setStyleSheet("color: #5e5; font-size: 10px;")
                QTimer.singleShot(2000, lambda: self._status_text.setText("Connected"))

        send_next()

    # ── Connect / Disconnect ──

    def _toggle_connection(self):
        if self._reader and self._reader._running:
            self._do_disconnect()
        else:
            self._do_connect()

    def _do_connect(self):
        port = self._port_edit.text().strip()
        if self._reader:
            self._reader.stop()

        self._reader = SerialReader(port, 921600, skip_crc=True)
        self._reader.data_ready.connect(self._on_data)
        self._reader.connected.connect(self._on_connected)
        self._reader.error.connect(self._on_error)
        self._reader.start()

        self._btn_toggle.setText("Disconnect")
        self._btn_toggle.setStyleSheet(self._btn_disconnect_style)
        self._port_edit.setEnabled(False)
        self._status_text.setText("Connecting...")
        self._status_indicator.setStyleSheet("color: #ea0; font-size: 10px;")

    def _do_disconnect(self):
        if self._reader:
            self._reader.stop()
            self._reader = None

        self._btn_toggle.setText("Connect")
        self._btn_toggle.setStyleSheet(self._btn_connect_style)
        self._port_edit.setEnabled(True)
        self._status_text.setText("Disconnected")
        self._status_indicator.setStyleSheet("color: #e55; font-size: 10px;")
        self._fps_label.setText("Data: -- Hz")
        self._crc_label.setText("Frames: --")
        self._id_label.setText("ID: --")

    # ── Serial callbacks ──

    def closeEvent(self, event):
        if self._reader:
            self._reader.stop()
        super().closeEvent(event)

    def _on_data(self, d: IMUData):
        self._frame_count += 1

        # Feed charts at full data rate (not throttled to UI refresh)
        t = d.timestamp
        self.euler_chart.add_data(t, d.euler)
        self.accel_chart.add_data(t, d.accel)
        self.gyro_chart.add_data(t, d.gyro)

        # Update 3D and data panel (throttled per last stored)
        with QMutexLocker(self._data_lock):
            self._data = d

    def _on_connected(self, status: bool):
        if status:
            self._status_text.setText("Connected")
            self._status_indicator.setStyleSheet("color: #5e5; font-size: 10px;")
        else:
            self._status_text.setText("Connection lost")
            self._status_indicator.setStyleSheet("color: #e55; font-size: 10px;")
            self._btn_toggle.setText("Connect")
            self._btn_toggle.setStyleSheet(self._btn_connect_style)
            self._port_edit.setEnabled(True)

    def _on_error(self, msg: str):
        self._status_text.setText(f"Error: {msg}")
        self._status_indicator.setStyleSheet("color: #e55; font-size: 10px;")

    def _update_ui(self):
        if not self._reader:
            return

        # Stats once per second
        now = time.time()
        dt = now - self._last_fps_time
        if dt >= 1.0:
            if dt > 0:
                fps = self._frame_count / dt
                self._fps_label.setText(f"Data: {fps:.0f} Hz")
            self._frame_count = 0
            self._last_fps_time = now

            frames, _ = self._reader.get_stats()
            self._crc_label.setText(f"Frames: {frames}")

        # Repaint charts (data already pushed at full rate in _on_data)
        if self.euler_chart.isVisible():
            self.euler_chart.update()
        if self.accel_chart.isVisible():
            self.accel_chart.update()
        if self.gyro_chart.isVisible():
            self.gyro_chart.update()

        # 3D and data panel from latest stored data
        with QMutexLocker(self._data_lock):
            d = self._data
            if d.timestamp == 0:
                return
            dev_id = d.dev_id
            euler = list(d.euler)
            accel = list(d.accel)
            gyro = list(d.gyro)
            quat = list(d.quat)

        self._id_label.setText(f"ID: {dev_id}")

        dd = IMUData()
        dd.dev_id = dev_id
        dd.euler = euler
        dd.accel = accel
        dd.gyro = gyro
        dd.quat = quat
        self.attitude.update_attitude(dd)
        if self.data_panel.isVisible():
            self.data_panel.update_data(dd)


# ═══════════════════════════════════════════════════════════════════════════
# Main
# ═══════════════════════════════════════════════════════════════════════════

def main():
    parser = argparse.ArgumentParser(description="DM-IMU-L1 Visualizer")
    parser.add_argument("port", nargs="?", default=None,
                        help="Serial port (e.g. /dev/ttyACM0). If omitted, reads from stdin.")
    parser.add_argument("-b", "--baud", type=int, default=921600,
                        help="Baud rate (default: 921600)")
    args = parser.parse_args()

    app = QApplication(sys.argv)
    app.setStyle("Fusion")

    # Dark palette
    palette = app.palette()
    palette.setColor(QtGui.QPalette.Window, QColor(26, 26, 32))
    palette.setColor(QtGui.QPalette.WindowText, QColor(200, 200, 210))
    palette.setColor(QtGui.QPalette.Base, QColor(20, 20, 26))
    palette.setColor(QtGui.QPalette.AlternateBase, QColor(30, 30, 38))
    palette.setColor(QtGui.QPalette.Text, QColor(200, 200, 210))
    palette.setColor(QtGui.QPalette.Button, QColor(40, 40, 50))
    palette.setColor(QtGui.QPalette.ButtonText, QColor(200, 200, 210))
    app.setPalette(palette)

    if not HAS_OPENGL:
        QtWidgets.QMessageBox.critical(None, "Error",
            "PyOpenGL is required. Install with: pip install PyOpenGL")
        return 1

    window = MainWindow(args.port, args.baud)
    window.show()
    return app.exec_()


if __name__ == "__main__":
    sys.exit(main())

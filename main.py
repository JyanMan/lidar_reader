import sys
import time
import socket

import matplotlib.pyplot as plt
import matplotlib as mpl
import numpy as np
import serial
import math

from matplotlib.backends.backend_qtagg import FigureCanvasQTAgg
from matplotlib.backends.qt_compat import QtWidgets, QtCore
from matplotlib.figure import Figure
from queue import Queue

DEBUG = True

class Point:
    def __init__(self, x, y):
        self.x = x
        self.y = y

    def __str__(self):
        return f"({self.x}, {self.y})"


class PointsList:
    def __init__(self, points: list[Point]):
        self.points = points


DEBUG_PACKET = True
DEBUG_CHECKSUM = True
DEBUG_NEW_POINTS = True
class ReadLidarThread(QtCore.QThread):

    data_ready = QtCore.pyqtSignal(PointsList)
    
    def __init__(self):
        super().__init__()
        self._running = True

    def stop(self):
        self._running = False


    def compute_checksum(self, packet: bytearray) -> (int, int):
        total_len=len(packet)

        left_cs = 0x00
        right_cs = 0x00

        for i, b in enumerate(packet):
            if i == 8 or i == 9:
                continue
            if i % 2 == 0:
                left_cs ^= b
            else:
                right_cs ^= b

        return (left_cs, right_cs)
    
    def parse_data_to_point(self, packet) -> list[Point]:
        if DEBUG_PACKET:
            for b in packet:
                print(hex(b), end="|")

        if len(packet) < 11:  # minimal packet size
            return []

        ph = packet[0:2]
        ct = packet[2]
        lsn = packet[3]
        fsa = packet[4:6]
        lsa = packet[6:8]
        cs = packet[8:10]
        all_bytes = packet[0:len(packet)]

        num_sample = lsn

        if ct == 0:

            # checksum
            calc_cs = 0x0000 

            left_cs, right_cs = self.compute_checksum(packet)
            if DEBUG_CHECKSUM:
                print(f"{hex(left_cs)}, {hex(right_cs)}")

            if cs[0] != left_cs or cs[1] != right_cs:
                return []

        points = []
        raw_start_ang = ((fsa[1] << 8) | fsa[0])
        raw_end_ang = ((lsa[1] << 8) | lsa[0]) 


        start_ang = (raw_start_ang >> 1) / 64.0
        end_ang = (raw_end_ang >> 1) / 64.0

        if DEBUG:
            print(f"fsa: {raw_start_ang >> 1}, lsa: {raw_end_ang >> 1}");

        first_dist= ((packet[11] << 8) | packet[10]) / 4.0
        last_dist = ((packet[num_sample-1] << 8) | packet[num_sample-2]) / 4.0
                   
        clockwise_diff = 0
        if num_sample > 1:
            # strict clockwise from start to end --> not the shortest distance
            clockwise_diff = (
                end_ang - start_ang if start_ang <= end_ang
                else (end_ang + 360) - start_ang
            )

        for n in range(0, num_sample):
            
            sn_byte = packet[10 + n*2 : 12 + n*2]
            if len(sn_byte) < 2:
                continue

            dist = (sn_byte[1] << 8) | sn_byte[0]
            dist = dist / 4.0
            if dist <= 5:
                continue


            ang_correction =math.atan(21.8 * ((155.3-dist)/max(155.3*dist, 1)))

            step = (clockwise_diff) / max(num_sample-1, 1)
            ang = start_ang + step*n + ang_correction
            # ang = start_ang + step*n
            if ang < 0:
                ang += 360
            elif ang > 360:
                ang -= 360
            ang_rad = math.radians(ang) 

            new_point = Point(
                dist * math.cos(ang_rad),
                dist * math.sin(ang_rad)
            )

            if DEBUG_NEW_POINTS:
                print(f"step: {step}, ang: {ang} => {ang_rad}, dist: {dist} new point: {new_point}")

            points.append(new_point)
            

        return points



    def parse_packet(self, data):
        pass
        # for i in range(len(data)):
            
    
    def test_buffer_read(self, buffer):
        # buffer = bytearray()
        # buffer = bytearray([
        #     0xAA, 0x55,       # ph
        #     0x00,             # ct
        #     0x02,             # lsn
        #     0x00, 0x00,       # fsa (start angle)
        #     0x00, 0x05,       # lsa (end angle)
        #     0x00, 0x00,       # cs (checksum placeholder)
        #     0xA0, 0x0F,       # s1 = 1000mm
        #     0x40, 0x1F        # s2 = 2000mm
        # ])

        left_cs, right_cs = self.compute_checksum(buffer)
        buffer[8] = left_cs
        buffer[9] = right_cs
        self.curr_packet_size_left = 0
        points = []
        # while self._running:
        self.buffer = buffer
        self.read_buffer_update_to_point()


    def read_buffer_update_to_point(self):
        # # print(f"Received: {data.decode('utf-8')}")

        # leave room for header
        if len(self.buffer) < 2:
            return

        # print("second reach")

        # start only at the header
        if self.buffer[0] != 0xAA or self.buffer[1] != 0x55:
        #     # print(self.buffer)
            self.buffer.pop(0)
            return

        # print("third reach")

        # leave room for lsn
        if len(self.buffer) < 4:
            return

        if self.curr_packet_size_left == 0:

            lsn = self.buffer[3]
            FIXED_BYTES = 10  # PH(2) + CT(1) + LSN(1) + FSA(2) + LSA(2) + CS(2) = 10
            self.curr_packet_size_left = FIXED_BYTES + (lsn * 2)

        # print("reaches fourth")

        next_header_idx = self.buffer.find(bytearray([0xAA, 0x55]), 1)
        if next_header_idx != -1 and next_header_idx < self.curr_packet_size_left:
            self.buffer = self.buffer[next_header_idx:]  # discard bytes before next header
            self.curr_packet_size_left = 0
            return

        # print("reaches fifth")

        # check if packet_size was reached
        if len(self.buffer) >= self.curr_packet_size_left:
        #     print("reaches sixth")
            packet = self.buffer[:self.curr_packet_size_left]
            self.points = self.points + self.parse_data_to_point(packet)
            self.buffer = self.buffer[self.curr_packet_size_left:]
            self.curr_packet_size_left = 0

            if len(self.points) != 0:
        #         print("points were made")
        #         print("ADD POINTS")
                self.data_ready.emit(PointsList(self.points))

            self.buffer.clear()
            return

    def read_from_socket(self):
        
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.bind(("127.0.0.1", 2000))

        while self._running:
            # time.sleep(0.1)
            data, addr = s.recvfrom(64)
            # Receive data in a self.buffer size of 1024 bytes
            # new_buff = conn.recv(64)
            if not data:
                print("no received data")
                # If recv returns an empty bytes object, the client closed the connection
                return
            # print("first reach")
            new_buff = data
            self.buffer += new_buff
            self.read_buffer_update_to_point()

    def read_from_lidar(self):
        while self._running:
            try: 
                with serial.Serial('/dev/ttyUSB0', 115200, timeout=1) as ser:
                    new_buff = ser.read(ser.in_waiting or 64)
                    self.buffer += new_buff
                    self.read_buffer_update_to_point()
            except serial.SerialException as e:
                log(e)
                time.sleep(1)

    
    def run(self):
        self.buffer = bytearray()
        self.curr_packet_size_left = 0
        self.points: list[Point] = []

        self.read_from_socket()
        # self.test_buffer_read(bytearray([
        #     0xaa , 0x55 , 0x00 , 0x02 , 0x33 , 0x69 , 0xc5 , 0x69 , 0xad , 0x57 , 0x8a , 0x04 , 0x7b, 0x04
        # ]))
        # self.read_from_lidar()


def log(*msg: str):
    print(f"{TAG}:", *msg)


class AskForPointCoordinateDialog(QtWidgets.QDialog):
    def __init__(self, point: Point):
        super().__init__()

        self.setWindowTitle("HELLO!")

        self.new_point = point

        self.btn_apply = QtWidgets.QDialogButtonBox(
            QtWidgets.QDialogButtonBox.StandardButton.Apply 
        ) 
        self.btn_apply.clicked.connect(self.add_point)

        self.btn_cancel= QtWidgets.QDialogButtonBox(
            QtWidgets.QDialogButtonBox.StandardButton.Cancel
        )
        self.btn_cancel.clicked.connect(self.reject)

        self.input_field = QtWidgets.QLineEdit()

        layout = QtWidgets.QVBoxLayout()
        message = QtWidgets.QLabel("input coordinates")
        layout.addWidget(message)
        layout.addWidget(self.input_field)
        layout.addWidget(self.btn_cancel)
        layout.addWidget(self.btn_apply)
        self.setLayout(layout)

    def add_point(self):
        log("point added")
        data = self.input_field.text()
        try:
            data = data.split(',')
            x = data[0]
            y = data[1]
            self.new_point.x = int(x)
            self.new_point.y = int(y)
        except e:
            log(e)
        self.accept()

class ApplicationWindow(QtWidgets.QMainWindow):
    def __init__(self):
        super().__init__()

        self.layout = QtWidgets.QGridLayout()
        
        self.fig= FigureCanvasQTAgg(Figure(figsize=(3, 3)))
        self.fig.figure.add_subplot(1, 1, 1)
        axes_list = self.fig.figure.axes
        axes_list[0].plot(1, 2, 'bo')
        axes_list[0].plot(0.5, 0, 'bo')
        axes_list[0].plot(0, 0.5, 'bo')
        axes_list[0].relim()
        axes_list[0].autoscale_view()
        self.fig.figure.canvas.draw()

        self.btn_add_point = QtWidgets.QPushButton()
        self.btn_add_point.clicked.connect(self.add_point_prompt)

        self.layout.addWidget(self.btn_add_point)
        self.layout.addWidget(self.fig)
        widget = QtWidgets.QWidget()

        widget.setLayout(self.layout)
        self.setCentralWidget(widget)

        self.init_tasks()
        self.init_queues()


    def closeEvent(self, e):
        pass
        # self.thread.stop()


    def add_point(self, points: PointsList):
        for p in points.points:
            self.fig.figure.axes[0].plot(p.x, p.y, 'bo')

        self.fig.figure.canvas.draw()
    

    def add_point_prompt(self):
        new_point = Point(0, 0)
        dlg = AskForPointCoordinateDialog(new_point)
        dlg.setWindowTitle("Add Point")

        log("adding another point, dialog show")
        if dlg.exec():
            log(f"new_point: {new_point.x}, {new_point.y}")
            self.add_point(new_point)
        else:
            log("failed to exec dialog window")

    def init_queues(self):
        self.lidar_data_q = Queue()


    def init_tasks(self):
        self.thread = ReadLidarThread()  
        self.thread.data_ready.connect(self.add_point)
        self.thread.start()

        # pool = QtCore.QThreadPool.globalInstance()
        # self.read_lidar_runnable = ReadLidarRunnable()
        # pool.start(self.read_lidar_runnable)


TAG = "LOG"


if __name__ == "__main__":

    mpl.rcParams["backend"] = "qtagg"
    log(f"backend -> {mpl.get_backend()}")

    app = QtWidgets.QApplication(sys.argv)
    window = ApplicationWindow()
    window.show()
    app.exec()

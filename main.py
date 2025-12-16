import sys
import time

import matplotlib.pyplot as plt
import matplotlib as mpl
import numpy as np
import serial
import math

from matplotlib.backends.backend_qtagg import FigureCanvasQTAgg
from matplotlib.backends.qt_compat import QtWidgets, QtCore
from matplotlib.figure import Figure
from queue import Queue


class Point:
    def __init__(self, x, y):
        self.x = x
        self.y = y


class PointsList:
    def __init__(self, points: list[Point]):
        self.points = points



class ReadLidarThread(QtCore.QThread):

    data_ready = QtCore.pyqtSignal(PointsList)
    
    def __init__(self):
        super().__init__()
        self._running = True

    def stop(self):
        self._running = False

    # def parse_data_to_point(self, packet) -> list[Point]:
    #     # exclusive end
    #     def from_data(start, end):
    #         return packet[start:end]

    #     # lsb is index 0, msb is index 1
    #     ph = from_data(0,2)
    #     ct = from_data(2,3)[0]
    #     lsn = from_data(3,4)[0]
    #     fsa = from_data(4,6)
    #     lsa = from_data(6,8)
    #     cs = from_data(8,10) 
    #     all = from_data(0, len(packet)-1)

    #     num_sample = lsn
    #     start_ang = (fsa[1] << 8) | fsa[0]
    #     end_ang = (lsa[1] << 8) | lsa[0]

    #     calc_cs = 0x0000
    #     for n in range(0, len(all)-1, 2):
    #         two_byte = (all[n+1] << 8) | all[n]
    #         calc_cs ^= two_byte

    #     word_cs = (cs[0] << 8) | cs[1]
    #     if calc_cs != word_cs:
    #         log(f"calc_cs: {hex(calc_cs)}\ncs: {hex(word_cs)}")
    #         return []
                
    #     points = []
    #     for n in range(num_sample):
    #         sn_byte = from_data(11+(n*2), 11+(n*2)+2)

    #         step = (end_ang - start_ang) / max(num_sample - 1, 1)
    #         ang = start_ang + step * n
    #         ang = math.radians(ang)

    #         if len(sn_byte) < 2:
    #             log("broken packet")
    #             continue

    #         sn = (sn_byte[1] << 8) | sn_byte[0]
    #         # ang = (end_ang-start_ang)*(i-1) + start_ang
    #         points.append(Point(
    #             sn * math.cos(ang),
    #             sn * math.sin(ang)
    #         ))

    #     return points
        

    def parse_data_to_point(self, packet) -> list[Point]:
        # print("THE HEX")
        # for b in packet:
        #     print(hex(b), end=" | ")

        # print()
        # print("END HEX")
        
        def from_data(start, end):
            return packet[start:end]

        if len(packet) < 11:  # minimal packet size
            return []

        ph = packet[0:2]
        ct = packet[2]
        lsn = packet[3]
        fsa = packet[4:6]
        lsa = packet[6:8]
        cs = packet[8:10]
        all_bytes = packet[0:len(packet)]

        if ct != 0:
            print("ct is zero, start packet")
            return []

        num_sample = lsn
        if num_sample == 0:
            return []

        start_ang = (fsa[1] << 8) | fsa[0]
        end_ang = (lsa[1] << 8) | lsa[0]

        # checksum
        calc_cs = 0x0000 

        total_len=10 + 2 * lsn

        left_cs = 0x00
        right_cs = 0x00

        for i, b in enumerate(packet):
            if i == 8 or i == 9:
                continue
            if i % 2 == 0:
                left_cs ^= b
            else:
                right_cs ^= b

        print(f"{hex(left_cs)}, {hex(right_cs)}")

        # for i, b in enumerate(packet):
        #     if i % 2 != 0:
        #         calc_cs |= (b << 8) 
        #     else:
        #         calc_cs |= (b)
            
        
        # for n in range(0, total_len-1, 2):
        #     if n == 8 or n==9:
        #         continue
        #     two_byte = (all_bytes[n+1] << 8) | all_bytes[n]
        #     before_cs = calc_cs
        #     calc_cs ^= two_byte
        #     print(f"{hex(before_cs)} ^ {hex(two_byte)} ({n}): {hex(calc_cs)}")
        # calc_cs = 0x0000
        # for n in range(0, total_len, 2):
        #     # skip CS bytes
        #     if n == 8:
        #         continue
    
        #     if n+1 >= len(all_bytes):
        #         two_byte = all_bytes[n]
        #     else:
        #         two_byte = (all_bytes[n+1] << 8) | all_bytes[n]
    
        #     before_cs = calc_cs
        #     calc_cs ^= two_byte
        #     print(f"{hex(before_cs)} ^ {hex(two_byte)} ({n}): {hex(calc_cs)}")

        

        # word_cs = (cs[1] << 8) | cs[0]
        if cs[0] != left_cs or cs[1] != right_cs:
            log(f"calc_cs: {hex(left_cs)} {hex(right_cs)}, packet_cs: {hex(cs[0])} {hex(cs[1])}")
            return []

        points = []
        for n in range(num_sample):
            sn_byte = from_data(11 + n*2, 11 + n*2 + 2)
            if len(sn_byte) < 2:
                log("broken packet")
                continue

            sn = (sn_byte[1] << 8) | sn_byte[0]
            step = (end_ang - start_ang) / max(num_sample - 1, 1)
            ang = math.radians(start_ang + step * n)

            points.append(Point(
                sn * math.cos(ang),
                sn * math.sin(ang)
            ))

        return points



    def parse_packet(self, data):
        pass
        # for i in range(len(data)):
            
    
    def test_buffer_read(self):
        # buffer = bytearray()
        buffer = bytearray([
            0xaa, 0x55, 0x22, 0x28, 0x51, 0x36, 0xa7, 0x43,
            0x2c, 0x04, 0xf8, 0x18, 0x88, 0x19, 0x18, 0x1a,
            0xa0, 0x1b, 0x80, 0x1b, 0x40, 0x1a, 0x0c, 0x1b,
            0xbc, 0x1a, 0x68, 0x1a, 0x70, 0x1a, 0x3c, 0x1a,
            0x10, 0x1a, 0xf0, 0x19, 0xc8, 0x19, 0xa4, 0x19,
            0x7c, 0x19, 0x58, 0x19, 0x34, 0x19, 0x14, 0x19,
            0xf4, 0x18, 0xd4, 0x18, 0xb4, 0x18, 0x98, 0x18,
            0x78, 0x18, 0x60, 0x18, 0x8a, 0x15, 0x64, 0x15,
            0xfc, 0x15, 0x00, 0x16, 0x00, 0x16, 0x04, 0x16,
            0x0c, 0x16, 0x08, 0x16, 0x20, 0x16, 0x28, 0x16,
            0x34, 0x16, 0x48, 0x16, 0x58, 0x16, 0x68, 0x16,
            0x78, 0x16
        ])
        curr_packet_size_left = 0
        points = []
        while self._running:
            # new_buff = ser.read(ser.in_waiting or 64)
            # buffer += new_buff

            # leave room for header
            # print(f"buff size: {len(buffer)}, curr_packet_size_left: {curr_packet_size_left}")
            if len(buffer) < 2:
                continue

            # start only at the header
            if buffer[0] != 0xAA or buffer[1] != 0x55:
                buffer.clear()
                continue

            # leave room for lsn
            if len(buffer) < 4:
                continue


            if curr_packet_size_left == 0:

                lsn = buffer[3]
                FIXED_BYTES = 10  # header + fixed fields (example)
                curr_packet_size_left = FIXED_BYTES + (lsn * 2)
                print(f"the supposed length: {curr_packet_size_left}")

            next_header_idx = buffer.find(bytearray([0xAA, 0x55]), 1)
            if next_header_idx != -1 and next_header_idx < curr_packet_size_left:
                print("PACKET WAS LOST")
                buffer = buffer[next_header_idx:]  # discard bytes before next header
                curr_packet_size_left = 0
                continue

            # check if packet_size was reached
            if len(buffer) >= curr_packet_size_left:
                packet = buffer[:curr_packet_size_left]
                print("NO PACKET WAS LOST")
                points = points + self.parse_data_to_point(packet)
                buffer = buffer[curr_packet_size_left:]
                curr_packet_size_left = 0
                continue
    
    def run(self):
        buffer = bytearray()
        curr_packet_size_left = 0
        # self.test_buffer_read()

        # return
        
        while self._running:
            time.sleep(0.1)
            try:
                points = []
                with serial.Serial('/dev/ttyUSB0', 115200, timeout=1) as ser:

                    new_buff = ser.read(ser.in_waiting or 64)
                    buffer += new_buff

                    # leave room for header
                    # print(f"buff size: {len(buffer)}, curr_packet_size_left: {curr_packet_size_left}")
                    if len(buffer) < 2:
                        continue

                    # start only at the header
                    if buffer[0] != 0xAA or buffer[1] != 0x55:
                        buffer.clear()
                        continue

                    # leave room for lsn
                    if len(buffer) < 4:
                        continue


                    if curr_packet_size_left == 0:

                        lsn = buffer[3]
                        # FIXED_BYTES = 12  # header + fixed fields (example)
                        FIXED_BYTES = 10  # PH(2) + CT(1) + LSN(1) + FSA(2) + LSA(2) + CS(2) = 10
                        # CHECK_SUM_B = 2
                        curr_packet_size_left = FIXED_BYTES + (lsn * 2)

                    next_header_idx = buffer.find(bytearray([0xAA, 0x55]), 1)
                    if next_header_idx != -1 and next_header_idx < curr_packet_size_left:
                        print("PACKET WAS LOST")
                        buffer = buffer[next_header_idx:]  # discard bytes before next header
                        curr_packet_size_left = 0
                        continue

                    # check if packet_size was reached
                    if len(buffer) >= curr_packet_size_left:
                        packet = buffer[:curr_packet_size_left]
                        print("NO PACKET WAS LOST")
                        points = points + self.parse_data_to_point(packet)
                        buffer = buffer[curr_packet_size_left:]
                        curr_packet_size_left = 0
                        if len(points) != 0:
                            print("ADD POINT")
                            self.data_ready.emit(PointsList(points))
                        continue


                    # time.sleep(3)
            except serial.SerialException as e:
                log(e)
                time.sleep(1)


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

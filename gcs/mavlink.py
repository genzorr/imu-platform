from PyQt5.QtCore import QThread, pyqtSignal
from pymavlink.dialects.v10.mavmessages import *
from pymavlink import mavutil
import serial as ser

import time

ACCUM_LEN = 5
serial_device = '/dev/ttyUSB0'
serial_baud = 115200

class MsgAccumulator:
    def __init__(self, batch_size, signal):
        self.batch_size = batch_size
        self.signal = signal
        self.accumulator = []

    def push_message(self, msg):
        self.accumulator.append(msg)
        if len(self.accumulator) >= self.batch_size:
            self.signal.emit(self.accumulator)          # send all accumulator to slot @QtCore.pyqtSlot(list)
            self.accumulator = []
            # print('PUSH COMPLETED')


class MavlinkThread(QThread):
    new_state_record = pyqtSignal(list)
    new_imu_isc_record = pyqtSignal(list)
    new_imu_rsc_record = pyqtSignal(list)

    def __init__(self):
        QThread.__init__(self)
        self.state_accum = MsgAccumulator(ACCUM_LEN, self.new_state_record)
        self.imu_isc_accum = MsgAccumulator(ACCUM_LEN, self.new_imu_isc_record)
        self.imu_rsc_accum = MsgAccumulator(ACCUM_LEN, self.new_imu_rsc_record)

    def process_message(self, msg):
        if isinstance(msg, MAVLink_state_message):
            self.state_accum.push_message(msg)
        if isinstance(msg, MAVLink_imu_isc_message):
            self.imu_isc_accum.push_message(msg)
        if isinstance(msg, MAVLink_imu_rsc_message):
            self.imu_rsc_accum.push_message(msg)


    def run(self):
        # try:
        #     # mav = mavutil.mavserial(serial_device, baud=serial_baud, autoreconnect=True)
        #     # mav = mavutil.mavlink_connection('udpin:0.0.0.0:10000', dialect='mavmessages')
        # except BaseException:
        #     print('error')
        #     return
        # mav = mavutil.mavlink_connection(device=serial_device, baud=serial_baud, dialect='mavmessages')
        # mav = mavutil.mavserial(device=serial_device, baud=serial_baud, autoreconnect=True)
        t = time.time()
        mav = mavutil.mavlink_connection('udpin:0.0.0.0:10000', dialect='mavmessages')
        while True:
            pack = mav.recv_match(blocking=False)
            # if pack:
            #     print(pack)
            t_prev = t
            t = time.time()
            print(t - t_prev)
            self.process_message(pack)

# ;MAVLINK20=TRUE;MAVLINK_DIALECT=mavmessages
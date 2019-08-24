from PyQt5.QtCore import QThread, pyqtSignal
from pymavlink.dialects.v10.mavmessages import *
from pymavlink import mavutil
import struct

ACCUM_LEN = 10

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
        mav = mavutil.mavlink_connection("udpin:192.168.0.103:10000")

        while True:
            pack = mav.recv_match(blocking=False)
            if pack:
                print(pack)
            self.process_message(pack)
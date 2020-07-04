import serial
import struct

serial_device = '/dev/ttyUSB0'
serial_baud = 115200

class SerialMessage:
    def __init__(self):
        self.time = 0
        self.MPU_state = 0
        self.NRF_state = 0
        self.accel_rsc = []
        self.gyro = []
        self.magn = []
        self.accel_isc = []
        self.quaternion = []
        self.data_size = 4

def bytes_to_int(bytes):
    result = 0
    for b in bytes:
        result = result * 256 + int(b)
    return result

def bytes_to_float(bytes):
    x = 0
    [x] = struct.unpack('f', bytes)
    return x

def bytes_to_double(bytes):
    x = 0
    [x] = struct.unpack('d', bytes)
    return x

class SerialConnector:
    def __init__(self):
        self.dev = serial.Serial(
                    port=serial_device,
                    baudrate=serial_baud,
                    parity=serial.PARITY_NONE,
                    stopbits=serial.STOPBITS_ONE,
                    bytesize=serial.EIGHTBITS
                )
        if not self.dev.is_open:
            self.dev.open()

        self.time_prev = 0

    def close(self):
        self.dev.close()

    def parse_packet(self):
        while True:
            descr_val1 = bytes_to_int(self.dev.read(1))
            descr_val2 = bytes_to_int(self.dev.read(1))

            if (descr_val1 != 0x0A) and (descr_val2 != 0x0A):
                if (descr_val2 == 0x0A):
                    descr_val1 = 0x0A
                    descr_val2 = bytes_to_int(self.dev.read(1))

                    if (descr_val2 == 0x0A):
                        break

                print("Bad index", descr_val1, descr_val2)
            else:
                break

        packet = SerialMessage()

        packet.time = bytes_to_float(self.dev.read(packet.data_size))

        print(packet.time - self.time_prev)
        self.time_prev = packet.time

        packet.MPU_state = bytes_to_int(self.dev.read(1))
        packet.NRF_state = bytes_to_int(self.dev.read(1))

        for i in range(3):
            if (packet.data_size == 4):
                packet.accel_rsc.append(bytes_to_float(self.dev.read(packet.data_size)))
            if (packet.data_size == 8):
                packet.accel_rsc.append(bytes_to_double(self.dev.read(packet.data_size)))

        for i in range(3):
            if (packet.data_size == 4):
                packet.gyro.append(bytes_to_float(self.dev.read(packet.data_size)))
            if (packet.data_size == 8):
                packet.gyro.append(bytes_to_double(self.dev.read(packet.data_size)))

        for i in range(3):
            if (packet.data_size == 4):
                packet.magn.append(bytes_to_float(self.dev.read(packet.data_size)))
            if (packet.data_size == 8):
                packet.magn.append(bytes_to_double(self.dev.read(packet.data_size)))

        for i in range(3):
            if (packet.data_size == 4):
                packet.accel_isc.append(bytes_to_float(self.dev.read(packet.data_size)))
            if (packet.data_size == 8):
                packet.accel_isc.append(bytes_to_double(self.dev.read(packet.data_size)))

        for i in range(4):
            if (packet.data_size == 4):
                packet.quaternion.append(bytes_to_float(self.dev.read(packet.data_size)))
            if (packet.data_size == 8):
                packet.quaternion.append(bytes_to_double(self.dev.read(packet.data_size)))

        return packet
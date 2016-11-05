import spidev
import time
import struct
from sys import stdout

class EncoderReader(object):
    """
    publish estimate position of robot
    """
    def __init__(self, device, port, speed):
        # Define all constants.
        self.CLEAR_COUNTER = 0x20
        self.CLEAR_STATUS = 0x30
        self.READ_COUNTER = 0x60
        self.READ_STATUS = 0x70
        self.WRITE_MODE0 = 0x88
        self.WRITE_MODE1 = 0x90

        self.FOURX_COUNT = 0x03
        self.FOURBYTE_COUNTER = 0x00
        self.THREEBYTE_COUNTER = 0x01
        self.TWOBYTE_COUNTER = 0x02
        self.ONEBYTE_COUNTER = 0x03

        # Counter Size (be sure to change this value if settings are changed)
        self.counterSize = 4

        self.spi = spidev.SpiDev()  # Initialize spi object.
        self.spi.open(device, port)
        self.spi.max_speed_hz = speed
        self.encoder_count = 0

    def init_encoder(self):
        self.clear_counter()
        self.clear_status()

        self.spi.xfer2([self.WRITE_MODE0])
        self.spi.xfer2([self.FOURX_COUNT])

        time.sleep(.1)

        self.spi.xfer2([self.WRITE_MODE1])
        self.spi.xfer2([self.FOURX_COUNT])

    def clear_counter(self):
        self.spi.xfer2([self.CLEAR_COUNTER])

        # set offset
        self.spi.xfer2([0x98, 0x01, 0x00, 0x00, 0x00])
        self.spi.xfer2([0xE0])

        time.sleep(.1)

    def clear_status(self):
        self.spi.xfer2([self.CLEAR_STATUS])

    def get_number(data):
        print data
        bytes_tmp = list(reversed(data))
        return struct.unpack('@l', "".join(map(chr, bytes_tmp)))[0]

    def read_counter(self):
        data = [0, 0, 0, 0, 0]

        data = self.spi.xfer2([self.READ_COUNTER, 0, 0, 0, 0])
        self.encoder_count = 0
        for i in range(self.counterSize):
            self.encoder_count = self.encoder_count*256 + data[i+1]

        # encoder_count = get_number(data[1:5])
        return self.encoder_count

    def read_status(self):
        # data = []
        self.spi.xfer2([self.READ_STATUS])
        data = self.spi.xfer2([0x00])
        return data

    def close_spi(self):
        self.spi.close()
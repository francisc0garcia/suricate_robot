import spidev
import time
import struct
from sys import stdout

# Define all constants.
CLEAR_COUNTER = 0x20
CLEAR_STATUS = 0x30
READ_COUNTER = 0x60
READ_STATUS = 0x70
WRITE_MODE0 = 0x88
WRITE_MODE1 = 0x90

FOURX_COUNT = 0x03
FOURBYTE_COUNTER = 0x00
THREEBYTE_COUNTER = 0x01
TWOBYTE_COUNTER = 0x02
ONEBYTE_COUNTER = 0x03

# Counter Size (be sure to change this value if settings are changed)
counterSize = 4

spi = spidev.SpiDev()  # Initialize spi object.
spi.open(0, 0)
spi.max_speed_hz = 500000


def init_encoder():
    print 'Clearing encoder...\t', clear_counter()
    print 'Clearing status....\t', clear_status()

    spi.xfer2([WRITE_MODE0])
    spi.xfer2([FOURX_COUNT])

    time.sleep(.1)

    spi.xfer2([WRITE_MODE1])
    spi.xfer2([FOURX_COUNT])


def clear_counter():
    spi.xfer2([CLEAR_COUNTER])
    return '[DONE]'


def clear_status():
    spi.xfer2([CLEAR_STATUS])
    return '[DONE]'


def get_number(data):
    print data
    bytes_tmp = list(reversed(data))
    return struct.unpack('@l', "".join(map(chr, bytes_tmp)))[0]


def read_counter():
    data = [0, 0, 0, 0, 0]

    data = spi.xfer2([READ_COUNTER, 0, 0, 0, 0])
    encoder_count = 0
    for i in range(counterSize):
        encoder_count = encoder_count*256 + data[i+1]

    # encoder_count = get_number(data[1:5])
    return encoder_count


def read_status():
    # data = []
    spi.xfer2([READ_STATUS])
    data = spi.xfer2([0x00])
    return data


if __name__ == '__main__':
    init_encoder()

    try:
        while True:
            EncoderCount = read_counter()
            stdout.write("\rCount is [%s] status is %s.     " % (str(EncoderCount), str(read_status())))
            stdout.flush()
    except KeyboardInterrupt:
        print 'Thanks for Counting! :)'
        spi.close()


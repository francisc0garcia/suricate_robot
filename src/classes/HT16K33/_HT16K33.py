#!/bin/env python
import time
from matrix_symbols import MatrixSymbols
import rospy

class Device(object):
    DISPLAY_ADDRESS=0x80
    BRIGHTNESS_ADDRESS=0xE0
    OSCILLATOR=0x21

  # Constants
    DEFAULT_ADDRESS             = 0x70
    HT16K33_BLINK_CMD           = 0x80
    HT16K33_BLINK_DISPLAYON     = 0x01
    HT16K33_BLINK_OFF           = 0x00
    HT16K33_BLINK_2HZ           = 0x02
    HT16K33_BLINK_1HZ           = 0x04
    HT16K33_BLINK_HALFHZ        = 0x06
    HT16K33_SYSTEM_SETUP        = 0x20
    HT16K33_OSCILLATOR          = 0x01
    HT16K33_CMD_BRIGHTNESS      = 0xE0

    def __init__(self, address, bus):
        self.address = address
        self.bus = bus
        self.delay_transactions = 0.001
        self.symbols = MatrixSymbols()
        self.buffer = bytearray([0]*16)

    def clear(self):
        '''
         Loop through all data addresses, and clear any LEDS

         Example:
         >>> bus = Base()
         >>> bus.clear()  # doctest: +ELLIPSIS
         <...Base object at 0x...>
        '''
        for i in range(0x10):
          self.bus.write_byte_data(self.address, i, 0x00)
          time.sleep(self.delay_transactions)

        return self

    def setBrightness(self, brightness=0x0F):
        '''
         Set brightness level
         - brightness (0..15)
         --  0 =  1/16 duty
         --  1 =  2/16 duty
         --  2 =  3/16 duty
         --  3 =  4/16 duty
         --  4 =  5/16 duty
         --  5 =  6/16 duty
         --  6 =  7/16 duty
         --  7 =  8/16 duty
         --  8 =  9/16 duty
         --  9 = 10/16 duty
         -- 10 = 11/16 duty
         -- 11 = 12/16 duty
         -- 12 = 13/16 duty
         -- 13 = 14/16 duty
         -- 14 = 15/16 duty
         -- 15 = 16/16 duty

         Example:
         >>> bus = Base()
         >>> bus.setBrightness(15) # doctest: +ELLIPSIS
         <...Base object at 0x...>
        '''
        brightness = int(brightness) % 0x10
        self.bus.write_byte(self.address, self.BRIGHTNESS_ADDRESS | brightness )
        time.sleep(self.delay_transactions)
        return self

    def setDisplay(self, on=True, blink_rate=0x00):
        '''
         Set display options
         - on (Boolean)
         - blink_rate (0..3)
         -- 0 = Blink off
         -- 1 = 2HZ
         -- 2 = 1HZ
         -- 3 = 0.5HZ

         Example:
         >>> bus = Base()
         >>> bus.setDisplay(True, 4) #doctest: +ELLIPSIS
         <...Base object at 0x...>
        '''
        blink_rate = int(blink_rate) % 0x04
        on = int(on) % 0x02
        self.bus.write_byte( self.address, self.DISPLAY_ADDRESS | (blink_rate << 0x01) | on )
        time.sleep(self.delay_transactions)
        return self

    def setUp(self):
        '''
        Clear & set default state of HT16K33 internal systems
        KeyWords:
        - display_on (Boolean, default True)
        - blink_rate (0x00..0x03, default 0x00)
        - brightness (0x00..0x0F, default 0x07)

        Example:
        >>> bus = Base().setUp()
        '''
        self.clear() # Clear out manufacturer's test message
        self.turnOnOscillator() # Start internal oscillator
        self.setDisplay(on=True, blink_rate=0x00)
        self.setBrightness(brightness=0x07)
        return self

    def turnOnOscillator(self):
        '''
         Enable HT16K33 internal system oscillator

         Example:
         >>> bus = Base()
         >>> bus.turnOnOscillator() # doctest: +ELLIPSIS
         <...Base object at 0x...>
        '''
        self.bus.write_byte(self.address, self.OSCILLATOR)
        time.sleep(self.delay_transactions)
        return self

    def turnOffOscillator(self):
        '''
         Disable HT16K33 internal system oscillator

         Example:
         >>> bus = Base()
         >>> bus.turnOffOscillator()  # doctest: +ELLIPSIS
         <...Base object at 0x...>
        '''
        self.bus.write_byte(self.address, self.OSCILLATOR^0x01)
        time.sleep(self.delay_transactions)
        return self

    def clear_buffer(self):
        """Clear contents of display buffer."""
        for i, value in enumerate(self.buffer):
            self.buffer[i] = 0

    def set_pixel(self, x, y, value):
        """Set pixel at position x, y to the given value.  X and Y should be values
        of 0 to 8.  Value should be 0 for off and non-zero for on.
        """
        if x < 0 or x > 7 or y < 0 or y > 7:
            # Ignore out of bounds pixels.
            return

        self.set_led(y * 16 + ((x + 7) % 8), value)

    def set_led(self, led, value):
        """Sets specified LED (value of 0 to 127) to the specified value, 0/False
        for off and 1 (or any True/non-zero value) for on.
        """
        if led < 0 or led > 127:
            raise ValueError('LED must be value of 0 to 127.')

        # Calculate position in byte buffer and bit offset of desired LED.
        pos = led // 8
        offset = led % 8

        if not value:
            # Turn off the specified LED (set bit to zero).
            self.buffer[pos] &= ~(1 << offset)
        else:
            # Turn on the specified LED (set bit to one).
            self.buffer[pos] |= (1 << offset)

    def write_display(self):
        """Write display buffer to display hardware."""
        for i, value in enumerate(self.buffer):
            self.bus.write_byte_data(self.address, i, value)
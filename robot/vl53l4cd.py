# MicroPython driver for the VL53L4CD ToF sensor, needs to be saved on the Pico
# register values and start up steps come from ST's driver, via Adafruit's MIT licensed port:
# https://github.com/adafruit/Adafruit_CircuitPython_VL53L4CD
import time

REG_I2C_ADDRESS = 0x0001
REG_OSC_FREQ = 0x0006
REG_VHV_CONFIG = 0x0008
REG_VHV_START = 0x000B
REG_VHV_DECIMATION = 0x0024
REG_DEFAULT_CONFIG = 0x002D
REG_INTERRUPT_POLARITY = 0x0030
REG_DATA_READY = 0x0031
REG_RANGE_CONFIG_A = 0x005E
REG_RANGE_CONFIG_B = 0x0061
REG_INTERMEASUREMENT = 0x006C
REG_INTERRUPT_CLEAR = 0x0086
REG_SYSTEM_START = 0x0087
REG_RANGE_STATUS = 0x0089
REG_DISTANCE = 0x0096
REG_BOOT_STATUS = 0x00E5

START_CONTINUOUS = 0x21
START_AUTONOMOUS = 0x40
STOP = 0x00
RAW_STATUS_VALID = 9
TIMEOUT_MS = 1000

# the settings for registers 0x2D to 0x87, written all at once on start up
DEFAULT_CONFIG = (
    b"\x12\x00\x00\x11\x02\x00\x02\x08\x00\x08\x10\x01\x01\x00\x00\x00"
    b"\x00\xff\x00\x0f\x00\x00\x00\x00\x00\x20\x0b\x00\x00\x02\x14\x21"
    b"\x00\x00\x05\x00\x00\x00\x00\xc8\x00\x00\x38\xff\x01\x00\x08\x00"
    b"\x00\x01\xcc\x07\x01\xf1\x05\x00\xa0\x00\x80\x08\x38\x00\x00\x00"
    b"\x00\x0f\x89\x00\x00\x00\x00\x00\x00\x00\x01\x07\x05\x06\x06\x00"
    b"\x00\x02\xc7\xff\x9b\x00\x00\x00\x01\x00\x00"
)


def set_address(i2c, new_address, address=0x29):
    # moves a powered up sensor to its own address, lost again when it powers off
    i2c.writeto_mem(address, REG_I2C_ADDRESS, bytes([new_address & 0x7F]), addrsize=16)


class VL53L4CD:
    # timing_budget_ms is how long each reading takes, 10 to 200, longer is less noisy
    def __init__(self, i2c, address=0x29, timing_budget_ms=50):
        self.i2c = i2c
        self.address = address
        self.init_sensor()
        self.set_timing_budget(timing_budget_ms)
        self.write(REG_SYSTEM_START, bytes([START_CONTINUOUS]))
        self.wait_for_data()
        self.write(REG_INTERRUPT_CLEAR, b"\x01")

    def write(self, register, data):
        self.i2c.writeto_mem(self.address, register, data, addrsize=16)

    def read_bytes(self, register, length):
        return self.i2c.readfrom_mem(self.address, register, length, addrsize=16)

    def read_u16(self, register):
        data = self.read_bytes(register, 2)
        return (data[0] << 8) | data[1]

    def wait_for(self, is_ready, what):
        start = time.ticks_ms()
        while not is_ready():
            if time.ticks_diff(time.ticks_ms(), start) > TIMEOUT_MS:
                raise OSError("VL53L4CD timed out waiting for " + what)
            time.sleep_ms(1)

    def data_is_ready(self):
        # the ready bit is 0 when the interrupt is set to active low
        active_low = self.read_bytes(REG_INTERRUPT_POLARITY, 1)[0] & 0x10
        ready_bit = self.read_bytes(REG_DATA_READY, 1)[0] & 0x01
        return ready_bit == (0 if active_low else 1)

    def wait_for_data(self):
        self.wait_for(self.data_is_ready, "a reading")

    def init_sensor(self):
        self.wait_for(lambda: self.read_bytes(REG_BOOT_STATUS, 1)[0] == 0x03, "boot")
        self.write(REG_DEFAULT_CONFIG, DEFAULT_CONFIG)
        # one throwaway reading calibrates the sensor
        self.write(REG_SYSTEM_START, bytes([START_AUTONOMOUS]))
        self.wait_for_data()
        self.write(REG_INTERRUPT_CLEAR, b"\x01")
        self.write(REG_SYSTEM_START, bytes([STOP]))
        self.write(REG_VHV_CONFIG, b"\x09")
        self.write(REG_VHV_START, b"\x00")
        self.write(REG_VHV_DECIMATION, b"\x05\x00")
        # 0 means take readings back to back
        self.write(REG_INTERMEASUREMENT, b"\x00\x00\x00\x00")

    def range_config(self, budget_us, macro_period_us, vcsel):
        # the sensor stores each time as an 8 bit value and a shift
        step = (macro_period_us * vcsel) >> 6
        value = ((budget_us << 12) + (step >> 1)) // step - 1
        shift = 0
        while value >> 8:
            value >>= 1
            shift += 1
        return bytes([shift, value & 0xFF])

    def set_timing_budget(self, timing_budget_ms):
        if not 10 <= timing_budget_ms <= 200:
            raise ValueError("timing budget must be 10 to 200 ms")
        osc_freq = self.read_u16(REG_OSC_FREQ)
        macro_period_us = (2304 * (0x40000000 // osc_freq)) >> 6
        budget_us = timing_budget_ms * 1000 - 2500
        self.write(REG_RANGE_CONFIG_A, self.range_config(budget_us, macro_period_us, 16))
        self.write(REG_RANGE_CONFIG_B, self.range_config(budget_us, macro_period_us, 12))

    def read(self):
        # returns the distance in mm, or None if there was no trustworthy reading
        self.wait_for_data()
        status = self.read_bytes(REG_RANGE_STATUS, 1)[0] & 0x1F
        distance = self.read_u16(REG_DISTANCE)
        self.write(REG_INTERRUPT_CLEAR, b"\x01")
        if status != RAW_STATUS_VALID:
            return None
        return distance

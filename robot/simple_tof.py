# Reads ONE VL53L4CD ToF sensor and prints the distance in mm.
# Needs vl53l4cd.py saved on the Pico too.
# Change WHICH to "left" or "right" to test the other sensors.
from machine import Pin, I2C
import time
from vl53l4cd import VL53L4CD

WHICH = "front"
XSHUT_PINS = {"front": 10, "left": 11, "right": 12}

# Turn every sensor off, then turn on only the one we want.
# (They all share address 0x29, so only one can be on at a time here.)
for name, pin in XSHUT_PINS.items():
    Pin(pin, Pin.OUT, value=0)
time.sleep_ms(50)
Pin(XSHUT_PINS[WHICH], Pin.OUT, value=1)
time.sleep_ms(50)

i2c = I2C(0, sda=Pin(0), scl=Pin(1), freq=400_000)
print("I2C devices found:", [hex(a) for a in i2c.scan()])  # expect ['0x29']

sensor = VL53L4CD(i2c)

while True:
    print(WHICH, "distance (mm):", sensor.read())  # None = nothing in range
    time.sleep_ms(200)

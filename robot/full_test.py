# Tests everything: the 3 ToF sensors, then both motors and encoders, then both together.
# Needs vl53l4cd.py on the Pico too. Wheels OFF the ground, supply on.
#
# Wiring (physical pin numbers in brackets):
#   ToF (shared): SDA GP0 (1)   SCL GP1 (2)   VIN 3.3 V   GND
#   ToF XSHUT:    front GP10 (14)   left GP11 (15)   right GP12 (16)
#   Motor A: AIN1 GP2 (4)   AIN2 GP3 (5)    PWMA GP5 (7)    encoder A GP8 (11), B GP9 (12)
#   Motor B: BIN1 GP6 (9)   BIN2 GP7 (10)   PWMB GP4 (6)    encoder A GP14 (19), B GP15 (20)
#   Driver STBY, VCC -> 3.3 V
# The motor connector labels are printed in REVERSE, see tests/both_motors.py.
from machine import Pin, I2C
import time
from vl53l4cd import VL53L4CD, set_address

RUN_MS = 1500


# ---------- ToF sensors ----------

SENSORS = (
    # name, XSHUT pin, new address
    ("front", 10, 0x30),
    ("left", 11, 0x31),
    ("right", 12, 0x32),
)

i2c = I2C(0, sda=Pin(0), scl=Pin(1), freq=400_000)


def start_sensors():
    # all off, then switch each on in turn and give it its own address
    xshuts = [Pin(pin, Pin.OUT, value=0) for name, pin, address in SENSORS]
    time.sleep_ms(50)
    if i2c.scan():
        print("FAIL ToF - something answered with all sensors off, check the XSHUT wires")
    sensors = []
    for (name, pin, address), xshut in zip(SENSORS, xshuts):
        xshut.value(1)
        time.sleep_ms(50)
        if 0x29 not in i2c.scan():
            print("FAIL ToF", name, "- not found, check its XSHUT wire (and shared VIN/GND/SDA/SCL)")
            xshut.value(0)
            continue
        try:
            set_address(i2c, address)
            time.sleep_ms(10)
            sensors.append((name, VL53L4CD(i2c, address=address)))
            print("PASS ToF", name, "at", hex(address))
        except Exception as e:
            print("FAIL ToF", name, "-", e)
            xshut.value(0)
    return sensors


def distances(sensors):
    return "  ".join(name + ": " + str(s.read()) + "mm" for name, s in sensors)


# ---------- motors ----------

class Motor:
    def __init__(self, name, in1, in2, pwm, enc_a, enc_b):
        self.name = name
        self.in1 = Pin(in1, Pin.OUT, value=0)
        self.in2 = Pin(in2, Pin.OUT, value=0)
        self.pwm = Pin(pwm, Pin.OUT, value=0)   # on/off, see tests/both_motors.py
        self.ticks = 0
        self.enc_a = Pin(enc_a, Pin.IN, Pin.PULL_UP)
        self.enc_b = Pin(enc_b, Pin.IN, Pin.PULL_UP)
        self.enc_a.irq(trigger=Pin.IRQ_RISING | Pin.IRQ_FALLING, handler=self.on_edge)

    def on_edge(self, pin):
        if self.enc_a.value() == self.enc_b.value():
            self.ticks += 1
        else:
            self.ticks -= 1

    def run(self, forward):
        self.in1.value(1 if forward else 0)
        self.in2.value(0 if forward else 1)
        self.pwm.value(1)

    def stop(self):
        self.pwm.value(0)
        self.in1.value(0)
        self.in2.value(0)


motor_a = Motor("A", 2, 3, 5, 8, 9)
motor_b = Motor("B", 6, 7, 4, 14, 15)
motors = (motor_a, motor_b)


def drive(group, label, forward):
    print(label)
    for m in group:
        m.ticks = 0
        m.run(forward)
    time.sleep_ms(RUN_MS)
    for m in group:
        m.stop()
    for m in group:
        result = "PASS" if m.ticks != 0 else "FAIL (no ticks, check encoder wires)"
        print("  motor", m.name, "ticks:", m.ticks, result)
    time.sleep_ms(500)


# ---------- run everything ----------

try:
    print("=== 1. ToF sensors ===")
    sensors = start_sensors()
    print(len(sensors), "/ 3 sensors working")
    if sensors:
        print("Move your hand in front of each sensor:")
        for i in range(12):
            print(" ", distances(sensors))
            time.sleep_ms(250)

    print("=== 2. Motors (wheels off the ground!) starting in 3 s ===")
    time.sleep(3)
    for m in motors:
        drive([m], "motor " + m.name + " FORWARD", True)
        drive([m], "motor " + m.name + " BACKWARD", False)

    print("=== 3. Both motors + sensors together ===")
    for m in motors:
        m.ticks = 0
        m.run(True)
    for i in range(8):
        line = "A ticks: " + str(motor_a.ticks) + "  B ticks: " + str(motor_b.ticks)
        if sensors:
            line += "  " + distances(sensors)
        print(" ", line)
        time.sleep_ms(250)
finally:
    for m in motors:
        m.stop()
    print("=== done, motors stopped ===")

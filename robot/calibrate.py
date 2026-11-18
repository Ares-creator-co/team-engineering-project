# calibrate.py
# Works out how many encoder ticks = 1 mm, and how many ticks = a full spin.
# We need these to drive exactly one cell and turn exactly 90 degrees.
#
# MODE = "straight": drives straight for DRIVE_TICKS then stops. Measure how far it went.
#   ticks per mm = average ticks / distance in mm
# MODE = "spin": spins on the spot for SPIN_TICKS then stops. Measure how far it turned.
#   ticks per degree = average ticks / degrees turned
#
# Put on the Pico as main.py (put drive_straight.py back as main.py after!).
# Turn the battery on and it goes straight away. Results go in calib.txt.
# Doesn't drive with USB plugged in, it just prints the last result.
from machine import Pin, PWM
import time

MODE = "straight"
DRIVE_TICKS = 2500     # about 1 m if our guess of 2.6 ticks/mm is right
SPIN_TICKS = 580       # about one full spin if our guess is right
SPEED = 300            # ticks/s, slow so it doesn't skid
START_POWER_LEFT = 0.42
START_POWER_RIGHT = 0.22
SPEED_GAIN = 0.00008
SYNC = 3               # how hard it keeps the two wheels level (no walls to steer off here)


class Wheel:
    def __init__(self, in1, in2, enable, enc_a, reverse):
        self.a = PWM(Pin(in1))
        self.b = PWM(Pin(in2))
        self.a.freq(1000)
        self.b.freq(1000)
        self.a.duty_u16(0)
        self.b.duty_u16(0)
        Pin(enable, Pin.OUT, value=1)
        self.reverse = reverse
        self.ticks = 0
        self.enc = Pin(enc_a, Pin.IN, Pin.PULL_UP)
        self.enc.irq(trigger=Pin.IRQ_RISING | Pin.IRQ_FALLING, handler=self.count)

    def count(self, pin):
        self.ticks += 1

    def power(self, p):
        duty = int(min(1.0, abs(p)) * 65535)
        if self.reverse:
            forwards = p < 0
        else:
            forwards = p >= 0
        if forwards:
            self.b.duty_u16(0)
            self.a.duty_u16(duty)
        else:
            self.a.duty_u16(0)
            self.b.duty_u16(duty)

    def stop(self):
        self.a.duty_u16(0)
        self.b.duty_u16(0)


# same speed control as drive_straight
class Speed:
    def __init__(self, wheel, power):
        self.wheel = wheel
        self.power = power
        self.last_ticks = wheel.ticks
        self.last_time = time.ticks_ms()

    def drive(self, target):
        now = time.ticks_ms()
        dt = time.ticks_diff(now, self.last_time)
        if dt >= 50:
            speed = (self.wheel.ticks - self.last_ticks) * 1000 // dt
            self.last_ticks = self.wheel.ticks
            self.last_time = now
            change = SPEED_GAIN * (abs(target) - speed)
            if change > 0.05:
                change = 0.05
            if change < -0.05:
                change = -0.05
            self.power += change
            if self.power > 1.0:
                self.power = 1.0
            if self.power < 0.05:
                self.power = 0.05
        if target >= 0:
            self.wheel.power(self.power)
        else:
            self.wheel.power(-self.power)


left = Wheel(2, 3, 5, 8, True)
right = Wheel(6, 7, 4, 14, True)


def stop():
    left.stop()
    right.stop()


def run(spin):
    left_speed = Speed(left, START_POWER_LEFT)
    right_speed = Speed(right, START_POWER_RIGHT)
    if spin:
        goal = SPIN_TICKS
    else:
        goal = DRIVE_TICKS
    start = time.ticks_ms()
    while (left.ticks + right.ticks) // 2 < goal:
        if time.ticks_diff(time.ticks_ms(), start) > 20000:
            break   # something's wrong, don't drive forever
        # if one wheel has done more ticks, slow it down and speed the other up
        diff = left.ticks - right.ticks
        l_target = SPEED - SYNC * diff
        r_target = SPEED + SYNC * diff
        if spin:
            left_speed.drive(-l_target)   # left backwards, right forwards = spin left
        else:
            left_speed.drive(l_target)
        right_speed.drive(r_target)
    stop()
    time.sleep_ms(500)   # let it coast to a stop, the coast ticks count too
    secs = time.ticks_diff(time.ticks_ms(), start) / 1000
    avg = (left.ticks + right.ticks) / 2
    out = []
    out.append("mode: " + MODE)
    out.append("ticks: left %d  right %d  average %d" % (left.ticks, right.ticks, avg))
    out.append("took %.1f s" % secs)
    if spin:
        out.append("measure how many degrees it turned, then ticks per degree = %d / degrees" % avg)
    else:
        out.append("measure how far it went in mm, then ticks per mm = %d / mm" % avg)
    return out


def usb_plugged_in():
    try:
        return Pin("WL_GPIO2", Pin.IN).value() == 1
    except Exception:
        return False


if usb_plugged_in():
    print("USB plugged in, not driving. Last result:")
    try:
        print(open("calib.txt").read())
    except OSError:
        print("(no calib.txt yet)")
    raise SystemExit

time.sleep_ms(500)
try:
    result = run(MODE == "spin")
except Exception as e:
    result = ["ERROR: " + str(e)]
finally:
    stop()
f = open("calib.txt", "w")
f.write("\n".join(result) + "\n")
f.close()

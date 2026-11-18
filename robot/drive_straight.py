# drive_straight.py
# Drives through the maze staying in the middle of the corridor. When it gets to a wall
# it turns (left if it can, then right, otherwise turns around) and keeps going.
#
# Save on the Pico as main.py with vl53l4cd.py. Put it in a corridor and turn it on.
# It doesn't drive when USB is plugged in so you can read last_run.txt after a run.
from machine import Pin, PWM, I2C
import time
from vl53l4cd import VL53L4CD, set_address

# settings
TARGET_SPEED = 400       # ticks per second for each wheel
START_POWER_LEFT = 0.22  # what the motors start at, speed control changes it after
START_POWER_RIGHT = 0.16
SPEED_GAIN = 0.00008

# the side sensors are angled 30 deg forward so these are in "sensor mm" not real sideways mm
STEER_P = 0.002          # turn up if it drifts into a wall
STEER_D = 0.0006         # turn up if it wobbles side to side
MAX_STEER = 0.35
SMOOTH = 0.5
SIDE_GUESS_MM = 100      # only used if the start readings are wrong
WALL_FACTOR = 1.6        # side reading less than centred * this = wall there
SIDES_BLIND_MM = 120     # side sensors can see the front wall when it's closer than this
TURN_AT_MM = 50          # how close to the front wall before turning (was 40)

TURN_SPEED = 250
TURN_CLEAR_MM = 150      # front reading more than this = open cell
TURN_MAX_WINDOW = 200
TURN_TIMEOUT_MS = 4000
MAX_SECONDS = 60

REVERSE_LEFT = True
REVERSE_RIGHT = True
DRIVE_ON_USB = False

led = Pin("LED", Pin.OUT, value=0)
lines = []


def log(*parts):
    line = " ".join(str(p) for p in parts)
    print(line)
    lines.append(line)


def save_log(extra=()):
    try:
        f = open("last_run.txt", "w")
        f.write("\n".join(lines) + "\n")
        if extra:
            f.write("(saved mid-run, trace so far:)\n" + "\n".join(extra) + "\n")
        f.close()
    except Exception as e:
        print("could not save log:", e)


# motors
class Wheel:
    def __init__(self, in1, in2, enable, enc_a, enc_b, reverse):
        self.a = PWM(Pin(in1))
        self.b = PWM(Pin(in2))
        self.a.freq(1000)
        self.b.freq(1000)
        self.a.duty_u16(0)
        self.b.duty_u16(0)
        Pin(enable, Pin.OUT, value=1)   # PWMA/PWMB always on, we PWM the IN pins instead
        self.reverse = reverse
        self.ticks = 0
        self.enc_a = Pin(enc_a, Pin.IN, Pin.PULL_UP)
        self.enc_b = Pin(enc_b, Pin.IN, Pin.PULL_UP)
        self.enc_a.irq(trigger=Pin.IRQ_RISING | Pin.IRQ_FALLING, handler=self.count)

    def count(self, pin):
        self.ticks += 1

    def power(self, p):
        # p goes from -1 to 1, minus = backwards
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


left_wheel = Wheel(2, 3, 5, 8, 9, REVERSE_LEFT)
right_wheel = Wheel(6, 7, 4, 14, 15, REVERSE_RIGHT)


def stop():
    left_wheel.stop()
    right_wheel.stop()


# speed control for one wheel - every 50ms check the speed and change the power a bit.
# our motors aren't the same so each wheel needs its own
class Speed:
    def __init__(self, wheel, power):
        self.wheel = wheel
        self.power = power
        self.speed = 0
        self.reset()

    def reset(self):
        self.last_ticks = self.wheel.ticks
        self.last_time = time.ticks_ms()

    def drive(self, target):
        now = time.ticks_ms()
        dt = time.ticks_diff(now, self.last_time)
        if dt >= 50:
            self.speed = (self.wheel.ticks - self.last_ticks) * 1000 // dt
            self.last_ticks = self.wheel.ticks
            self.last_time = now
            change = SPEED_GAIN * (abs(target) - self.speed)
            if change > 0.05:
                change = 0.05
            if change < -0.05:
                change = -0.05
            self.power += change
            if self.power > 1.0:
                self.power = 1.0
            if self.power < 0.05:
                self.power = 0.05
        # encoders can't tell direction so for backwards just flip the motor
        if target >= 0:
            self.wheel.power(self.power)
        else:
            self.wheel.power(-self.power)


left_speed = Speed(left_wheel, START_POWER_LEFT)
right_speed = Speed(right_wheel, START_POWER_RIGHT)


# sensors
# front and left xshut wires got swapped when we soldered it
SENSORS = (("front", 11, 0x30), ("left", 10, 0x31), ("right", 12, 0x32))
i2c = I2C(0, sda=Pin(0), scl=Pin(1), freq=400_000)
xshut = {}
for name, pin, addr in SENSORS:
    xshut[name] = Pin(pin, Pin.OUT, value=0)
tof = {}


def start_sensors():
    # they all start as 0x29 so turn them on one by one and change the address
    time.sleep_ms(50)
    for name, pin, addr in SENSORS:
        for attempt in range(3):
            xshut[name].value(0)
            time.sleep_ms(20)
            xshut[name].value(1)
            time.sleep_ms(50)
            try:
                set_address(i2c, addr)
                time.sleep_ms(10)
                tof[name] = VL53L4CD(i2c, address=addr, timing_budget_ms=15)
                break
            except Exception as e:
                log(name, "sensor attempt", attempt + 1, "failed:", e)
        if name in tof:
            log(name, "sensor OK")
        else:
            log(name, "sensor FAILED")


last_good = {}


def distance(name):
    if name not in tof:
        return 9999
    try:
        d = tof[name].read()
    except OSError:
        # i2c sometimes gives EIO, just use the last reading
        if name in last_good:
            return last_good[name]
        return 9999
    if d is None:
        d = 9999
    last_good[name] = d
    return d


def off_centre(f, l, r, centred):
    # + means too far right so steer left
    if f < SIDES_BLIND_MM:
        return None, "front wall near"
    wall = centred * WALL_FACTOR
    if l < wall and r < wall:
        return (l - r) / 2, "both walls"
    if l < wall:
        return l - centred, "left wall"
    if r < wall:
        return centred - r, "right wall"
    return None, "no walls"


# turning
def spin(d, until, timeout_ms, watch=None):
    # spin on the spot. d=1 left, d=-1 right
    # angle = average ticks of the 2 wheels. stops when it gets to until or watch returns True
    l0 = left_wheel.ticks
    r0 = right_wheel.ticks
    left_speed.reset()
    right_speed.reset()
    t0 = time.ticks_ms()
    angle = 0
    while time.ticks_diff(time.ticks_ms(), t0) < timeout_ms:
        left_speed.drive(-TURN_SPEED * d)
        right_speed.drive(TURN_SPEED * d)
        angle = ((left_wheel.ticks - l0) + (right_wheel.ticks - r0)) // 2
        if until is not None and angle >= until:
            break
        if watch is not None and watch(angle, distance("front")):
            break
    stop()
    time.sleep_ms(80)
    return ((left_wheel.ticks - l0) + (right_wheel.ticks - r0)) // 2


def turn(direction):
    # Spin until the front sensor reads long (looking into an open cell), keep going until
    # it reads short again, then turn back half way. That's facing straight down the gap.
    s = {"found": None, "lost": None, "long": 0, "short": 0, "best": 0, "best_at": 0}

    def watch(angle, f):
        if f > s["best"] and f < 2000:
            s["best"] = f
            s["best_at"] = angle
        # 2 in a row so one dodgy reading doesn't mess it up
        if f > TURN_CLEAR_MM:
            s["long"] += 1
            s["short"] = 0
        else:
            s["long"] = 0
            s["short"] += 1
        if s["found"] is None:
            if s["long"] >= 2 and angle > 20:
                s["found"] = angle
        elif s["short"] >= 2 or angle - s["found"] > TURN_MAX_WINDOW:
            s["lost"] = angle
            return True
        return False

    total = spin(direction, None, TURN_TIMEOUT_MS, watch)
    found = s["found"]
    lost = s["lost"]
    if direction > 0:
        name = "left"
    else:
        name = "right"

    if found is None:
        # never found a gap so go back to the longest reading
        back = total - s["best_at"]
        if back > 0:
            spin(-direction, back, 3000)
        return "%s: no gap in %d ticks, went back to longest reading (%d mm), front now %d" % (
            name, total, s["best"], distance("front"))

    if lost is None:
        lost = total
    back = total - (found + lost) // 2
    if back > 0:
        spin(-direction, back, 1500)
    return "%s: gap from %d to %d ticks, ended at %d, turned back %d, front now %d" % (
        name, found, lost, total, back, distance("front"))


def edge(d, limit):
    # spin until the front goes from open to wall
    s = {"short": 0, "seen": False}

    def watch(angle, f):
        if f >= TURN_CLEAR_MM:
            s["seen"] = True
            s["short"] = 0
        elif s["seen"]:
            s["short"] += 1
        return s["short"] >= 2

    return spin(d, limit, 3000, watch)


def line_up():
    # find the left and right edges of the corridor and face the middle
    # so it still works if we put it down crooked
    if distance("front") < TURN_CLEAR_MM:
        return "facing a wall, turning to find a corridor: " + turn(1)
    a = edge(1, 150)
    b = edge(-1, a + 300)
    spin(1, b // 2, 2000)
    return "edges %d left, %d right, went back %d, front now %d" % (a, b, b // 2, distance("front"))


def drive():
    log("Line up:", line_up())
    f = distance("front")
    l = distance("left")
    r = distance("right")
    log("Start: front %d  left %d  right %d mm" % (f, l, r))
    # walls on both sides at the start so the average = what it reads in the middle
    centred = (l + r) / 2
    if centred < 40 or centred > 200:
        log("Start side readings look wrong, using SIDE_GUESS_MM")
        centred = SIDE_GUESS_MM
    log("Centred side reading: %d mm" % centred)
    save_log()

    start = time.ticks_ms()
    last_time = start
    loop_time = start
    last_trace = start
    left_speed.reset()
    right_speed.reset()
    speeds = []
    closest_side = 9999
    trace = []
    last_mode = None
    last_err = None
    smooth_err = None
    rate = 0
    modes = {}
    close_count = 0
    turns = 0
    # when it's right at the wall the side sensors only see the front wall,
    # so check which sides are open a bit before and remember it
    open_left = None
    open_right = None

    while time.ticks_diff(time.ticks_ms(), start) < MAX_SECONDS * 1000:
        f = distance("front")
        l = distance("left")
        r = distance("right")
        closest_side = min(closest_side, l, r)
        if f >= SIDES_BLIND_MM and f <= 280:
            open_left = l > centred * WALL_FACTOR
            open_right = r > centred * WALL_FACTOR

        # wall in front? needs 2 readings in a row
        if f < TURN_AT_MM:
            close_count += 1
        else:
            close_count = 0

        if close_count >= 2:
            stop()
            time.sleep_ms(80)
            if open_left or open_left is None:
                direction = 1
                if open_left:
                    why = "left open"
                else:
                    why = "not sure, try left"
            elif open_right:
                direction = -1
                why = "right open"
            else:
                direction = 1
                why = "dead end, turning round"
            turns += 1
            result = turn(direction)
            trace.append("TURN %d at %d ms: %s. %s" % (
                turns, time.ticks_diff(time.ticks_ms(), start), why, result))
            save_log(trace)   # save every turn incase we turn it off
            # reset everything for the new corridor
            close_count = 0
            open_left = None
            open_right = None
            last_mode = None
            last_err = None
            smooth_err = None
            rate = 0
            left_speed.reset()
            right_speed.reset()
            last_time = time.ticks_ms()
            loop_time = last_time
            continue

        now = time.ticks_ms()
        dt = time.ticks_diff(now, loop_time) / 1000
        loop_time = now

        if time.ticks_diff(now, last_time) >= 200:
            last_time = now
            speeds.append("%d/%d" % (left_speed.speed, right_speed.speed))

        # PD steering
        # P = how far from the middle, D = how fast its moving towards a wall (stops the wobble)
        err, mode = off_centre(f, l, r, centred)
        if mode in modes:
            modes[mode] += 1
        else:
            modes[mode] = 1
        if mode == "both walls":
            centred += 0.02 * ((l + r) / 2 - centred)   # keep updating the middle reading slowly
        if err is None or mode != last_mode or last_err is None or dt <= 0:
            rate = 0
            smooth_err = err
        else:
            smooth_err += SMOOTH * (err - smooth_err)
            new_rate = (smooth_err - last_err) / dt
            rate += SMOOTH * (new_rate - rate)
        last_mode = mode
        last_err = smooth_err

        if smooth_err is None:
            steer = 0
        else:
            steer = STEER_P * smooth_err + STEER_D * rate
        if steer > MAX_STEER:
            steer = MAX_STEER
        if steer < -MAX_STEER:
            steer = -MAX_STEER
        # one wheel faster one slower
        left_speed.drive(TARGET_SPEED * (1 - steer))
        right_speed.drive(TARGET_SPEED * (1 + steer))

        if time.ticks_diff(now, last_trace) >= 200:
            last_trace = now
            if smooth_err is None:
                e = "-"
            else:
                e = "%d" % smooth_err
            trace.append("%5d %4d %4d %4d %6s %6s %5.2f %4.2f %4.2f" % (
                time.ticks_diff(now, start), f, l, r, e, "%d" % rate, steer,
                left_speed.power, right_speed.power))

    stop()
    log("Stopped: time up after %d s, %d turns" % (MAX_SECONDS, turns))
    log("Ticks: left %d  right %d" % (left_wheel.ticks, right_wheel.ticks))
    if left_wheel.ticks < 50 or right_wheel.ticks < 50:
        if left_wheel.ticks < right_wheel.ticks:
            log("WARNING: LEFT wheel barely moved, check the motor/encoder")
        else:
            log("WARNING: RIGHT wheel barely moved, check the motor/encoder")
    log("Closest a side reading got: %d mm" % closest_side)
    log("Steering used:", modes)
    log("Wheel speeds L/R every 0.2 s:", speeds)
    log("Power at the end: left %.2f  right %.2f" % (left_speed.power, right_speed.power))
    log("Centred side reading at the end: %d mm" % centred)
    log("Trace (ms front left right off-centre rate steer powerL powerR), and turns:")
    for t in trace:
        log(t)


def usb_plugged_in():
    try:
        return Pin("WL_GPIO2", Pin.IN).value() == 1   # VBUS on the pico 2 w
    except Exception:
        return False


def keep_previous_log():
    import os
    try:
        if "last_run.txt" in os.listdir():
            os.rename("last_run.txt", "prev_run.txt")
    except Exception:
        pass


if usb_plugged_in() and not DRIVE_ON_USB:
    # plugged into laptop - just show sensors, don't drive
    start_sensors()
    print("USB connected, not driving. front %d  left %d  right %d mm"
          % (distance("front"), distance("left"), distance("right")))
    raise SystemExit

keep_previous_log()
try:
    log("drive_straight starting")
    time.sleep_ms(500)
    start_sensors()
    if "front" not in tof:
        raise RuntimeError("front sensor failed, not driving without it")
    led.value(1)
    drive()
    led.value(0)
except Exception as e:
    stop()
    log("ERROR:", e)
finally:
    stop()
    save_log()

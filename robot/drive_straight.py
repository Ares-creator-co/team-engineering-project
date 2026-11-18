# drive_straight.py - drives round the maze, keeping centred in each corridor.
# When it gets to a wall it turns (left if open, then right, otherwise turns round) and keeps going.
#
# Put it on the Pico as main.py. Place the robot in a corridor and switch the battery on,
# it starts straight away. LED is on while it's driving.
# It won't drive if USB is plugged in, so it's safe to plug in and read last_run.txt after a run.
# Needs vl53l4cd.py on the Pico too.
from machine import Pin, PWM, I2C
import time
from vl53l4cd import VL53L4CD, set_address

# ---- settings ----
TARGET_SPEED = 400       # encoder ticks/s for each wheel
START_POWER_LEFT = 0.22  # starting motor power, the speed control adjusts it from here
START_POWER_RIGHT = 0.16
SPEED_GAIN = 0.00008     # how quickly the power changes to fix the speed (too high = it hunts)

# side sensors point 30 deg out from the front so they look ahead a bit, which means
# left - right changes with both position and angle. numbers below are in sensor mm
STEER_P = 0.002          # steering per mm off centre. raise if it drifts
STEER_D = 0.0006         # damping, steering per mm/s. raise if it weaves
MAX_STEER = 0.35         # max difference between wheel speeds (35%)
SMOOTH = 0.5             # filter on the readings, lower = smoother but slower to react
SIDE_GUESS_MM = 100      # backup centred reading if the start readings are rubbish
WALL_FACTOR = 1.6        # side reading below centred x 1.6 counts as a wall
SIDES_BLIND_MM = 120     # if the front wall is closer than this the side sensors see it, so don't steer
TURN_AT_MM = 40          # turn when the front wall is this close

TURN_SPEED = 250         # wheel speed when turning on the spot
TURN_CLEAR_MM = 150      # front reading above this = looking into an open cell
TURN_MAX_WINDOW = 200    # stop looking for the end of the gap after this many ticks
TURN_TIMEOUT_MS = 4000
MAX_SECONDS = 60         # how long a run lasts

REVERSE_LEFT = True
REVERSE_RIGHT = True
DRIVE_ON_USB = False

# ---- logging ----
led = Pin("LED", Pin.OUT, value=0)
lines = []


def log(*parts):
    line = " ".join(str(p) for p in parts)
    print(line)
    lines.append(line)


def save_log(extra=()):
    try:
        with open("last_run.txt", "w") as f:
            f.write("\n".join(lines) + "\n")
            if extra:
                f.write("(saved mid-run, trace so far:)\n" + "\n".join(extra) + "\n")
    except Exception as e:
        print("could not save log:", e)


# ---- motors ----
class Wheel:
    def __init__(self, in1, in2, enable, enc_a, enc_b, reverse):
        self.a = PWM(Pin(in1))
        self.b = PWM(Pin(in2))
        for p in (self.a, self.b):
            p.freq(1000)
            p.duty_u16(0)
        Pin(enable, Pin.OUT, value=1)   # PWMA/PWMB just held high, speed is set on IN1/IN2
        self.reverse = reverse
        self.ticks = 0
        self.enc_a = Pin(enc_a, Pin.IN, Pin.PULL_UP)
        self.enc_b = Pin(enc_b, Pin.IN, Pin.PULL_UP)
        self.enc_a.irq(trigger=Pin.IRQ_RISING | Pin.IRQ_FALLING, handler=self.count)

    def count(self, pin):
        self.ticks += 1

    def power(self, p):
        # -1 to 1, negative is backwards
        duty = int(min(1.0, abs(p)) * 65535)
        forwards = (p >= 0) != self.reverse
        on, off = (self.a, self.b) if forwards else (self.b, self.a)
        off.duty_u16(0)
        on.duty_u16(duty)

    def stop(self):
        self.a.duty_u16(0)
        self.b.duty_u16(0)


left_wheel = Wheel(2, 3, 5, 8, 9, REVERSE_LEFT)
right_wheel = Wheel(6, 7, 4, 14, 15, REVERSE_RIGHT)


def stop():
    left_wheel.stop()
    right_wheel.stop()


class Speed:
    # keeps one wheel at a set speed. every 50ms it checks the ticks and nudges the power.
    # this is why it drives straight even though our motors aren't matched
    def __init__(self, wheel, power):
        self.wheel, self.power, self.speed = wheel, power, 0
        self.reset()

    def reset(self):
        self.last_ticks, self.last_time = self.wheel.ticks, time.ticks_ms()

    def drive(self, target):
        now = time.ticks_ms()
        dt = time.ticks_diff(now, self.last_time)
        if dt >= 50:
            self.speed = (self.wheel.ticks - self.last_ticks) * 1000 // dt
            self.last_ticks, self.last_time = self.wheel.ticks, now
            change = SPEED_GAIN * (abs(target) - self.speed)
            self.power += max(-0.05, min(0.05, change))
            self.power = max(0.05, min(1.0, self.power))
        # encoders only count, they don't know direction, so a negative target just flips the motor
        self.wheel.power(self.power if target >= 0 else -self.power)


left_speed = Speed(left_wheel, START_POWER_LEFT)
right_speed = Speed(right_wheel, START_POWER_RIGHT)


# ---- sensors ----
# front and left XSHUT got swapped when soldering, so front is GP11 and left is GP10
SENSORS = (("front", 11, 0x30), ("left", 10, 0x31), ("right", 12, 0x32))
i2c = I2C(0, sda=Pin(0), scl=Pin(1), freq=400_000)
xshut = {name: Pin(pin, Pin.OUT, value=0) for name, pin, addr in SENSORS}
tof = {}


def start_sensors():
    # all three start on address 0x29, so turn them on one at a time and give each a new address
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
        log(name, "sensor", "OK" if name in tof else "FAILED")


last_good = {}


def distance(name):
    if name not in tof:
        return 9999
    try:
        d = tof[name].read()
    except OSError:
        # the I2C bus glitches now and then (EIO), just reuse the last reading
        return last_good.get(name, 9999)
    d = 9999 if d is None else d
    last_good[name] = d
    return d


# ---- steering ----
def off_centre(f, l, r, centred):
    # positive = too far right, so steer left
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


# ---- turning ----
def spin(d, until, timeout_ms, watch=None):
    # turn on the spot, d = 1 for left, -1 for right.
    # angle is the average ticks of both wheels. stops at `until` or when watch() says so
    l0, r0 = left_wheel.ticks, right_wheel.ticks
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
    # Turns until the front sensor sees into an open cell (reading goes long), keeps turning until
    # it sees a wall again, then turns back to halfway between the two. That puts it facing
    # straight down the corridor without needing to know how many ticks a 90 is.
    w = {"found": None, "lost": None, "long": 0, "short": 0, "best": 0, "best_at": 0}

    def watch(angle, f):
        if f > w["best"] and f < 2000:
            w["best"], w["best_at"] = f, angle
        # need 2 readings in a row so one bad reading doesn't count
        if f > TURN_CLEAR_MM:
            w["long"], w["short"] = w["long"] + 1, 0
        else:
            w["long"], w["short"] = 0, w["short"] + 1
        if w["found"] is None:
            if w["long"] >= 2 and angle > 20:
                w["found"] = angle
        elif w["short"] >= 2 or angle - w["found"] > TURN_MAX_WINDOW:
            w["lost"] = angle
            return True
        return False

    total = spin(direction, None, TURN_TIMEOUT_MS, watch)
    found, lost = w["found"], w["lost"]
    name = "left" if direction > 0 else "right"
    if found is None:
        # didn't find a gap, go back to wherever the front reading was longest
        back = total - w["best_at"]
        if back > 0:
            spin(-direction, back, 3000)
        return "%s: no gap in %d ticks, went back to longest reading (%d mm), front now %d" % (
            name, total, w["best"], distance("front"))
    if lost is None:
        lost = total
    back = total - (found + lost) // 2
    if back > 0:
        spin(-direction, back, 1500)
    return "%s: gap from %d to %d ticks, ended at %d, turned back %d, front now %d" % (
        name, found, lost, total, back, distance("front"))


def edge(d, limit):
    # turn until the front sensor goes from seeing the corridor to seeing a wall
    w = {"short": 0, "seen": False}

    def watch(angle, f):
        if f >= TURN_CLEAR_MM:
            w["seen"], w["short"] = True, 0
        elif w["seen"]:
            w["short"] += 1
        return w["short"] >= 2
    return spin(d, limit, 3000, watch)


def line_up():
    # at the start, find both edges of the corridor and point down the middle,
    # so it doesn't matter if we put it down a bit crooked
    if distance("front") < TURN_CLEAR_MM:
        return "facing a wall, turning to find a corridor: " + turn(1)
    a = edge(1, 150)
    b = edge(-1, a + 300)
    spin(1, b // 2, 2000)
    return "edges %d left, %d right, went back %d, front now %d" % (a, b, b // 2, distance("front"))


# ---- main loop ----
def drive():
    log("Line up:", line_up())
    f, l, r = distance("front"), distance("left"), distance("right")
    log("Start: front %d  left %d  right %d mm" % (f, l, r))
    # start cell has walls both sides, so the average is what a centred robot reads
    centred = (l + r) / 2
    if not 40 <= centred <= 200:
        log("Start side readings look wrong, using SIDE_GUESS_MM")
        centred = SIDE_GUESS_MM
    log("Centred side reading: %d mm" % centred)
    save_log()

    last_time = loop_time = last_trace = start = time.ticks_ms()
    left_speed.reset()
    right_speed.reset()
    speeds, closest_side = [], 9999
    trace = []
    last_mode, last_err, smooth_err, rate = None, None, None, 0
    modes = {}
    close_count = 0
    turns = 0
    # Right at the wall the side sensors only see the front wall, so we work out which sides
    # are open while still 120-280mm away and remember it
    open_left = open_right = None

    while time.ticks_diff(time.ticks_ms(), start) < MAX_SECONDS * 1000:
        f, l, r = distance("front"), distance("left"), distance("right")
        closest_side = min(closest_side, l, r)
        if SIDES_BLIND_MM <= f <= 280:
            open_left = l > centred * WALL_FACTOR
            open_right = r > centred * WALL_FACTOR

        # wall in front (2 readings in a row) -> pick a way and turn
        close_count = close_count + 1 if f < TURN_AT_MM else 0
        if close_count >= 2:
            stop()
            time.sleep_ms(80)
            if open_left or open_left is None:
                direction, why = 1, "left open" if open_left else "not sure, try left"
            elif open_right:
                direction, why = -1, "right open"
            else:
                direction, why = 1, "dead end, turning round"
            turns += 1
            result = turn(direction)
            trace.append("TURN %d at %d ms: %s. %s" % (
                turns, time.ticks_diff(time.ticks_ms(), start), why, result))
            save_log(trace)   # save after every turn in case we switch it off mid-run
            close_count = 0
            open_left = open_right = None
            last_mode, last_err, smooth_err, rate = None, None, None, 0
            left_speed.reset()
            right_speed.reset()
            last_time = loop_time = time.ticks_ms()
            continue

        now = time.ticks_ms()
        dt = time.ticks_diff(now, loop_time) / 1000
        loop_time = now

        if time.ticks_diff(now, last_time) >= 200:
            last_time = now
            speeds.append("%d/%d" % (left_speed.speed, right_speed.speed))

        # PD steering: P = how far off centre, D = how fast that's changing (stops it weaving)
        err, mode = off_centre(f, l, r, centred)
        modes[mode] = modes.get(mode, 0) + 1
        if mode == "both walls":
            centred += 0.02 * ((l + r) / 2 - centred)   # slowly update the centred reading
        if err is None or mode != last_mode or last_err is None or dt <= 0:
            rate = 0
            smooth_err = err
        else:
            smooth_err += SMOOTH * (err - smooth_err)
            new_rate = (smooth_err - last_err) / dt
            rate += SMOOTH * (new_rate - rate)
        last_mode, last_err = mode, smooth_err

        if smooth_err is None:
            steer = 0
        else:
            steer = STEER_P * smooth_err + STEER_D * rate
        steer = max(-MAX_STEER, min(MAX_STEER, steer))
        # steer by giving the wheels different target speeds
        left_speed.drive(TARGET_SPEED * (1 - steer))
        right_speed.drive(TARGET_SPEED * (1 + steer))

        if time.ticks_diff(now, last_trace) >= 200:
            last_trace = now
            trace.append("%5d %4d %4d %4d %6s %6s %5.2f %4.2f %4.2f" % (
                time.ticks_diff(now, start), f, l, r,
                "-" if smooth_err is None else "%d" % smooth_err, "%d" % rate, steer,
                left_speed.power, right_speed.power))

    stop()
    log("Stopped: time up after %d s, %d turns" % (MAX_SECONDS, turns))
    log("Ticks: left %d  right %d" % (left_wheel.ticks, right_wheel.ticks))
    if left_wheel.ticks < 50 or right_wheel.ticks < 50:
        log("WARNING: %s wheel barely moved, check the motor/encoder"
            % ("LEFT" if left_wheel.ticks < right_wheel.ticks else "RIGHT"))
    log("Closest a side reading got: %d mm" % closest_side)
    log("Steering used:", modes)
    log("Wheel speeds L/R every 0.2 s:", speeds)
    log("Power at the end: left %.2f  right %.2f" % (left_speed.power, right_speed.power))
    log("Centred side reading at the end: %d mm" % centred)
    log("Trace (ms front left right off-centre rate steer powerL powerR), and turns:")
    for t in trace:
        log(t)


# ---- start ----
def usb_plugged_in():
    try:
        return Pin("WL_GPIO2", Pin.IN).value() == 1   # VBUS pin on the Pico 2 W
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
    # on USB: just print the sensors and don't touch the log from the last run
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

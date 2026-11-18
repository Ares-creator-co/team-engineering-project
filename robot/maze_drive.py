# maze_drive.py - drive_straight.py plus turning into side openings (wall follower).
# When it sees an opening on the FOLLOW_SIDE partway down a corridor, it drives on to the
# middle of that cell and turns 90 deg into it.
# At a wall ahead: FOLLOW_SIDE if open, else the other side, else turn round.
# drive_straight.py is the saved working version, don't change that one.
#
# How to use: put the robot in a corridor pointing straight down it, switch the battery on.
# It drives for MAX_SECONDS, then stops.
# It starts driving as soon as it's switched on (no countdown). LED on while driving, off when done.
# Afterwards plug in USB and open last_run.txt on the Pico to see what happened.
# With USB plugged in it does NOT drive (see DRIVE_ON_USB), so plugging in is always safe.
#
# Steering: keeps the gap to the left wall equal to the gap to the right wall.
#           With only one wall, keeps that wall at the centred reading (learned at the start).
#           No walls, or front wall close: drives straight on.
#           Also damps by how fast it's closing on a wall (STEER_D), so it doesn't weave.
# Speed:    each wheel has its own speed control (every 50 ms, more power if it's too slow, less
#           if too fast), so it still drives straight if one motor is weaker than the other.
#           Steering works by asking one wheel to go faster and the other slower.
# Needs vl53l4cd.py on the Pico.
from machine import Pin, PWM, I2C
import time
from vl53l4cd import VL53L4CD, set_address

# ---------------- settings ----------------
TARGET_SPEED = 400     # encoder ticks per second for each wheel. Lower = slower
# Power each wheel starts at. The left motor needs far more than the right (wheel test 16 Nov:
# at 0.3 the left barely moved, right did 1300 ticks/s). The speed control corrects these as
# it drives; good starting values just stop it curving in the first second.
START_POWER_LEFT = 0.55   # 18 Nov maze_drive runs: left needed 0.49-0.61 (left motor keeps getting weaker)
START_POWER_RIGHT = 0.22  # and the right ~0.20-0.26
SPEED_GAIN = 0.00008   # power change every 50 ms per tick/s a wheel is off. Speed hunts: lower it
# Side sensors point 30 deg out from straight ahead (a V), ~23 mm either side of the front sensor.
# So they look ~100 mm AHEAD, and their readings change a lot when the robot turns. Because of
# that, left - right already contains the heading, so STEER_D can stay small.
# All the steering numbers below are in side-sensor mm (what the sensor reads), not sideways mm.
STEER_P = 0.002        # steering per mm of (left - right)/2, as a fraction of speed. Drifts: raise it
STEER_D = 0.0006       # steering per mm/s that difference changes (damping). Weaves: raise it
MAX_STEER = 0.35       # most the wheel speeds differ by when steering (0.35 = +/-35%)
SMOOTH = 0.5           # 0..1, how much of each new reading is used (lower = smoother, slower)
SIDE_GUESS_MM = 127    # side reading when centred. Every run has measured 124-131 in our maze.
                       # Only replaced by the start readings if both of those look like real walls
                       # The robot learns the real value at the start and while it drives.
WALL_FACTOR = 1.6      # a side reading under (centred reading x this) means there's a wall
SIDES_BLIND_MM = 120   # front wall closer than this: the V sensors see it too, so stop steering
TURN_AT_MM = 60        # turn when the wall in front is this close (2 readings in a row). was 40, then 50
# Turning finds the straight: while turning, the front sensor reads long (past TURN_CLEAR_MM)
# for a window of ~40 deg while it looks down an open corridor. It notes where that window
# starts and ends, then turns back to the middle of it = pointing straight down the corridor.
TURN_SPEED = 250       # wheel speed (ticks/s) while turning on the spot. Slower = more accurate
TURN_CLEAR_MM = 150    # front reading longer than this = looking into an open cell. Facing a wall
                       # in the cell reads ~30-90; an open cell with a wall after it reads ~200
TURN_MAX_WINDOW = 200  # ticks: if the long readings go on longer than this (open area), stop there
TURN_TIMEOUT_MS = 4000 # give up on a turn after this long and just try driving
MAX_SECONDS = 60       # stop after this long
# turning into openings
FOLLOW_SIDE = "right"  # "right" = right-hand rule, "left" = left-hand rule
TAKE_SIDE_OPENINGS = True
TICKS_PER_MM = 2.5     # calibrated 18 Nov: 2507 ticks was about 1 m
# Finding the middle of a gap: note where the side reading goes "open" and where it goes back
# to "wall" (the far corner). Halfway between = middle of the gap, as seen by the sensor.
# The sensor looks ~110 mm ahead and sits ~57 mm in front of the wheels, so the wheels reach
# the middle of the gap LOOK_AHEAD_MM after that. Turns too early: raise it. Too late: lower it
LOOK_AHEAD_MM = 120    # was 165: turned ~40-50 mm too late (18 Nov logs)
GAP_SEEN_MM = 85       # how long a one-cell gap looks to the side sensor (measured 57-92)
GAP_MIN_MM = 40        # ignore 'gaps' shorter than this (real ones measure ~85 mm, the sensor
                       # sees the far corner early so gaps look shorter than 168)
GAP_MAX_MM = 220       # gap still open after this far = no far wall (open area): turn at
                       # gap start + LOOK_AHEAD_MM + 84 instead
SETTLE_MS = 200        # stop and settle this long before turning in
TURN_90_TICKS = 130    # it coasts ~12 ticks after stopping, so this ends up ~142 = 90 deg
                       # (was 140, which ended at 151-153 = ~97 deg). Turns too far: lower it
OPEN_COUNT = 2         # readings in a row that must say "open" (or "wall") before we believe it
STUCK_MS = 500         # if the wheels hardly move for this long while driving, treat it as a wall
REVERSE_LEFT = True    # flip if the left wheel turns the wrong way
REVERSE_RIGHT = True   # flip if the right wheel turns the wrong way
LINE_UP = False        # False: just drive off straight as placed (the line-up turned left and hit walls)
TOO_CLOSE_MM = 65      # side reading under this (centred is ~125) = too close, steer away hard (was 55)
AFTER_TURN_MM = 100    # drive slower for this far after every turn so the steering can centre it
AFTER_TURN_SPEED = 0.6 # ... at this fraction of TARGET_SPEED
IGNORE_AFTER_TURN_MM = 60  # after a turn the side sensors still see the corridor we just left,
                           # so don't count side readings for deciding the next turn until here
TOO_CLOSE_STEER = 0.3  # how hard it steers away when too close
SELF_CHECK = False     # True: nudge left and back at the start to check the steering direction
                       # (it passed every run on 16 Nov, so it's off: the nudge knocked it crooked)
DRIVE_ON_USB = False   # False: when USB is plugged in, don't drive, just show the sensors

# ---------------- log + LED ----------------
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


# ---------------- motors ----------------
class Wheel:
    def __init__(self, in1, in2, enable, enc_a, enc_b, reverse):
        self.a = PWM(Pin(in1))
        self.b = PWM(Pin(in2))
        for p in (self.a, self.b):
            p.freq(1000)
            p.duty_u16(0)
        Pin(enable, Pin.OUT, value=1)   # PWMA / PWMB held on
        self.reverse = reverse
        self.ticks = 0
        self.enc_a = Pin(enc_a, Pin.IN, Pin.PULL_UP)
        self.enc_b = Pin(enc_b, Pin.IN, Pin.PULL_UP)
        self.enc_a.irq(trigger=Pin.IRQ_RISING | Pin.IRQ_FALLING, handler=self.count)

    def count(self, pin):
        self.ticks += 1

    def power(self, p):
        # p from -1 (full backwards) to 1 (full forwards)
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
    # holds one wheel at a target speed (ticks/s) by adjusting its power every 50 ms.
    # Negative target = backwards (the encoders can't tell direction, so it just uses the size).
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
        self.wheel.power(self.power if target >= 0 else -self.power)


left_speed = Speed(left_wheel, START_POWER_LEFT)
right_speed = Speed(right_wheel, START_POWER_RIGHT)


# ---------------- sensors ----------------
# front and left XSHUT wires are swapped on the soldered robot (checked 13 Nov)
SENSORS = (("front", 11, 0x30), ("left", 10, 0x31), ("right", 12, 0x32))
i2c = I2C(0, sda=Pin(0), scl=Pin(1), freq=400_000)
xshut = {name: Pin(pin, Pin.OUT, value=0) for name, pin, addr in SENSORS}
tof = {}


def start_sensors():
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
    except OSError:                   # odd I2C glitch (EIO): use the last good reading
        return last_good.get(name, 9999)
    d = 9999 if d is None else d
    last_good[name] = d
    return d


# ---------------- self check ----------------
def side_average():
    l = r = 0
    for i in range(4):
        l += distance("left")
        r += distance("right")
    return l / 4, r / 4


def run_wheel(control, ticks=None, ms=250):
    # spin one wheel forward until it has done `ticks` (or for `ms` if its encoder isn't counting)
    wheel = control.wheel
    t0, start = wheel.ticks, time.ticks_ms()
    control.reset()
    while time.ticks_diff(time.ticks_ms(), start) < (ms if ticks is None else 1000):
        control.drive(TURN_SPEED)
        if ticks is not None and wheel.ticks - t0 >= ticks:
            break
    stop()
    time.sleep_ms(150)
    return wheel.ticks - t0


def self_check():
    # Turning LEFT should make the left reading shorter and the right reading longer
    # (the side sensors point 30 deg out from straight ahead). If the readings go the other way,
    # the side sensors or the wheels are swapped; for driving it doesn't matter which, the
    # steering just has to be flipped. Returns 1 (normal) or -1 (flipped).
    l0, r0 = side_average()
    turned = run_wheel(right_speed)                   # right wheel only = turn left
    l1, r1 = side_average()
    back = run_wheel(left_speed, turned if turned > 10 else None)   # left wheel only = turn back
    change = (l1 - r1) - (l0 - r0)                    # should go negative
    log("Self check: right wheel %d ticks, left wheel %d ticks" % (turned, back))
    log("Self check: before L %d R %d, after a left nudge L %d R %d" % (int(l0), int(r0), int(l1), int(r1)))
    if turned < 10:
        log("Self check: WARNING RIGHT wheel not turning or its encoder not counting")
    if back < 10:
        log("Self check: WARNING LEFT wheel not turning or its encoder not counting")
    if max(l0, r0, l1, r1) > 300:
        log("Self check: no wall on one side, can't check the steering, assuming normal")
        return 1
    if change < -8:
        log("Self check: steering direction OK")
        return 1
    if change > 8:
        log("Self check: STEERING WAS BACKWARDS (side sensors or wheels swapped) - flipped it")
        return -1
    log("Self check: readings barely moved (%d mm), assuming normal" % int(change))
    return 1


# ---------------- driving ----------------
def wall_ahead(f, l, r, near):
    # A real wall ahead shows on all 3 sensors: the angled side sensors hit it too, so they read
    # short as well. If the front reads short but the sides don't, it's a bad front reading
    # (it read 48 mm with an open corridor ahead on 18 Nov), so ignore it.
    limit = 1.2 * f + 40
    return f < near and l < limit and r < limit


def wall_ahead_turn(f, l, r):
    # for deciding to turn: front close and at least one side sensor also close. When it's
    # crooked against a wall one side sensor looks out into the open, so we can't need both
    limit = 1.2 * f + 40
    if f < TURN_AT_MM and (l < limit or r < limit):
        return True
    # jammed: wall fairly close in front and a side practically touching (wheels just slip here)
    return f < TURN_AT_MM + 40 and (l < 30 or r < 30)


def off_centre(f, l, r, centred):
    # how far off-centre (in side-sensor mm): positive = too far right (so steer left)
    if wall_ahead(f, l, r, SIDES_BLIND_MM):
        return None, "front wall near"   # side beams are hitting the front wall, ignore them
    wall = centred * WALL_FACTOR
    if l < wall and r < wall:
        return (l - r) / 2, "both walls"
    if l < wall:
        return l - centred, "left wall"
    if r < wall:
        return centred - r, "right wall"
    return None, "no walls"


def spin(d, until, timeout_ms, watch=None):
    # turn on the spot (d = 1 left, -1 right). Calls watch(angle, front) each loop and stops
    # when it returns True, or when angle (average ticks of both wheels) reaches `until`.
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
    time.sleep_ms(80)                 # let it stop, then count any coast too
    return ((left_wheel.ticks - l0) + (right_wheel.ticks - r0)) // 2


def turn(direction, sign):
    # turn (1 = left, -1 = right) until it's pointing straight down the next open corridor
    d = direction * sign              # if the steering was backwards, so is turning
    w = {"found": None, "lost": None, "long": 0, "short": 0, "best": 0, "best_at": 0}

    def watch(angle, f):
        if f > w["best"] and f < 2000:
            w["best"], w["best_at"] = f, angle
        # count readings in a row, so one odd reading can't start or end the window
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

    total = spin(d, None, TURN_TIMEOUT_MS, watch)
    found, lost = w["found"], w["lost"]
    name = "left" if direction > 0 else "right"
    if found is None:
        # never saw a clear window: go back to where the front reading was longest
        back = total - w["best_at"]
        if back > 0:
            spin(-d, back, 3000)
        return "%s: no clear window in %d ticks, went back to the longest reading (%d mm) -> front now %d" % (
            name, total, w["best"], distance("front"))
    if lost is None:
        lost = total
    back = total - (found + lost) // 2   # how far past the middle of the window it ended up
    if back > 0:
        spin(-d, back, 1500)
    return "%s: corridor from %d to %d ticks, ended at %d, turned back %d -> front now %d" % (
        name, found, lost, total, back, distance("front"))


def edge(d, limit):
    # turn (d = 1 left, -1 right) until the front sensor stops seeing the open corridor
    # only counts the edge after it has seen the corridor (it may start off looking at a wall)
    w = {"short": 0, "seen": False}

    def watch(angle, f):
        if f >= TURN_CLEAR_MM:
            w["seen"], w["short"] = True, 0
        elif w["seen"]:
            w["short"] += 1
        return w["short"] >= 2
    return spin(d, limit, 3000, watch)


def line_up(sign):
    # Start-of-run: point straight down the corridor even if it was put down at an angle.
    # Turns left until the front sensor sees a wall (edge of the corridor), then right to the
    # other edge, then back to halfway. If it's facing a wall, it does a normal turn instead.
    if distance("front") < TURN_CLEAR_MM:
        return "facing a wall, turning to find a corridor: " + turn(1, sign)
    a = edge(sign, 150)                # left edge (limit ~60 deg)
    b = edge(-sign, a + 300)           # right edge
    spin(sign, b // 2, 2000)           # back to the middle
    return "edges %d left, %d right, went back %d -> front now %d" % (a, b, b // 2, distance("front"))


def drive():
    sign = self_check() if SELF_CHECK else 1
    if LINE_UP:
        log("Line up:", line_up(sign))
    f, l, r = distance("front"), distance("left"), distance("right")
    log("Start: front %d  left %d  right %d mm" % (f, l, r))
    # the start cell has walls on both sides, so the average of the two is the centred reading
    # (averaging means it doesn't matter if you put it down a bit off-centre)
    # only trust the start readings if BOTH sides look like walls near the usual value.
    # (started next to an opening on 18 Nov: averaged 111 and 246 = 178, and then it
    # thought the opening was a wall)
    if 95 <= l <= 160 and 95 <= r <= 160:
        centred = (l + r) / 2
    else:
        log("Start side readings not both walls, using SIDE_GUESS_MM")
        centred = SIDE_GUESS_MM
    log("Centred side reading: %d mm" % centred)
    save_log()   # so there's a log even if the power is cut mid-run

    last_time = loop_time = last_trace = start = time.ticks_ms()
    left_speed.reset()
    right_speed.reset()
    speeds, closest_side = [], 9999
    trace = []
    last_mode, last_err, smooth_err, rate = None, None, None, 0
    modes = {}
    close_count = 0
    turns = 0
    # Is there an opening on each side of the cell ahead? The side sensors look ~100 mm ahead,
    # so they see the last cell's side walls while the front wall is ~150-280 mm away. Right at
    # the wall they only see the front wall, so we remember what they saw on the way in.
    open_left = open_right = None
    open_l_count = open_r_count = seen_ahead = 0
    last_turn_ticks = (left_wheel.ticks + right_wheel.ticks) // 2
    # side openings: only count an opening after we've seen a left wall (wall -> gap), and
    # remember the tick count where we should turn
    seen_left_wall = False
    left_gap_count = 0
    turn_at_ticks = None
    gap_start = None
    wall_count = 0
    side_turns = 0
    started_in_gap = False
    if FOLLOW_SIDE == "right":
        side0 = r
    else:
        side0 = l
    if TAKE_SIDE_OPENINGS and side0 > centred * WALL_FACTOR:
        # put down right beside an opening: we don't know where the gap began, so we'll
        # work out the middle from where it ends instead
        seen_left_wall = True
        started_in_gap = True
        gap_start = (left_wheel.ticks + right_wheel.ticks) // 2
        log("Started beside an opening on the %s" % FOLLOW_SIDE)
    last_moving = time.ticks_ms()
    last_save = last_moving

    while time.ticks_diff(time.ticks_ms(), start) < MAX_SECONDS * 1000:
        f, l, r = distance("front"), distance("left"), distance("right")
        since_turn = ((left_wheel.ticks + right_wheel.ticks) // 2 - last_turn_ticks) / TICKS_PER_MM
        closest_side = min(closest_side, l, r)
        # which sides are open at the end of this corridor? Count "open" readings on the way in.
        # Don't just use the last reading: right at the end the angled sensor can hit the wall
        # ahead where it carries on past the gap (zigzags) and read "wall" when it's open
        if since_turn >= IGNORE_AFTER_TURN_MM and TURN_AT_MM <= f <= 280:
            seen_ahead += 1
            if l > centred * WALL_FACTOR:
                open_l_count += 1
            if r > centred * WALL_FACTOR:
                open_r_count += 1
            open_left = open_l_count >= 3
            open_right = open_r_count >= 3

        # wall ahead: turn and carry on (2 close readings in a row, so one odd reading can't do it)
        close_count = close_count + 1 if wall_ahead_turn(f, l, r) else 0
        # stuck: trying to drive but the wheels have hardly moved for STUCK_MS
        if left_speed.speed + right_speed.speed > TARGET_SPEED // 2:
            last_moving = time.ticks_ms()
        stuck = time.ticks_diff(time.ticks_ms(), last_moving) > STUCK_MS
        if stuck:
            trace.append("STUCK at %d ms (wheels not moving), treating it as a wall" % (
                time.ticks_diff(time.ticks_ms(), start)))
        if close_count >= 2 or stuck:
            stop()
            time.sleep_ms(80)
            if FOLLOW_SIDE == "right":
                first_open, other_open, first_dir = open_right, open_left, -1
            else:
                first_open, other_open, first_dir = open_left, open_right, 1
            if first_open or first_open is None:
                direction = first_dir
                why = FOLLOW_SIDE + (" open" if first_open else " (not sure)")
            elif other_open:
                direction = -first_dir
                why = "other side open"
            else:
                direction, why = 1, "dead end, turning round"
            why += " [open readings L %d R %d of %d]" % (open_l_count, open_r_count, seen_ahead)
            turns += 1
            result = turn(direction, sign)
            trace.append("TURN %d at %d ms: %s. %s" % (
                turns, time.ticks_diff(time.ticks_ms(), start), why, result))
            save_log(trace)               # so switching off mid-run doesn't lose everything
            close_count = 0
            open_left = open_right = None
            open_l_count = open_r_count = seen_ahead = 0
            last_turn_ticks = (left_wheel.ticks + right_wheel.ticks) // 2
            last_mode, last_err, smooth_err, rate = None, None, None, 0
            left_speed.reset()
            right_speed.reset()
            last_time = loop_time = time.ticks_ms()
            last_moving = last_time
            seen_left_wall = False
            left_gap_count = 0
            turn_at_ticks = None
            gap_start = None
            wall_count = 0
            started_in_gap = False
            continue

        # ---- side openings (on the FOLLOW_SIDE) ----
        # (variable names say "left" from the first version, they mean the follow side)
        moved = (left_wheel.ticks + right_wheel.ticks) // 2
        if FOLLOW_SIDE == "right":
            side = r
        else:
            side = l
        if TAKE_SIDE_OPENINGS and turn_at_ticks is None:
            now_ms = time.ticks_diff(time.ticks_ms(), start)
            if side < centred * WALL_FACTOR:
                seen_left_wall = True
                left_gap_count = 0
                wall_count += 1
                if gap_start is not None and wall_count >= OPEN_COUNT and not started_in_gap and \
                        moved - gap_start < GAP_MIN_MM * TICKS_PER_MM:
                    trace.append("ignored a %d mm gap (too short to be real)" % (
                        (moved - gap_start) / TICKS_PER_MM))
                    gap_start = None
                if gap_start is not None and wall_count >= OPEN_COUNT:
                    # far corner of the gap: the middle is halfway between start and here
                    middle = (gap_start + moved) // 2
                    if started_in_gap:
                        # we didn't see where it started, so go back half a gap from the end
                        middle = moved - int(GAP_SEEN_MM / 2 * TICKS_PER_MM)
                        started_in_gap = False
                    turn_at_ticks = middle + int(LOOK_AHEAD_MM * TICKS_PER_MM)
                    trace.append("GAP END at %d ms, gap was %d mm long, turning in %d mm" % (
                        now_ms, (moved - gap_start) / TICKS_PER_MM,
                        (turn_at_ticks - moved) / TICKS_PER_MM))
                    gap_start = None
            elif seen_left_wall:
                wall_count = 0
                left_gap_count += 1
                if left_gap_count >= OPEN_COUNT and gap_start is None:
                    gap_start = moved
                    trace.append("GAP START on the %s at %d ms" % (FOLLOW_SIDE, now_ms))
            if gap_start is not None and moved - gap_start > GAP_MAX_MM * TICKS_PER_MM:
                # no far wall showed up: open area, so go from the start of the gap instead
                turn_at_ticks = gap_start + int((LOOK_AHEAD_MM + 84) * TICKS_PER_MM)
                trace.append("NO FAR WALL at %d ms, turning in %d mm" % (
                    now_ms, (turn_at_ticks - moved) / TICKS_PER_MM))
                gap_start = None

        if turn_at_ticks is not None and moved >= turn_at_ticks:
            stop()
            time.sleep_ms(SETTLE_MS)
            if FOLLOW_SIDE == "right":
                d = -1
            else:
                d = 1
            got = spin(d * sign, TURN_90_TICKS, 2000)
            side_turns += 1
            trace.append("SIDE TURN %d at %d ms: %s 90, %d ticks, front now %d" % (
                side_turns, time.ticks_diff(time.ticks_ms(), start), FOLLOW_SIDE, got, distance("front")))
            save_log(trace)
            close_count = 0
            open_left = open_right = None
            open_l_count = open_r_count = seen_ahead = 0
            last_turn_ticks = (left_wheel.ticks + right_wheel.ticks) // 2
            last_mode, last_err, smooth_err, rate = None, None, None, 0
            left_speed.reset()
            right_speed.reset()
            last_time = loop_time = time.ticks_ms()
            last_moving = last_time
            seen_left_wall = False
            left_gap_count = 0
            turn_at_ticks = None
            gap_start = None
            wall_count = 0
            started_in_gap = False
            continue

        now = time.ticks_ms()
        dt = time.ticks_diff(now, loop_time) / 1000
        loop_time = now

        if time.ticks_diff(now, last_time) >= 200:
            last_time = now
            speeds.append("%d/%d" % (left_speed.speed, right_speed.speed))

        # steering = P (how far off-centre) + D (how fast it's heading towards a wall).
        # The D part is what stops it overshooting the centre and weaving into the walls.
        err, mode = off_centre(f, l, r, centred)
        modes[mode] = modes.get(mode, 0) + 1
        if mode == "both walls":
            # keep learning the centred reading (slowly), so one-wall mode aims for the right spot
            centred += 0.02 * ((l + r) / 2 - centred)
        if err is None or mode != last_mode or last_err is None or dt <= 0:
            rate = 0                      # no walls / walls just changed: no reliable rate
            smooth_err = err
        else:
            smooth_err += SMOOTH * (err - smooth_err)
            new_rate = (smooth_err - last_err) / dt
            rate += SMOOTH * (new_rate - rate)
        last_mode, last_err = mode, smooth_err

        if smooth_err is None:
            steer = 0                     # no walls: just drive straight on the encoders
        else:
            steer = sign * (STEER_P * smooth_err + STEER_D * rate)
        # safety: if one side is nearly touching (and it's not just both beams seeing the front
        # wall, which reads about the same on both sides), steer away hard. Works even when the
        # normal steering is switched off near a front wall
        if l < TOO_CLOSE_MM and r - l > 30:
            steer = -TOO_CLOSE_STEER * sign       # too close on the left: steer right
            mode = "too close left"
        elif r < TOO_CLOSE_MM and l - r > 30:
            steer = TOO_CLOSE_STEER * sign        # too close on the right: steer left
            mode = "too close right"
        if mode.startswith("too close"):
            modes[mode] = modes.get(mode, 0) + 1
        steer = max(-MAX_STEER, min(MAX_STEER, steer))
        # steer by asking for different wheel speeds; each wheel's speed control finds the power
        speed = TARGET_SPEED
        if since_turn < AFTER_TURN_MM:
            speed = TARGET_SPEED * AFTER_TURN_SPEED   # slow just after a turn
        left_speed.drive(speed * (1 - steer))
        right_speed.drive(speed * (1 + steer))

        if time.ticks_diff(now, last_save) >= 2000:   # save every 2 s so we always get a log
            last_save = now
            save_log(trace)

        if time.ticks_diff(now, last_trace) >= 200:   # trace for tuning, 5 lines a second
            last_trace = now
            trace.append("%5d %4d %4d %4d %6s %6s %5.2f %4.2f %4.2f" % (
                time.ticks_diff(now, start), f, l, r,
                "-" if smooth_err is None else "%d" % smooth_err, "%d" % rate, steer,
                left_speed.power, right_speed.power))

    stop()
    log("Stopped: time up after %d s, %d wall turns, %d side turns" % (MAX_SECONDS, turns, side_turns))
    log("Ticks: left %d  right %d" % (left_wheel.ticks, right_wheel.ticks))
    if left_wheel.ticks < 50 or right_wheel.ticks < 50:
        log("WARNING: %s wheel barely counted - wheel not driving or encoder not connected"
            % ("LEFT" if left_wheel.ticks < right_wheel.ticks else "RIGHT"))
    log("Closest a side reading got: %d mm" % closest_side)
    log("Steering used:", modes)
    log("Wheel speeds L/R every 0.2 s:", speeds)
    log("Power at the end: left %.2f  right %.2f  (use these as START_POWER_LEFT/RIGHT)"
        % (left_speed.power, right_speed.power))
    log("Centred side reading at the end: %d mm" % centred)
    log("Trace (ms front left right off-centre rate steer powerL powerR), and turns:")
    for t in trace:
        log(t)


# ---------------- run ----------------
def usb_plugged_in():
    try:
        return Pin("WL_GPIO2", Pin.IN).value() == 1   # VBUS sense on the Pico 2 W
    except Exception:
        return False


def keep_previous_log():
    # the run before this one is kept as prev_run.txt (plugging in USB starts a new run)
    import os
    try:
        if "last_run.txt" in os.listdir():
            os.rename("last_run.txt", "prev_run.txt")
    except Exception:
        pass


if usb_plugged_in() and not DRIVE_ON_USB:
    # plugged into the laptop: report only, never drive, never overwrite the battery run's log
    start_sensors()
    print("USB connected, so not driving. Sensors now: front %d  left %d  right %d mm"
          % (distance("front"), distance("left"), distance("right")))
    raise SystemExit

keep_previous_log()
try:
    log("maze_drive starting")
    time.sleep_ms(500)        # let the battery settle
    start_sensors()
    if "front" not in tof:
        raise RuntimeError("front sensor failed, not driving without it")
    led.value(1)
    drive()
    led.value(0)
except Exception as e:
    stop()
    log("ERROR:", e)              # details are in last_run.txt
finally:
    stop()
    save_log()

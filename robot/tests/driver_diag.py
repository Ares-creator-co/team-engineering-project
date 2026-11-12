# Steps the TB6612 through its modes, 5 s each. Battery ON, wheel off the ground.
# During each step, try turning the wheel by hand and note what it does.
from machine import Pin
import time

ai1 = Pin(2, Pin.OUT, value=0)   # physical pin 4
ai2 = Pin(3, Pin.OUT, value=0)   # physical pin 5
pwma = Pin(5, Pin.OUT, value=0)  # physical pin 7

STEPS = (
    # AI1, AI2, PWMA, name, what you should feel
    (0, 0, 0, "1 COAST", "wheel turns FREELY by hand"),
    (1, 1, 1, "2 BRAKE", "wheel is STIFF"),
    (1, 0, 1, "3 FORWARD full power", "wheel SPINS on its own"),
    (0, 1, 1, "4 BACKWARD full power", "wheel SPINS the other way"),
)

try:
    for a1, a2, p, name, expect in STEPS:
        ai1.value(a1)
        ai2.value(a2)
        pwma.value(p)
        print("Step", name, "- AI1=%d AI2=%d PWMA=%d - expect: %s" % (a1, a2, p, expect))
        time.sleep(5)
finally:
    ai1.value(0)
    ai2.value(0)
    pwma.value(0)
    print("done, driver off")

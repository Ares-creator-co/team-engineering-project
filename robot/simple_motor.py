# Spins one motor through the TB6612FNG driver and prints the encoder count.
# Keep the wheels off the ground, and keep a hand on the battery switch.
#
# TB6612 wiring this expects (physical pin numbers in brackets):
#   PWMA -> GP5 (7)    speed / on-off
#   AI1  -> GP2 (4)    direction
#   AI2  -> GP3 (5)    direction
#   STBY, VCC -> 3.3 V    VM -> battery +    GND -> battery - and Pico GND
#   AO1/AO2 -> motor M1/M2
from machine import Pin, PWM
import time

SPEED = 0.5      # 0.0 to 1.0
RUN_MS = 1500

PWMA_PIN = 5     # physical pin 7
AI1_PIN = 2      # physical pin 4
AI2_PIN = 3      # physical pin 5 (set to None if AI2 is not wired)
ENC_A_PIN = 8    # physical pin 11
ENC_B_PIN = 9    # physical pin 12

# Encoder: count every time channel A changes
ticks = 0
enc_a = Pin(ENC_A_PIN, Pin.IN, Pin.PULL_UP)
enc_b = Pin(ENC_B_PIN, Pin.IN, Pin.PULL_UP)

def on_edge(pin):
    global ticks
    if enc_a.value() == enc_b.value():
        ticks += 1
    else:
        ticks -= 1

enc_a.irq(trigger=Pin.IRQ_RISING | Pin.IRQ_FALLING, handler=on_edge)

# Driver
pwma = PWM(Pin(PWMA_PIN))
pwma.freq(20_000)
pwma.duty_u16(0)
ai1 = Pin(AI1_PIN, Pin.OUT, value=0)
ai2 = Pin(AI2_PIN, Pin.OUT, value=0) if AI2_PIN is not None else None

def stop():
    pwma.duty_u16(0)
    ai1.value(0)
    if ai2:
        ai2.value(0)

def run(forward):
    global ticks
    ticks = 0
    ai1.value(1 if forward else 0)
    if ai2:
        ai2.value(0 if forward else 1)
    pwma.duty_u16(int(SPEED * 65535))
    for i in range(RUN_MS // 200):
        time.sleep_ms(200)
        print("Ticks:", ticks)

    stop()
    time.sleep_ms(500)

try:
    print("FORWARD")
    run(True)
    if ai2:
        print("BACKWARD")
        run(False)
    else:
        print("(AI2 not set, so skipping backward)")
finally:
    # always stop, even if you press Stop in Thonny
    stop()
    print("stopped")

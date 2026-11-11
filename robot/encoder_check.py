# Checks one encoder by hand: no motor power needed.
# Turn the wheel slowly with your fingers and watch the numbers change.
#
# Wiring (physical pin numbers in brackets):
#   encoder VCC -> 3V3 (36)    encoder GND -> GND (38)
#   encoder A   -> GP8 (11)    encoder B   -> GP9 (12)
# Leave M1/M2 unplugged or the battery off for this test.
from machine import Pin
import time

ENC_A_PIN = 8    # physical pin 11
ENC_B_PIN = 9    # physical pin 12

enc_a = Pin(ENC_A_PIN, Pin.IN, Pin.PULL_UP)
enc_b = Pin(ENC_B_PIN, Pin.IN, Pin.PULL_UP)

ticks = 0

def on_edge(pin):
    global ticks
    if enc_a.value() == enc_b.value():
        ticks += 1
    else:
        ticks -= 1

enc_a.irq(trigger=Pin.IRQ_RISING | Pin.IRQ_FALLING, handler=on_edge)

print("Turn the wheel. Press Stop in Thonny to finish.")
print("A B ticks")

last = None
try:
    while True:
        now = (enc_a.value(), enc_b.value(), ticks)
        if now != last:
            print(now[0], now[1], now[2])
            last = now
        time.sleep_ms(20)
finally:
    enc_a.irq(handler=None)
    print("final ticks:", ticks)

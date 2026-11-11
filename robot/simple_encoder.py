# Prints one encoder's count. No motor power: turn the wheel by hand.
# Change MOTOR to "left" or "right".
from machine import Pin
import time

MOTOR = "right"

# GP numbers (not physical pin numbers): encoder A, encoder B
PINS = {
    "left":  (6, 7),   # physical pins 9, 10
    "right": (8, 9),   # physical pins 11, 12
}
ENC_A, ENC_B = PINS[MOTOR]

ticks = 0
enc_a = Pin(ENC_A, Pin.IN, Pin.PULL_UP)
enc_b = Pin(ENC_B, Pin.IN, Pin.PULL_UP)

def on_edge(pin):
    global ticks
    if enc_a.value() == enc_b.value():
        ticks += 1
    else:
        ticks -= 1

enc_a.irq(trigger=Pin.IRQ_RISING | Pin.IRQ_FALLING, handler=on_edge)

print("Turn the", MOTOR, "wheel by hand. Press Stop to finish and ,make changes.")
while True:
    print("ticks:", ticks, " A:", enc_a.value(), " B:", enc_b.value())
    time.sleep_ms(200)


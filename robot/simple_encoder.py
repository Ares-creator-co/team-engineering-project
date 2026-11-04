# Prints the LEFT encoder count. No motor power: turn the wheel by hand.
# For the right encoder use ENC_A=8, ENC_B=9.
from machine import Pin
import time

ENC_A = 6
ENC_B = 7

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

while True:
    print("ticks:", ticks, " A:", enc_a.value(), " B:", enc_b.value())
    time.sleep_ms(200)

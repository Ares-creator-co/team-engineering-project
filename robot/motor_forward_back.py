## Wiring (physical pin numbers in brackets):
#   AI1  -> GP2 (4)     AI2 -> GP3 (5)     PWMA -> GP5 (7)
#   STBY, VCC -> 3V3 (36)
#   VM -> power supply +     GND -> power supply - AND Pico GND (38)
#   AO1, AO2 -> the motor's two power wires (the ones that spun it on the supply)
from machine import Pin
import time

ai1 = Pin(2, Pin.OUT, value=0)
ai2 = Pin(3, Pin.OUT, value=0)
pwma = Pin(5, Pin.OUT, value=0)   # full speed on/off, no PWM

def stop():
    pwma.value(0)
    ai1.value(0)
    ai2.value(0)

try:
    print("forward")
    ai1.value(1); ai2.value(0); pwma.value(1)
    time.sleep(2)
    stop(); time.sleep(1)

    print("backward")
    ai1.value(0); ai2.value(1); pwma.value(1)
    time.sleep(2)
finally:
    stop()
    print("stopped")

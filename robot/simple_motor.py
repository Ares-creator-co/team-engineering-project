# Spins the LEFT motor forward for 2 s and prints the encoder count while it runs.
# Keep the wheels off the ground.
# For the right motor use IN1=4, IN2=5, ENC_A=8, ENC_B=9.
from machine import Pin, PWM
import time

IN1 = 2
IN2 = 3
ENC_A = 6
ENC_B = 7
SPEED = 0.4  # 0.0 to 1.0

# Encoder: count every time channel A changes
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

# Motor driver: PWM on IN1 = forward, PWM on IN2 = backward
in1 = PWM(Pin(IN1))
in2 = PWM(Pin(IN2))
in1.freq(20_000)
in2.freq(20_000)

try:
    in1.duty_u16(int(SPEED * 65535))
    in2.duty_u16(0)
    for i in range(10):
        time.sleep_ms(200)
        print("ticks:", ticks)
finally:
    # always stop, even if you press Stop in Thonny
    in1.duty_u16(0)
    in2.duty_u16(0)
    print("stopped, final ticks:", ticks)

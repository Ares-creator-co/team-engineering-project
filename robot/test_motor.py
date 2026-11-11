# assumes PWMA, PWMB and STBY are wired to 3.3 V, the same setup as mindmap.py
from machine import Pin, PWM
import time

PWM_FREQ = 20_000
MAX_DUTY = 65535

# lower this if the motors spin too fast, raise it if they only hum
TEST_SPEED = 0.4
RUN_MS = 1500
PAUSE_MS = 1000
START_DELAY = 5

# name, IN1 pin, IN2 pin, encoder A pin, encoder B pin, reversed
MOTORS = (
    ("left", 2, 3, 6, 7, False),
    ("right", 4, 5, 8, 9, True),
)


class TestMotor:

    def __init__(self, name, in1_pin, in2_pin, enc_a_pin, enc_b_pin, reversed):
        self.name = name
        self.in1 = PWM(Pin(in1_pin))
        self.in2 = PWM(Pin(in2_pin))
        self.in1.freq(PWM_FREQ)
        self.in2.freq(PWM_FREQ)
        self.reversed = reversed
        self.ticks = 0
        self.enc_a = Pin(enc_a_pin, Pin.IN, Pin.PULL_UP)
        self.enc_b = Pin(enc_b_pin, Pin.IN, Pin.PULL_UP)
        self.enc_a.irq(trigger=Pin.IRQ_RISING | Pin.IRQ_FALLING, handler=self._on_edge)
        self.stop()

    def _on_edge(self, pin):
        if self.enc_a.value() == self.enc_b.value():
            self.ticks += 1
        else:
            self.ticks -= 1

    def set_speed(self, speed):
        duty = int(abs(speed) * MAX_DUTY)
        if (speed >= 0) != self.reversed:
            self.in1.duty_u16(duty)
            self.in2.duty_u16(0)
        else:
            self.in1.duty_u16(0)
            self.in2.duty_u16(duty)

    def stop(self):
        self.in1.duty_u16(0)
        self.in2.duty_u16(0)


def run(motors, label, speed):
    # runs the motors for a moment and reports how far each encoder moved
    print("Now:", label)
    for motor in motors:
        motor.ticks = 0
        motor.set_speed(speed)
    time.sleep_ms(RUN_MS)
    for motor in motors:
        motor.stop()
    for motor in motors:
        print("  ", motor.name, "encoder ticks:", motor.ticks)
    time.sleep_ms(PAUSE_MS)


motors = []
try:
    motors = [TestMotor(*config) for config in MOTORS]
    print("Keep the wheels off the ground. Starting in", START_DELAY, "seconds")
    time.sleep(START_DELAY)

    for motor in motors:
        run([motor], motor.name + " motor FORWARD", TEST_SPEED)
        run([motor], motor.name + " motor BACKWARD", -TEST_SPEED)
    run(motors, "both motors FORWARD", TEST_SPEED)

    print("Done. Ticks stay at 0 if the encoders are not wired yet.")
finally:
    # always stop the motors, even if you press Stop in Thonny
    for motor in motors:
        motor.stop()

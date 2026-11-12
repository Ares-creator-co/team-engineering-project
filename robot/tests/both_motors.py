# Runs motor A, then motor B, forward and back through the TB6612, printing encoder ticks.
# Wheels off the ground. Supply on VM/GND.
#
# NOTE: the motor connector labels are printed in REVERSE. Wire by the real wire:
#   label GND -> AO1/BO1    label B -> AO2/BO2    label A -> 3.3 V
#   label M1 -> GND         label VCC -> enc A    label M2 -> enc B
#
# Wiring (physical pin numbers in brackets):
#   Motor A: AIN1 GP2 (4)   AIN2 GP3 (5)    PWMA GP5 (7)    encoder A GP8 (11), B GP9 (12)
#   Motor B: BIN1 GP6 (9)   BIN2 GP7 (10)   PWMB GP4 (6)    encoder A GP14 (19), B GP15 (20)
#   STBY, VCC -> 3.3 V
from machine import Pin
import time

# PWMA/PWMB are plain on/off (full speed). GP4 and GP5 share one PWM unit on the
# Pico, and with PWM on both, motor A did not run (suspected cause). Speed control comes later.
RUN_MS = 1500


class Motor:
    def __init__(self, name, in1, in2, pwm, enc_a, enc_b):
        self.name = name
        self.in1 = Pin(in1, Pin.OUT, value=0)
        self.in2 = Pin(in2, Pin.OUT, value=0)
        self.pwm = Pin(pwm, Pin.OUT, value=0)
        self.ticks = 0
        self.enc_a = Pin(enc_a, Pin.IN, Pin.PULL_UP)
        self.enc_b = Pin(enc_b, Pin.IN, Pin.PULL_UP)
        self.enc_a.irq(trigger=Pin.IRQ_RISING | Pin.IRQ_FALLING, handler=self.on_edge)

    def on_edge(self, pin):
        if self.enc_a.value() == self.enc_b.value():
            self.ticks += 1
        else:
            self.ticks -= 1

    def run(self, forward):
        self.in1.value(1 if forward else 0)
        self.in2.value(0 if forward else 1)
        self.pwm.value(1)

    def stop(self):
        self.pwm.value(0)
        self.in1.value(0)
        self.in2.value(0)


motor_a = Motor("A", 2, 3, 5, 8, 9)
motor_b = Motor("B", 6, 7, 4, 14, 15)
motors = (motor_a, motor_b)


def test(group, label, forward):
    print(label)
    for m in group:
        m.ticks = 0
        m.run(forward)
    time.sleep_ms(RUN_MS)
    for m in group:
        m.stop()
    for m in group:
        print("  motor", m.name, "ticks:", m.ticks)
    time.sleep_ms(500)


try:
    for m in motors:
        test([m], "motor " + m.name + " FORWARD", True)
        test([m], "motor " + m.name + " BACKWARD", False)
    test(motors, "BOTH FORWARD", True)
    
finally:
    for m in motors:
        m.stop()
    print("stopped")
 
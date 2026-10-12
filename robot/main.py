
#importing pin,PWM and I2C pin is one of the physical legs of the pico each called a GPIO (general purpose input output pin)
#and pin is the class we are importing to control them 
#I2C or inter intergrated circuit is a comminication protocol used for sending data between two chips or wires - important for sending
# data between two hardware components 
from machine import Pin, PWM, I2C

import time
#distance sensor chip - importing corresponding libraries and modules for programming the distance chip
from vl53l0x import VL53L0X

# I2C bus shared by all three VL53L0X sensors
I2C_ID = 0
I2C_SDA_PIN = 0
I2C_SCL_PIN = 1
I2C_FREQ = 400_000

# XSHUT pins so each sensor can be powered up one at a time
FRONT_XSHUT_PIN = 10
LEFT_XSHUT_PIN = 11
RIGHT_XSHUT_PIN = 12

# every VL53L0X starts on 0x29 so each gets its own address
DEFAULT_TOF_ADDRESS = 0x29
FRONT_TOF_ADDRESS = 0x30
LEFT_TOF_ADDRESS = 0x31
RIGHT_TOF_ADDRESS = 0x32

# DRV8871 inputs, one driver board per motor
LEFT_MOTOR_IN1_PIN = 2
LEFT_MOTOR_IN2_PIN = 3
RIGHT_MOTOR_IN1_PIN = 4
RIGHT_MOTOR_IN2_PIN = 5

# motor encoder pins
LEFT_ENCODER_A_PIN = 6
LEFT_ENCODER_B_PIN = 7
RIGHT_ENCODER_A_PIN = 8
RIGHT_ENCODER_B_PIN = 9

PWM_FREQ = 20_000
MAX_DUTY = 65535

#initialising Buzzer 
pin.Buzzer = 1 
pin.Buzzer_fail = 0 


#NEW TOF == VL53L4CD 

# one VL53L0X distance sensor time of flight sensor
class TofSensor:
#note that Pin.out refers to the voltage of the pin being emmitied which for the pico is 3.3v for the initialise func

    def initialise(self, name, xshut_pin, address):
        self.name = name
        self.xshut = Pin(xshut_pin, Pin.OUT, value=0)
        self.address = address
        self.device = None

    def shutdown(self):
        self.xshut.value(0)

    def start(self, i2c):
        # power up the sensor and move it off the default address
        self.xshut.value(1)
        time.sleep_ms(10)
        if self.address != DEFAULT_TOF_ADDRESS:
            i2c.writeto_mem(DEFAULT_TOF_ADDRESS, 0x8A, bytes([self.address & 0x7F]))
            time.sleep_ms(10)
        self.device = VL53L0X(i2c, address=self.address)

    def read(self):
        # distance in mm
        return self.device.read()

# initialising TOF sensor running paramaters from the I2C bus unit  
# brings up all distance sensors on the shared I2C bus
def init_tof_sensors(i2c, sensors):
    for sensor in sensors:
        sensor.shutdown()
    time.sleep_ms(10)
    for sensor in sensors:
        sensor.start(i2c)
    found = i2c.scan()
# if sensors that have been scanned = true then it will retrun the scan of the i2c unit 
    for sensor in sensors:
        if sensor.address in found:
            print("Verified", sensor.name, "ToF sensor at", hex(sensor.address))
        else:
            print(sensor.name, "ToF sensor not found at", hex(sensor.address))
        

# reads the motor encoder
class Encoder:

    def __init__(self, pin_a, pin_b):
        self.pin_a = Pin(pin_a, Pin.IN, Pin.PULL_UP)
        self.pin_b = Pin(pin_b, Pin.IN, Pin.PULL_UP)
        self.ticks = 0
        self.pin_a.irq(trigger=Pin.IRQ_RISING | Pin.IRQ_FALLING, handler=self._on_edge)

    def _on_edge(self, pin):
        # if A and B match we're turning one way, otherwise the other way
        if self.pin_a.value() == self.pin_b.value():
            self.ticks += 1
        else:
            self.ticks -= 1

    def reset(self):
        self.ticks = 0


    def failcase(self): 
        for self in range(0):
            return False
        else:
            return True 
        

# one motor driven by a DRV8871 board
class Motor:

    def __init__(self, in1_pin, in2_pin, enc_a_pin, enc_b_pin, reversed=False):
        self.in1 = PWM(Pin(in1_pin))
        self.in2 = PWM(Pin(in2_pin))
        self.in1.freq(PWM_FREQ)
        self.in2.freq(PWM_FREQ)
        self.encoder = Encoder(enc_a_pin, enc_b_pin)
        self.reversed = reversed
        self.speed = 0
        self.stop()

    def set_speed(self, speed):
        # speed from -1.0 to 1.0, PWM on one input and the other held low
        speed = max(-1.0, min(1.0, speed))
        self.speed = speed
        duty = int(abs(speed) * MAX_DUTY)
        forward = (speed >= 0) != self.reversed
        if forward:
            self.in1.duty_u16(duty)
            self.in2.duty_u16(0)
        else:
            self.in1.duty_u16(0)
            self.in2.duty_u16(duty)

    def stop(self):
        # both inputs low lets the motor coast
        self.in1.duty_u16(0)
        self.in2.duty_u16(0)
        self.speed = 0

    def brake(self):
        # both inputs high brakes the motor
        self.in1.duty_u16(MAX_DUTY)
        self.in2.duty_u16(MAX_DUTY)
        self.speed = 0

# creating failcase for motor sensors 

    def failcase(self):
        for self in range(0):
            return False
        else:
            return True
        

class Buzzer: 
    #initialising buzzer beep 
    def buzzer_init(pin, PWM):
        if pin == True: 
            print('Output Voltage Detected:', pin)
            pin.OUT = 1 
        elif pin == False:
            print('Output Voltage Not Detected', pin)
            pin.OUT = 0      
        else: 
            return True

    def buzzer_beep(pin, PWM): 
        if pin == True and PWM == True:
            pin.out(3.3)            
            print('Buzzer activated')
        elif pin == False and PWM == False:
            print('Buzzer not Activated')
        else: 
            return False
#making sure fail case for buzzer dosnt affect i2c unit
    def buzzer_fail(pin, PWM):
        for i in range(pin):
            while pin.buzzer == True:
                if pin and PWM == True: 
                    return False
        if buzzer_work(pin, PWM):
            for i in range(pin):
                while pin.buzzer == False:
                    return True

# hardware setup
led = Pin("LED", Pin.OUT)

i2c = I2C(I2C_ID, sda=Pin(I2C_SDA_PIN), scl=Pin(I2C_SCL_PIN), freq=I2C_FREQ)

front_sensor = TofSensor("front", FRONT_XSHUT_PIN, FRONT_TOF_ADDRESS)
left_sensor = TofSensor("left", LEFT_XSHUT_PIN, LEFT_TOF_ADDRESS)
right_sensor = TofSensor("right", RIGHT_XSHUT_PIN, RIGHT_TOF_ADDRESS)
init_tof_sensors(i2c, [front_sensor, left_sensor, right_sensor])

left_motor = Motor(LEFT_MOTOR_IN1_PIN, LEFT_MOTOR_IN2_PIN, LEFT_ENCODER_A_PIN, LEFT_ENCODER_B_PIN)
right_motor = Motor(RIGHT_MOTOR_IN1_PIN, RIGHT_MOTOR_IN2_PIN, RIGHT_ENCODER_A_PIN, RIGHT_ENCODER_B_PIN,
                    reversed=True)

#creating test case for initialising LED on the board 
#
def blink_led(times=5, delay=1):
    # blink the onboard LED to check the board is alive
    for i in range(times):
        led.value(1)
        time.sleep(delay)
        led.value(0)
        time.sleep(delay)


def read_sensors():
    # distances in mm from front, left and right
    return front_sensor.read(), left_sensor.read(), right_sensor.read()


def print_status():
    # print every sensor and motor reading for debugging
    front, left, right = read_sensors()
    print("Sensors (mm) front:", front, "left:", left, "right:", right)
    print("Motors left speed:", left_motor.speed, "ticks:", left_motor.encoder.ticks,
          "right speed:", right_motor.speed, "ticks:", right_motor.encoder.ticks)


blink_led(3)

from machine import Pin
from machine import ADC
#initialising Sensors based on corresponding pins i found on Pico W  
front_sensor = ADC(26) 
left_sensor = ADC(27)
right_sensor = ADC(28)
import time 
# we are importing the Pin tool packages for Raspery Pico W which is 
# Micropythons hardware module and initialising
# variable led and passing a signal out with the Pin.OUT parameter
led = Pin("LED", Pin.OUT)
#turning LED on and off to check board works 
for i in range(5): 
    led.value(1)
    time.sleep(1)
    led.value(0)
#commenting this out as im just practicing 
# Set a motors direction Pin(n, Pin.OUT).value()
# Set motor speed = PWM(Pin(n)).duty_u16()
# Read a wall sensort ADC(n).read_u16()
#Time a turn or a pause = time.sleep_ms() 
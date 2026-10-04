from machine import Pin ,ADC 
import time 
#initialising Sensors based on corresponding pins i found on Pico W  
front_sensor = ADC(26) 
left_sensor = ADC(27)
right_sensor = ADC(28)

# we are importing the Pin tool packages for Raspery Pico W which is 
# Micropythons hardware module and initialising
# variable led and passing a signal out with the Pin.OUT parameter

led = Pin("LED", Pin.OUT)

#turning LED on and off to check board works 
for i in range(5): 
    led.value(1)
    time.sleep(1)
    led.value(0)
    time.sleep(1)
#commenting this out as im just practicing 
# Set a motors direction Pin(n, Pin.OUT).value()
# Set motor speed = PWM(Pin(n)).duty_u16()
# Read a wall sensort ADC(n).read_u16()
#Time a turn or a pause = time.sleep_ms() 

#creating a function which when called will return the data in all sensors 
def read_sensors():
    return front_sensor.read_u16(), left_sensor.read_u16(), right_sensor.read_u16()

#creating read motors function to allow for debugging capabilities 
def read_motors(): 
    return 




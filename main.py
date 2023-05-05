import utime
import machine
from machine import Pin, PWM, ADC
from picozero import Robot

# Sensor Pins
left_sensor = machine.ADC(26)
middle_sensor = machine.ADC(27)
right_sensor = machine.ADC(28)
# 
# button = machine.Pin(20, machine.Pin.IN, machine.Pin.PULL_UP)
# button2 = machine.Pin(21, machine.Pin.IN, machine.Pin.PULL_UP)
# 
# # Motor Pins
# left_motor1_pin = machine.PWM(machine.Pin(9))
# left_motor2_pin = machine.PWM(machine.Pin(8))
# right_motor1_pin = machine.PWM(machine.Pin(11))
# right_motor2_pin = machine.PWM(machine.Pin(10))


robot = Robot(left=(8, 9), right=(10, 11),pwm=True)
# robot.forward(speed=0.5)
# utime.sleep(5)
# robot.backward(speed=0.5)
# utime.sleep(5)
# robot.left(speed=0.5)
# utime.sleep(5)
# robot.right(speed=0.5)
# utime.sleep(5)
# robot.stop()

while True:
    left_sensor_val = left_sensor.read_u16()
    
    middle_sensor_val = middle_sensor.read_u16()
    
    right_sensor_val = right_sensor.read_u16()
    
    print("left sensor",left_sensor_val)
    print("middle sensor",middle_sensor_val)
    print("right sensor",right_sensor_val)
    
    #go forward if only the middle sensor sees the line 
    if left_sensor_val > (13000) and middle_sensor_val < (13000) and right_sensor_val > (13000) :
        robot.forward(speed=0.3)
        
    #turn left if the left and middle sensor see the line
    elif left_sensor_val < (13000) and middle_sensor_val < (13000) and right_sensor_val > (13000) :
        robot.left(speed=0.3)
        
    #extreme case of sharp turn to the left
    elif left_sensor_val < (13000) and middle_sensor_val > (13000) and right_sensor_val > (13000) :
        robot.left(speed=0.3)
    
    #turn right if the right and middle sensor see the line
    elif left_sensor_val > (13000) and middle_sensor_val < (13000) and right_sensor_val < (13000) :
        robot.right(speed=0.3)
        
    #extreme case of sharp turn to the right
    elif left_sensor_val > (13000) and middle_sensor_val > (13000) and right_sensor_val < (13000) :
        robot.right(speed=0.3)
        
    #stop the robot if all three sensors see white at the same time
    elif left_sensor_val > (13000) and middle_sensor_val > (13000) and right_sensor_val > (13000) :
        robot.stop()
        
    #stop if all sensors see black at the same time
    elif left_sensor_val < (13000) and middle_sensor_val < (13000) and right_sensor_val < (13000) :
        robot.stop()
    utime.sleep(0.1)
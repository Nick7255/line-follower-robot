import utime
import machine
from machine import Pin, PWM, ADC
from picozero import Robot

# Sensor Pins
left_sensor = machine.ADC(26)
middle_sensor = machine.ADC(27)
right_sensor = machine.ADC(28)

robot = Robot(left=(8, 9), right=(10, 11),pwm=True)

while True:
    left_sensor_val = left_sensor.read_u16()
    
    middle_sensor_val = middle_sensor.read_u16()
    
    right_sensor_val = right_sensor.read_u16()
    
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

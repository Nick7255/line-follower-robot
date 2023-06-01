from machine import Pin, ADC, PWM
import utime

# Define pins for IR sensors
sensor_left = ADC(Pin(26))
sensor_middle = ADC(Pin(27))
sensor_right = ADC(Pin(28))

# Define pins for motors
motor_left_dir = Pin(8, Pin.OUT)
motor_left_pwm = PWM(Pin(9))
motor_right_dir = Pin(10, Pin.OUT)
motor_right_pwm = PWM(Pin(11))

# Define motor direction constants
FORWARD = 1
REVERSE = 0

# Set initial PWM frequency
motor_left_pwm.freq(1000)
motor_right_pwm.freq(1000)

# PID constants
Kp = 1
Ki = 0
Kd = 0

# PID variables
integral = 0
previous_error = 0

# stop motors
def stop():
    motor_left_pwm.duty_u16(0)
    motor_right_pwm.duty_u16(0)

def get_error(left,middle,right):
   
    # Calculate error
    if middle < 12000:    # Over the line
        error = 0
    elif left < 12000:   # Left of the line
        error = -10
    elif right < 12000:  # Right of the line
        error = 10
    else:               # No line detected
        error = 0

    return error

while True:
    # Get sensor readings
    left = sensor_left.read_u16()
    middle = sensor_middle.read_u16()
    right = sensor_right.read_u16()
    
    # check if all sensors see black or white at the same time
    if(left >30000 and middle >30000 and right >30000) :
        stop()
    elif (left <30000 and middle <30000 and right <30000) :
        stop()
    else:
        # Calculate PID
        error = get_error(left,middle,right)
        integral += error
        derivative = error - previous_error
        turn = Kp*error + Ki*integral + Kd*derivative
        previous_error = error

        # Update motor speeds
        motor_left_pwm.duty_u16(max(min(65025, 32512 + turn), 0))
        motor_right_pwm.duty_u16(max(min(65025, 32512 - turn), 0))

        # Motor direction control
        motor_left_dir.value(FORWARD if turn >= 0 else REVERSE)
        motor_right_dir.value(FORWARD if turn <= 0 else REVERSE)



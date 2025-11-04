import time
import board
import pwmio
from adafruit_motor import motor
import digitalio
import socket
import threading
from gpiozero import RotaryEncoder
from simple_pid import PID
from adafruit_mcp3xxx.mcp3008 import MCP3008
from adafruit_mcp3xxx.analog_in import AnalogIn
import busio
import adafruit_tcs34725

emergency_stop_flag = False
emergency_stop_pi = digitalio.DigitalInOut(board.D16)
emergency_stop_pi.direction = digitalio.Direction.INPUT
emergency_stop_pi.pull = digitalio.Pull.UP

distance_stop_flag = False

motor1_pwmA = pwmio.PWMOut(board.D20, frequency=1000)
motor1_pwmB = pwmio.PWMOut(board.D21, frequency=1000)
motor2_pwmA = pwmio.PWMOut(board.D26, frequency=1000)
motor2_pwmB = pwmio.PWMOut(board.D19, frequency=1000)

motor1 = motor.DCMotor(motor1_pwmA, motor1_pwmB)
motor2 = motor.DCMotor(motor2_pwmA, motor2_pwmB)

motor1.throttle = 0
motor2.throttle = 0


PULSES_PER_ROTATION = 700
encoder1 = RotaryEncoder(a=23, b=24, max_steps=0)
encoder2 = RotaryEncoder(a=18, b=13, max_steps=0)

i2c = busio.I2C(board.SCL, board.SDA)

rgb_sensor = adafruit_tcs34725.TCS34725(i2c)
rgb_sensor.integration_time = 40
rgb_sensor.gain = 4


spi = busio.SPI(clock=board.SCK, MOSI=board.MOSI, MISO=board.MISO)
cs = digitalio.DigitalInOut(board.D5)
mcp = MCP3008(spi, cs)
channel = AnalogIn(mcp, 0)

target_speed = 60
pid_left = PID(0.008, 0.003, 0.0005, setpoint=target_speed)
pid_right = PID(0.008, 0.003, 0.0005, setpoint=target_speed)
pid_left.output_limits = (-1, 1)
pid_right.output_limits = (-1, 1)

HOST = ''
PORT = 55533

slope = -0.0294
intercept = 2.961

def compute_distance(voltage, slope, intercept):
    distance = (voltage - intercept) / slope
    return distance

def get_distance():
    voltage = channel.voltage
    distance = compute_distance(voltage, slope, intercept)
    return distance

def get_distance_thread():
    global pid_left, pid_right, distance_stop_flag
    last_distance = 0
    last_voltage = channel.voltage
    voltage_threshold = 0.5
    while True:
        distance = get_distance()
        current_voltage = channel.voltage
        if (22<= distance <= 25 and pid_left.setpoint > 0 and pid_right.setpoint > 0) or abs(current_voltage - last_voltage) > voltage_threshold:
            distance_stop_flag = True
        elif distance_stop_flag and distance > 30:
            distance_stop_flag = False
            continue
        
        last_distance = distance
        last_voltage = current_voltage
        time.sleep(0.5)


def debounce_button(pin, debounce_time=0.05):
    current_state = pin.value
    time.sleep(debounce_time)
    return current_state == pin.value


def check_emergency_stop():
    global emergency_stop_flag, pid_left, pid_right
    while True:
        if debounce_button(emergency_stop_pi):
            if not emergency_stop_pi.value:
                print("Emergency stop activated!")
                motor1.throttle = 0
                motor2.throttle = 0
                pid_left.setpoint = 0
                pid_right.setpoint = 0
                emergency_stop_flag = True
            else:
                emergency_stop_flag = False
        time.sleep(0.1)

def color_detection():
    #while True:
    # r, g, b, c = rgb_sensor.color_raw
        
    # if r<100 and g<100 and b<100:
    #     return "Black"
    # elif r>800:
    #     return "Orange"
    # elif r<210 and g<300 and b>140:
    #     return "Blue"
    # else:
    #     return "unknown"
    
    temp = rgb_sensor.color_temperature
    if 1500<temp<2500:
        return "Orange"
    elif 2750<temp<4000:
        return "Black"
    elif 4500<temp<6000:
        return "Blue"
        

def go_forward():
    pid_left.setpoint = target_speed
    pid_right.setpoint = target_speed
    
def go_right(errorticks):
    if errorticks < 10:
        pid_left.setpoint = target_speed
        pid_right.setpoint = target_speed * 0.8
    elif errorticks >= 10:
        pid_left.setpoint = target_speed
        pid_right.setpoint = target_speed * max(0.1, 1 - 0.1 * errorticks)

def go_left(errorticks):
    if errorticks < 10:
        pid_left.setpoint = target_speed * 0.8
        pid_right.setpoint = target_speed
    elif errorticks >= 10:
        pid_left.setpoint = target_speed * max(0.1, 1 - 0.1 * errorticks)
        pid_right.setpoint = target_speed

def leader_func():
    print("Leader mode")
    global pid_left, pid_right, target_speed
    error_ticks = 0
    last_color = "unknown"
    color = color_detection()
    if color != "unknown":
        last_color = color
        
    while True:
        color = color_detection()
        if distance_stop_flag:
            pid_left.setpoint = 0
            pid_right.setpoint = 0
            continue
        if color == "Black":
            last_color = color
            error_ticks = 0
            go_forward()
        elif color == "Orange":
            last_color = color
            error_ticks += 2
            go_right(error_ticks)
        elif color == "Blue":
            last_color = color
            error_ticks += 2
            go_left(error_ticks)
        else:
            error_ticks += 2
            if last_color == "Black":
                go_forward()
            elif last_color == "Orange":
                error_ticks += 2
                go_right(error_ticks)
            elif last_color == "Blue":
                error_ticks += 2
                go_left(error_ticks)
                
def follower_func():
    print("Follower mode")
    global target_speed, pid_left, pid_right
    error_ticks = 0
    last_color = "unknown"
    color = color_detection()
    distance = get_distance()
    if color != "unknown":
        last_color = color
        
    while True:
        color = color_detection()
        distance = get_distance()
        if 20< distance < 30:
            target_speed = 25
        elif 30 <= distance < 60:
            target_speed += 4
            if target_speed > 45:
                target_speed = 45
        else: 
            target_speed = 25
            
        if distance_stop_flag:
            pid_left.setpoint = 0
            pid_right.setpoint = 0
            continue
        
        if color == "Black":
            last_color = color
            error_ticks = 0
            go_forward()
        elif color == "Orange":
            last_color = color
            error_ticks += 2
            go_right(error_ticks)
        elif color == "Blue":
            last_color = color
            error_ticks += 2
            go_left(error_ticks)
        else:
            error_ticks += 2
            if last_color == "Black":
                go_forward()
            elif last_color == "Orange":
                error_ticks += 2
                go_right(error_ticks)
            elif last_color == "Blue":
                error_ticks += 2
                go_left(error_ticks)

def main():
    global target_speed
    target_speed = 0
    leader = False
    follower = False
    print("Starting server")
    distance = get_distance()
    print(f"Distance: {distance}")
    target_speed = 45
    if 20< distance < 50:
        follower = True
    else:
        leader = True
    try:
        if leader:
            leader_func()
        elif follower:
            follower_func()
    except KeyboardInterrupt:
        motor1.throttle = 0
        motor2.throttle = 0
        pid_left.setpoint = 0
        pid_right.setpoint = 0
        print("Exiting program")
        exit()

def get_motor_speed(encoder, time_interval, last_pulses, last_time):
    pulses = encoder.steps
    current_time = time.time() * 1000

    elapsed_time = current_time - last_time
    if elapsed_time >= time_interval:
        pps = (pulses - last_pulses) / (elapsed_time / 1000.0)
        rpm = (pps * 60) / PULSES_PER_ROTATION
        return rpm, pulses, current_time
    return None, last_pulses, last_time


def set_motor_throttle(motor, throttle):
    motor.throttle = throttle

def pid_control_loop():
    global pid_left, pid_right, target_speed, emergency_stop_flag
    time_interval = 100
    last_time_left = time.time() * 1000
    last_time_right = time.time() * 1000
    last_pulses_left = 0
    last_pulses_right = 0

    while True:
        if emergency_stop_flag:
            target_speed = 0
            time.sleep(0.1)
            continue

        speed_left, last_pulses_left, last_time_left = get_motor_speed(encoder1, time_interval, last_pulses_left, last_time_left)
        speed_right, last_pulses_right, last_time_right = get_motor_speed(encoder2, time_interval, last_pulses_right, last_time_right)

        if speed_left is not None and speed_right is not None:
            #print(f"Speed Left: {speed_left} RPM, Speed Right: {speed_right} RPM")

            throttle_left = pid_left(speed_left)
            throttle_right = pid_right(speed_right)
            #print(f"Throttle Left: {throttle_left}, Throttle Right: {throttle_right}")

            set_motor_throttle(motor1, throttle_left)
            set_motor_throttle(motor2, throttle_right)

        time.sleep(time_interval / 1000.0)

if __name__ == "__main__":
    # emergency_thread = threading.Thread(target=check_emergency_stop)
    # emergency_thread.daemon = True
    # emergency_thread.start()
    try:

        pid_thread = threading.Thread(target=pid_control_loop)
        pid_thread.daemon = True
        pid_thread.start()
        
        emergency_thread = threading.Thread(target=get_distance_thread)
        emergency_thread.daemon = True
        emergency_thread.start()

        main()
    
    except KeyboardInterrupt:
        motor1.throttle = 0
        motor2.throttle = 0
        set_motor_throttle(motor1, 0)
        set_motor_throttle(motor2, 0)
        pid_left.setpoint = 0
        pid_right.setpoint = 0
        print("Exiting program")
        exit()
import pid
import time
import board
import pwmio
from gpiozero import Motor, RotaryEncoder, Robot

# ==== Fixed PID gains for tuning ====
Kp = 1.05
Ki = 0.9
Kd = 0.01

# ==== CONFIGURATION =====

AIN1_PIN = board.D12 # right motor: forward
AIN2_PIN = board.D18 # right motor: backward
BIN1_PIN = board.D13 # left motor: forward
BIN2_PIN = board.D19 # left motor: backward

# GPIO pins for encoders
ENCODER_RIGHTC1_PIN = 14
ENCODER_RIGHTC2_PIN = 15
ENCODER_LEFTC1_PIN = 21
ENCODER_LEFTC2_PIN = 20

PWM_FREQ = 1200  # PWM frequency in Hz
COUNTS_PER_REV = 700  # Number of encoder counts per wheel revolution
time_interval = 0.1  # Time interval  for speed calculation in seconds
target_rpm = 100  # starting target

pwm_AIN1 = pwmio.PWMOut(AIN1_PIN, frequency=PWM_FREQ)
pwm_AIN2 = pwmio.PWMOut(AIN2_PIN, frequency=PWM_FREQ)
pwm_BIN1 = pwmio.PWMOut(BIN1_PIN, frequency=PWM_FREQ)
pwm_BIN2 = pwmio.PWMOut(BIN2_PIN, frequency=PWM_FREQ)

encoder_motor_right = RotaryEncoder(ENCODER_RIGHTC1_PIN, ENCODER_RIGHTC2_PIN, max_steps=0)
encoder_motor_left =  RotaryEncoder(ENCODER_LEFTC1_PIN, ENCODER_LEFTC2_PIN, max_steps=0)

pid_right = pid.PID(Kp, Ki, Kd, setpoint=target_rpm)
pid_left  = pid.PID(Kp, Ki, Kd, setpoint=target_rpm)
pid_right.sample_time = time_interval
pid_left.sample_time = time_interval

pid_right.output_limits = (-100, 100)
pid_left.output_limits = (-100, 100)

def calculate_rpm(encoder, interval):
    """Calculate speed based on encoder counts over a time interval."""
    steps = encoder.steps
    rpm = (steps / COUNTS_PER_REV) * (60 / interval)
    encoder.steps = 0  # Reset the encoder count after reading
    return rpm

def set_motor_speed(pwm1, pwm2, speed):
    if speed > 0:
        pwm1.duty_cycle = int(abs(speed) * 65535 / 100)
        pwm2.duty_cycle = 0
    elif speed < 0:
        pwm1.duty_cycle = 0
        pwm2.duty_cycle = int(abs(speed) * 65535 / 100)
    else:
        pwm1.duty_cycle = 0
        pwm2.duty_cycle = 0


def main():
    while True:

        left_rpm = calculate_rpm(encoder_motor_left, time_interval)
        right_rpm = calculate_rpm(encoder_motor_right, time_interval)

        left_out = pid_left(left_rpm)
        right_out = pid_right(right_rpm)

        set_motor_speed(pwm_BIN1, pwm_BIN2, left_out)
        set_motor_speed(pwm_AIN1, pwm_AIN2, right_out)

        # Debug logging
        print(f"Left RPM: {left_rpm:.1f}, Output: {left_out:.1f} | Right RPM: {right_rpm:.1f}, Output: {right_out:.1f}")

        time.sleep(time_interval)


if __name__ == "__main__":
    main()





import pid
import time
import board
import pwmio
from gpiozero import Motor, RotaryEncoder

# OLED imports
import busio
from adafruit_ssd1306 import SSD1306_I2C
from PIL import Image, ImageDraw, ImageFont


# --------------- Constants / Pins ---------------
# GPIO pins for motor control
AIN1_PIN = board.D18
AIN2_PIN = board.D12
BIN1_PIN = board.D19
BIN2_PIN = board.D13

# GPIO pins for encoders
ENCODER_RIGHTC1_PIN = 15
ENCODER_RIGHTC2_PIN = 14
ENCODER_LEFTC1_PIN = 21
ENCODER_LEFTC2_PIN = 20

PWM_FREQ = 1000  # PWM frequency in Hz
COUNTS_PER_REV = 700  # Number of encoder counts per wheel revolution
time_interval = 0.1  # Time interval for speed calculation in seconds

# (1000 / 700) * 60 = 85.5 RPM for target rpm
target_rpm = 85.5

# Speed adjustment threshold
speed_threshold_upper_rpm = 10 # Above this value, reduce speed
speed_threshold_lower_rpm = 10 # Below this value, increase speed

# --------------- Hardware setup ---------------
# Create PWMOut instances for motors
pwm_AIN1 = pwmio.PWMOut(AIN1_PIN, frequency=PWM_FREQ)
pwm_AIN2 = pwmio.PWMOut(AIN2_PIN, frequency=PWM_FREQ)
pwm_BIN1 = pwmio.PWMOut(BIN1_PIN, frequency=PWM_FREQ)
pwm_BIN2 = pwmio.PWMOut(BIN2_PIN, frequency=PWM_FREQ)

# Set up the encoders with gpiozero's RotaryEncoder
encoder_right = RotaryEncoder(ENCODER_RIGHTC1_PIN, ENCODER_RIGHTC2_PIN, max_steps=0)
encoder_left =  RotaryEncoder(ENCODER_LEFTC1_PIN, ENCODER_LEFTC2_PIN, max_steps=0)

# Define PID controllers for each motor
pid_right = PID(Kp = 0, Ki = 0, Kd = 0, setpoint=target_rpm)
pid_left  = PID(Kp = 0, Ki = 0, Kd = 0, setpoint=target_rpm)

pid_right.sample_time = time_interval  # Update every 0.1 seconds
pid_left.sample_time = time_interval  # Update every 0.1 seconds

# Set PID output limits to match PWM duty cycle range
pid_right.output_limits = (-100, 100)
pid_left.output_limits = (-100, 100)

# OLED setup
i2c = busio.I2C(board.SCL, board.SDA)
oled = SSD1306_I2C(128, 64, i2c)
image = Image.new("1", (oled.width, oled.height))
draw = ImageDraw.Draw(image)
font = ImageFont.load_default()

# Baseline values for Kp, Ki, Kd (tune these!)
Kp_baseline = 0.5
Ki_baseline = 0.0
Kd_baseline = 0.0
Ki_boosted   = 0.0  # e.g., 1.5 * Ki_baseline for uphill
Kd_increased = 0.0  # e.g., 1.5 * Kd_baseline for downhill

# --------------- Helpers ---------------
def calculate_rpm(encoder, interval):
    """Calculate speed based on encoder counts over a time interval."""
    counts = encoder.steps
    encoder.steps = 0  # Reset the encoder count after reading
    counts_per_sec = counts / interval  # Speed in counts per second
    rpm = (counts_per_sec / COUNTS_PER_REV) * 60  # Convert to RPM
    return rpm

def set_motor_speed(motor_pwm1, motor_pwm2, speed):
    """Set motor speed using PWM signals."""
    if speed > 0:
        motor_pwm1.duty_cycle = int(abs(speed) * 65535 / 100)  # Scale to 16-bit
        motor_pwm2.duty_cycle = 0
    elif speed < 0:
        motor_pwm1.duty_cycle = 0
        motor_pwm2.duty_cycle = int(abs(speed) * 65535 / 100)  # Scale to 16-bit
    else:
        motor_pwm1.duty_cycle = 0
        motor_pwm2.duty_cycle = 0

def draw_oled(left_rpm, right_rpm, target_rpm):
    """Display RPM values on the OLED."""
    draw.rectangle((0, 0, oled.width, oled.height), outline=0, fill=0)
    draw.text((0, 0),  f"Target: {target_rpm:5.1f} RPM", font=font, fill=255)
    draw.text((0, 16), f"Left:   {left_rpm:5.1f} RPM",   font=font, fill=255)
    draw.text((0, 32), f"Right:  {right_rpm:5.1f} RPM",  font=font, fill=255)
    # Simple bar graph (optional)
    # scale bars to 0..2*target for visualization
    max_rpm = max(1.0, 2.0 * target_rpm)
    lx = int(min(oled.width-4, (left_rpm / max_rpm) * (oled.width-4)))
    rx = int(min(oled.width-4, (right_rpm / max_rpm) * (oled.width-4)))
    draw.rectangle((0, 48, lx, 55), outline=1, fill=1)   # left bar
    draw.rectangle((0, 56, rx, 63), outline=1, fill=1)   # right bar
    oled.image(image)
    oled.show()


def main():
    try:
        while True:
            left_rpm = calculate_rpm(encoder_left, time_interval)
            right_rpm = calculate_rpm(encoder_right, time_interval)

            # Adjust left motor based on speed
            if left_rpm> target_rpm + speed_threshold_upper_rpm: # Going downhill (too fast)
                print("LEFT MOTOR: Higher Kd for downhill")
                pid_left.Kp = Kp_baseline
                pid_left.Ki = Ki_baseline
                pid_left.Kd = Kd_increased

            elif left_rpm < target_rpm - speed_threshold_lower_rpm: # Going uphill (too slow)
                print("LEFT MOTOR: Boosted Ki for uphill")
                pid_left.Kp = Kp_baseline
                pid_left.Ki = Ki_boosted
                pid_left.Kd = Kd_baseline

            else: # Flat terrain
                print("LEFT MOTOR: Stable speed")
                pid_left.Kp = Kp_baseline
                pid_left.Ki = Ki_baseline
                pid_left.Kd = Kd_baseline

            # Adjust right motor based on speed
            if right_rpm > target_rpm + speed_threshold_upper_rpm: # Going downhill (too fast)
                print("RIGHT MOTOR: Higher Kd for downhill")
                pid_right.Kp = Kp_baseline
                pid_right.Ki = Ki_baseline
                pid_right.Kd = Kd_increased

            elif right_rpm < target_rpm - speed_threshold_lower_rpm: # Going uphill (too slow)
                print("RIGHT MOTOR: Boosted Ki for uphill")
                pid_right.Kp = Kp_baseline
                pid_right.Ki = Ki_boosted
                pid_right.Kd = Kd_baseline

            else: # Flat terrain
                print("RIGHT MOTOR: Stable speed")
                pid_right.Kp = Kp_baseline
                pid_right.Ki = Ki_baseline
                pid_right.Kd = Kd_baseline

            # Compute PID outputs
            left_output = pid_left(left_rpm)
            right_output = pid_right(right_rpm)

            # Set motor speeds
            set_motor_speed(pwm_AIN1, pwm_AIN2, left_output)
            set_motor_speed(pwm_BIN1, pwm_BIN2, right_output)

            # Debugging output
            print(f"Left Speed: {left_rpm:.2f} RPM")
            print(f"Right Speed: {right_rpm:.2f} RPM")

            draw_oled(left_rpm, right_rpm, target_rpm)

            time.sleep(time_interval)

    except KeyboardInterrupt:
        # Graceful stop
        set_motor_speed(pwm_AIN1, pwm_AIN2, 0)
        set_motor_speed(pwm_BIN1, pwm_BIN2, 0)
        print("\nStopped.")

if __name__ == "__main__":
    main()

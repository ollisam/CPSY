import pid
import time
import board
import pwmio
from gpiozero import RotaryEncoder

# OLED imports
import busio
from adafruit_ssd1306 import SSD1306_I2C
from PIL import Image, ImageDraw, ImageFont

# ---------------- Pin constants you provided ----------------
LEFT_MOTOR_PINS  = ("GPIO19", "GPIO13")   # left IN1, IN2
RIGHT_MOTOR_PINS = ("GPIO18", "GPIO12")   # right IN1, IN2

LEFT_ENCODER_PINS  = ("GPIO21", "GPIO20") # (A, B)
RIGHT_ENCODER_PINS = ("GPIO14", "GPIO15") # (A, B)

# ---------------- Helpers to convert pin strings ----------------
def to_board_pin(gpio_str: str):
    """Convert 'GPIO19' -> board.D19 (for CircuitPython/PWM)."""
    num = int(gpio_str.replace("GPIO", ""))
    return getattr(board, f"D{num}")

def to_bcm_int(gpio_str: str) -> int:
    """Convert 'GPIO19' -> 19 (for gpiozero encoders)."""
    return int(gpio_str.replace("GPIO", ""))

# ---------------- Constants / Timing / Targets ----------------
PWM_FREQ = 1000                 # Hz
COUNTS_PER_REV = 702            # your measured CPR
time_interval = 0.1             # s (10 Hz loop)

# (1000 / 702) * 60 ≈ 85.5 RPM
target_rpm = 85.5

# Thresholds in RPM
speed_threshold_upper_rpm = 10.0
speed_threshold_lower_rpm = 10.0

# ---------------- Hardware setup ----------------
# Map motor pins (use neutral names to avoid A/B confusion)
LEFT_PWM1_PIN  = to_board_pin(LEFT_MOTOR_PINS[0])
LEFT_PWM2_PIN  = to_board_pin(LEFT_MOTOR_PINS[1])
RIGHT_PWM1_PIN = to_board_pin(RIGHT_MOTOR_PINS[0])
RIGHT_PWM2_PIN = to_board_pin(RIGHT_MOTOR_PINS[1])

pwm_LEFT1  = pwmio.PWMOut(LEFT_PWM1_PIN,  frequency=PWM_FREQ)
pwm_LEFT2  = pwmio.PWMOut(LEFT_PWM2_PIN,  frequency=PWM_FREQ)
pwm_RIGHT1 = pwmio.PWMOut(RIGHT_PWM1_PIN, frequency=PWM_FREQ)
pwm_RIGHT2 = pwmio.PWMOut(RIGHT_PWM2_PIN, frequency=PWM_FREQ)

# Encoders (gpiozero expects BCM ints, order is (A, B))
ENC_L_A = to_bcm_int(LEFT_ENCODER_PINS[0])
ENC_L_B = to_bcm_int(LEFT_ENCODER_PINS[1])
ENC_R_A = to_bcm_int(RIGHT_ENCODER_PINS[0])
ENC_R_B = to_bcm_int(RIGHT_ENCODER_PINS[1])

encoder_left  = RotaryEncoder(ENC_L_A, ENC_L_B, max_steps=0)
encoder_right = RotaryEncoder(ENC_R_A, ENC_R_B, max_steps=0)

# OLED setup
i2c  = busio.I2C(board.SCL, board.SDA)
oled = SSD1306_I2C(128, 64, i2c)
image = Image.new("1", (oled.width, oled.height))
draw = ImageDraw.Draw(image)
font = ImageFont.load_default()

# ---------------- PID setup ----------------
pid_left  = pid.PID(Kp=0.5, Ki=0.0, Kd=0.0, setpoint=target_rpm)
pid_right = pid.PID(Kp=0.5, Ki=0.0, Kd=0.0, setpoint=target_rpm)
pid_left.sample_time = pid_right.sample_time = time_interval
pid_left.output_limits = pid_right.output_limits = (-100, 100)

# Optional adaptive deltas
Kp_baseline = 0.5
Ki_baseline = 0.0
Kd_baseline = 0.0
Ki_boosted   = 0.0
Kd_increased = 0.0

# ---------------- Functions ----------------
def calculate_rpm(encoder, interval):
    counts = encoder.steps
    encoder.steps = 0  # reset after reading
    cps = counts / interval
    rpm = (cps / COUNTS_PER_REV) * 60.0
    return rpm

def set_motor_speed(motor_pwm1, motor_pwm2, speed_percent):
    """speed_percent in [-100, 100]; positive uses pwm1, negative uses pwm2."""
    cmd = max(-100.0, min(100.0, float(speed_percent)))
    duty = int(abs(cmd) * 65535 / 100.0)
    if cmd > 0:
        motor_pwm1.duty_cycle = duty
        motor_pwm2.duty_cycle = 0
    elif cmd < 0:
        motor_pwm1.duty_cycle = 0
        motor_pwm2.duty_cycle = duty
    else:
        motor_pwm1.duty_cycle = 0
        motor_pwm2.duty_cycle = 0

def draw_oled(left_rpm, right_rpm, target_rpm):
    draw.rectangle((0, 0, oled.width, oled.height), outline=0, fill=0)
    draw.text((0, 0),  f"Target: {target_rpm:5.1f} RPM", font=font, fill=255)
    draw.text((0, 16), f"Left:   {left_rpm:5.1f} RPM",   font=font, fill=255)
    draw.text((0, 32), f"Right:  {right_rpm:5.1f} RPM",  font=font, fill=255)
    max_rpm = max(1.0, 2.0 * target_rpm)
    lx = int(min(oled.width-4, (left_rpm  / max_rpm) * (oled.width-4)))
    rx = int(min(oled.width-4, (right_rpm / max_rpm) * (oled.width-4)))
    draw.rectangle((0, 48, lx, 55), outline=1, fill=1)
    draw.rectangle((0, 56, rx, 63), outline=1, fill=1)
    oled.image(image)
    oled.show()

# ---------------- Main ----------------
def main():
    try:
        while True:
            left_rpm  = calculate_rpm(encoder_left,  time_interval)
            right_rpm = calculate_rpm(encoder_right, time_interval)

            # Adaptive tweaks (RPM domain)
            if left_rpm > target_rpm + speed_threshold_upper_rpm:
                pid_left.Kp, pid_left.Ki, pid_left.Kd = Kp_baseline, Ki_baseline, Kd_increased
            elif left_rpm < target_rpm - speed_threshold_lower_rpm:
                pid_left.Kp, pid_left.Ki, pid_left.Kd = Kp_baseline, Ki_boosted,  Kd_baseline
            else:
                pid_left.Kp, pid_left.Ki, pid_left.Kd = Kp_baseline, Ki_baseline, Kd_baseline

            if right_rpm > target_rpm + speed_threshold_upper_rpm:
                pid_right.Kp, pid_right.Ki, pid_right.Kd = Kp_baseline, Ki_baseline, Kd_increased
            elif right_rpm < target_rpm - speed_threshold_lower_rpm:
                pid_right.Kp, pid_right.Ki, pid_right.Kd = Kp_baseline, Ki_boosted,  Kd_baseline
            else:
                pid_right.Kp, pid_right.Ki, pid_right.Kd = Kp_baseline, Ki_baseline, Kd_baseline

            # PID outputs (in % duty)
            left_out  = pid_left(left_rpm)
            right_out = pid_right(right_rpm)

            # Drive motors (left uses LEFT PWMs, right uses RIGHT PWMs)
            set_motor_speed(pwm_LEFT1,  pwm_LEFT2,  left_out)
            set_motor_speed(pwm_RIGHT1, pwm_RIGHT2, right_out)

            # Debug
            print(f"Target {target_rpm:5.1f} RPM | "
                  f"L {left_rpm:5.1f} RPM -> {left_out:+6.1f}% | "
                  f"R {right_rpm:5.1f} RPM -> {right_out:+6.1f}%")

            draw_oled(left_rpm, right_rpm, target_rpm)
            time.sleep(time_interval)

    except KeyboardInterrupt:
        # Graceful stop
        set_motor_speed(pwm_LEFT1,  pwm_LEFT2,  0)
        set_motor_speed(pwm_RIGHT1, pwm_RIGHT2, 0)
        print("\nStopped.")

if __name__ == "__main__":
    main()

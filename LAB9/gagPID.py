import time
from collections import deque

# ====== USER HOOKS: wire these to your hardware ======
def read_reflectance() -> float:
    """
    Return reflectance/intensity from your color sensor as a float in [0, 1].
    If your sensor is RGB (e.g., TCS34725), convert to brightness or use a single channel.
    """
    # TODO: replace with real sensor read
    return 0.5

def set_motor_pwm(left: float, right: float):
    """
    left/right in [-1.0, 1.0]; map to your motor driver's PWM and direction pins.
    """
    # TODO: replace with real motor driver calls
    pass

def read_encoder_ticks() -> tuple[int,int]:
    """
    Return cumulative tick counts (left, right). Sign should reflect direction.
    """
    # TODO: replace with real encoder reads
    return (0, 0)

# ====== Helpers ======
def clamp(x, lo, hi): 
    return lo if x < lo else hi if x > hi else x

class LowPass:
    def __init__(self, alpha=0.2):
        self.a = alpha; self.y = 0.0; self.init = False
    def __call__(self, x):
        if not self.init:
            self.y = x; self.init = True
        else:
            self.y = self.a * x + (1 - self.a) * self.y
        return self.y

# ====== Line follower with PID steering ======
class LineFollower:
    def __init__(self,
                 Kp=1.6, Ki=0.0, Kd=0.05,
                 base_pwm=0.35,
                 max_pwm=0.8,
                 integral_limit=1.0,
                 loop_hz=80):
        self.Kp, self.Ki, self.Kd = Kp, Ki, Kd
        self.base_pwm = base_pwm
        self.max_pwm = max_pwm
        self.ilim = integral_limit
        self.dt = 1.0/loop_hz
        self.e_int = 0.0
        self.prev_e = 0.0
        self.d_lpf = LowPass(alpha=0.25)  # smooth derivative
        self.target = 0.5                  # set after calibration
        self.black, self.white = 0.2, 0.8 # set after calibration
        self.last_u = 0.0
        self.lost_counter = 0
        self.lost_limit = int(0.25 / self.dt)  # 250 ms

    def calibrate(self, samples=60):
        # Ask the user to place sensor on black, then white; or auto-scan while you move it.
        print("Calibrating... place on BLACK")
        time.sleep(1.0)
        blacks = [read_reflectance() for _ in range(samples)]
        self.black = sum(blacks)/len(blacks)

        print("Now place on WHITE")
        time.sleep(1.0)
        whites = [read_reflectance() for _ in range(samples)]
        self.white = sum(whites)/len(whites)

        self.target = 0.5*(self.black + self.white)
        print(f"Calibrated: black={self.black:.3f} white={self.white:.3f} target={self.target:.3f}")

    def step(self):
        r = read_reflectance()
        e = (self.target - r) / max(1e-6, (self.white - self.black))  # normalize error to ~[-1,1]

        # PID
        self.e_int += e * self.dt
        self.e_int = clamp(self.e_int, -self.ilim, self.ilim)        # anti-windup

        de = (e - self.prev_e) / self.dt
        de = self.d_lpf(de)                                          # filtered D
        u = self.Kp*e + self.Ki*self.e_int + self.Kd*de

        self.prev_e = e
        self.last_u = u

        # “Line lost” detection
        on_track = (self.black <= r <= self.white)
        if on_track:
            self.lost_counter = 0
        else:
            self.lost_counter += 1

        # Base speed adjustment if lost: slow and search with last turn
        base = self.base_pwm
        if self.lost_counter > self.lost_limit:
            base = 0.25 * self.base_pwm
            u = 0.6 if self.last_u >= 0 else -0.6  # slow spin towards last known side

        # Differential drive mix
        left = clamp(base - u, -self.max_pwm, self.max_pwm)
        right = clamp(base + u, -self.max_pwm, self.max_pwm)
        set_motor_pwm(left, right)

# ====== Optional: keep forward speed with encoder PI ======
class SpeedController:
    def __init__(self, ticks_per_rev, wheel_circum_m, Kp=0.4, Ki=0.1, loop_hz=20, base_pwm=0.35):
        self.tpr = ticks_per_rev
        self.circ = wheel_circum_m
        self.dt = 1.0/loop_hz
        self.Kp, self.Ki = Kp, Ki
        self.ei = 0.0
        self.target_mps = 0.25   # target linear speed
        self.base_pwm = base_pwm
        self.prev_ticks = None

    def update(self):
        ticks = read_encoder_ticks()
        if self.prev_ticks is None:
            self.prev_ticks = ticks
            return self.base_pwm

        dtl = (ticks[0] - self.prev_ticks[0])
        dtr = (ticks[1] - self.prev_ticks[1])
        self.prev_ticks = ticks
        # avg wheel revs over dt -> speed
        revs_avg = 0.5 * (dtl + dtr) / float(self.tpr)
        dist = revs_avg * self.circ
        v = dist / self.dt  # m/s

        e = self.target_mps - v
        self.ei += e * self.dt
        pwm_delta = self.Kp*e + self.Ki*self.ei
        self.base_pwm = clamp(self.base_pwm + pwm_delta, 0.1, 0.9)
        return self.base_pwm

# ====== Main ======
if __name__ == "__main__":
    follower = LineFollower(Kp=1.8, Ki=0.02, Kd=0.06, base_pwm=0.32, max_pwm=0.9, loop_hz=90)
    speed = SpeedController(ticks_per_rev=360, wheel_circum_m=0.21*3.1416, loop_hz=20, base_pwm=follower.base_pwm)

    follower.calibrate()

    next_line_t = time.time()
    next_speed_t = time.time()
    line_dt = follower.dt
    speed_dt = speed.dt

    try:
        while True:
            t = time.time()

            if t >= next_speed_t:
                follower.base_pwm = speed.update()
                next_speed_t += speed_dt

            if t >= next_line_t:
                follower.step()
                next_line_t += line_dt

            # small sleep to prevent busy loop
            time.sleep(0.001)

    except KeyboardInterrupt:
        set_motor_pwm(0.0, 0.0)

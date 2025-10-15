import time
from gpiozero import RotaryEncoder

# Use the same pins you already wired for the left encoder
ENCODER_C1_PIN = 21   # adjust if needed
ENCODER_C2_PIN = 20   # adjust if needed

# Set up encoder
encoder = RotaryEncoder(ENCODER_C1_PIN, ENCODER_C2_PIN, max_steps=0)

print("Rotate the wheel/output shaft EXACTLY ONE full turn by hand.")
print("Press Ctrl+C to stop if needed.")

# Reset steps to zero
encoder.steps = 0

try:
    # Give yourself some time to turn the wheel
    time.sleep(10)

    counts = encoder.steps
    print(f"Counts for one wheel revolution: {counts}")

except KeyboardInterrupt:
    print("\nStopped.")


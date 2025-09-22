# servo_test_en.py
import utime
from machine import Pin, PWM

# --- Settings ---
# Choose an unused GPIO pin from your project
SERVO_PIN = 21

# Standard servos use a 50Hz PWM frequency
PWM_FREQ = 50

# Pulse width range for the servo (in microseconds, us)
# These values might need fine-tuning to precisely match your servo's 0 and 180-degree points.
MIN_PULSE_US = 500  # Pulse width for 0 degrees
MAX_PULSE_US = 2500  # Pulse width for 180 degrees

# --- Initialize PWM ---
# Set SERVO_PIN as a PWM output
pwm = PWM(Pin(SERVO_PIN))
# Set the PWM frequency
pwm.freq(PWM_FREQ)


# --- Helper Function ---
def angle_to_duty(angle: int) -> int:
    """
    Converts an angle from 0-180 degrees to the Pico's PWM duty_u16 value (0-65535).
    """
    if not 0 <= angle <= 180:
        raise ValueError("Angle must be between 0 and 180")

    # Calculate the corresponding pulse width for the angle (us)
    pulse_width = MIN_PULSE_US + (angle / 180) * (MAX_PULSE_US - MIN_PULSE_US)

    # Convert the pulse width (us) to a duty cycle (0-65535)
    # Period T = 1 / 50Hz = 20ms = 20000us
    # duty = (pulse_width / T) * 65535
    duty = int((pulse_width / 20000) * 65535)
    return duty


# --- Main Program ---
try:
    print(f"Starting servo motor test (Pin: GP{SERVO_PIN})")
    print("The motor will move between 0, 90, and 180 degrees. Press Ctrl+C to stop.")

    while True:
        # Move to 0 degrees
        print("Position: 0 degrees")
        pwm.duty_u16(angle_to_duty(0))
        utime.sleep(2)

        # Move to 90 degrees
        print("Position: 90 degrees")
        pwm.duty_u16(angle_to_duty(90))
        utime.sleep(2)

        # Move to 180 degrees
        print("Position: 180 degrees")
        pwm.duty_u16(angle_to_duty(180))
        utime.sleep(2)

except KeyboardInterrupt:
    print("\nProgram stopped.")
finally:
    # When the program ends, turn off the PWM output
    pwm.deinit()

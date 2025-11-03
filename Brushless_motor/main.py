# 控制無刷馬達

from machine import Pin, PWM
import time

# --- Config ---
PWM_FREQ = 50  # ESC expects 50 Hz

# ESC signal pins
PWM_PIN1 = 15  # Motor 1 -> ESC1 signal
# PWM_PIN2 = 14  # Motor 2 -> ESC2 signal

# Throttle range
MIN_DUTY_US = 500  # stop
MAX_DUTY_US = 2500  # full throttle
CALIBRATION_PULSE_US = 2000
STOP_PULSE_US = 1000

# --- Init PWM ---
pwm1 = PWM(Pin(PWM_PIN1))
# pwm2 = PWM(Pin(PWM_PIN2))
pwm1.freq(PWM_FREQ)
# pwm2.freq(PWM_FREQ)


def us_to_duty(us):
    """Convert microseconds pulse to duty cycle (0–65535)."""
    period_us = (1.0 / PWM_FREQ) * 1_000_000
    duty_cycle = (us / period_us) * 65535
    return int(duty_cycle)


def set_speed(pwm, percent):
    """Set motor speed by percent (0–100)."""
    percent = max(0, min(100, percent))
    us = MIN_DUTY_US + (MAX_DUTY_US - MIN_DUTY_US) * percent / 100
    pwm.duty_u16(us_to_duty(us))


def calibrate_esc(pwm, motor_id):
    print(f"--- Calibrating ESC for Motor {motor_id} ---")
    pwm.duty_u16(us_to_duty(CALIBRATION_PULSE_US))
    time.sleep(3)
    pwm.duty_u16(us_to_duty(STOP_PULSE_US))
    print(f"Motor {motor_id} calibration done. Wait for ESC confirmation.")
    time.sleep(3)


def stop_motor(pwm, motor_id):
    print(f"--- Stopping Motor {motor_id} ---")
    pwm.duty_u16(us_to_duty(STOP_PULSE_US))


def main():
    try:
        # Calibrate both ESCs
        calibrate_esc(pwm1, 1)
        # calibrate_esc(pwm2, 2)
        print("Both ESCs calibrated.")
        print("Enter two motor speeds (0–100). Example: 30 70")
        print("Press Ctrl+C to exit.")

        while True:
            user_input = input("Set speeds (M1 M2): ")
            try:
                values = user_input
                v1 = int(values.split()[0])

                set_speed(pwm1, v1)
                # set_speed(pwm2, v1)

                print(f"Motor1 -> {v1}% , Motor2 -> {v1}%")

            except ValueError:
                print("Invalid input. Example: 20 50")

    except KeyboardInterrupt:
        print("\nProgram interrupted by user.")
    finally:
        stop_motor(pwm1, 1)
        # stop_motor(pwm2, 2)
        pwm1.deinit()
        # pwm2.deinit()
        print("Program ended. Both PWM outputs disabled.")


if __name__ == "__main__":
    main()

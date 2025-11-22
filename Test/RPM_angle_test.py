# -*- coding: utf-8 -*-
# File: pc_tuner_v2_2.py
# Function: PC Manual Tuning Tool (Linear Stop + Calibrated Servo + Fire Command)
# Usage: Control ESC speed and Fire AUX servo via Thonny Shell.

from machine import Pin, PWM
import utime
import sys

# ==================== Hardware Pinout ====================
ESC_PIN_TOP = 15
ESC_PIN_BOTTOM = 14

SERVO_PAN_PIN = 6
SERVO_AUX_PIN = 12

# ==================== Servo Calibration Config ====================
PWM_FREQ_SERVO = 50
SERVO_MIN_PULSE_US = 500
SERVO_MAX_PULSE_US = 2500

# Pan Servo Calibration (Keep your settings)
PAN_CALIBRATION_0_DEG = 55
PAN_CALIBRATION_90_DEG = 90
PAN_CALIBRATION_180_DEG = 150

# ==================== ESC Config ====================
PWM_FREQ_ESC = 55
ESC_MIN_DUTY_US = 1000
ESC_MAX_DUTY_US = 2000
ESC_STOP_PULSE_US = 1000

# Linear Stop Config
RAMP_STEP_PERCENT = 2
RAMP_DELAY_MS = 20

# ==================== PWM Init ====================
pwm_esc_top = PWM(Pin(ESC_PIN_TOP))
pwm_esc_bottom = PWM(Pin(ESC_PIN_BOTTOM))
pwm_esc_top.freq(PWM_FREQ_ESC)
pwm_esc_bottom.freq(PWM_FREQ_ESC)

pwm_pan = PWM(Pin(SERVO_PAN_PIN))
pwm_aux = PWM(Pin(SERVO_AUX_PIN))
pwm_pan.freq(PWM_FREQ_SERVO)
pwm_aux.freq(PWM_FREQ_SERVO)

# ==================== Servo Helper Functions ====================


def _servo_raw_to_duty(raw_val: float) -> int:
    """Convert 0-180 raw value to PWM duty"""
    pulse_width = SERVO_MIN_PULSE_US + (raw_val / 180) * (
        SERVO_MAX_PULSE_US - SERVO_MIN_PULSE_US
    )
    duty = int((pulse_width / (1_000_000 / PWM_FREQ_SERVO)) * 65535)
    return duty


def set_pan_angle(angle: float):
    """Set Pan Servo Angle using Calibration Values"""
    angle = max(-90, min(90, angle))
    in_min, in_max = -90, 90
    out_min, out_max = PAN_CALIBRATION_180_DEG, PAN_CALIBRATION_0_DEG
    raw_value = (angle - in_min) * (out_max - out_min) / (in_max - in_min) + out_min
    pwm_pan.duty_u16(_servo_raw_to_duty(raw_value))


def set_aux_angle(angle: float):
    """Set AUX Servo Angle (Standard 0-180)"""
    angle = max(0, min(180, angle))
    pwm_aux.duty_u16(_servo_raw_to_duty(angle))


def init_servos_calibrated():
    """Initialize servos to calibrated home position"""
    set_pan_angle(0)  # Calibrated 0 degrees
    set_aux_angle(0)  # Standard 0 degrees
    print(f" -> Servos initialized (Pan: 0 deg / Val: {PAN_CALIBRATION_0_DEG})")


def fire_shot():
    """Execute Fire Sequence: 0 -> 90 -> 0"""
    print(" -> Firing! (AUX 0 -> 90 -> 0)")
    set_aux_angle(90)
    utime.sleep(0.5)  # Wait for mechanism travel
    set_aux_angle(0)
    print(" -> Fire Complete.")


# ==================== ESC Helper Functions ====================


def _esc_us_to_duty(us):
    period_us = (1.0 / PWM_FREQ_ESC) * 1_000_000
    duty_cycle = (us / period_us) * 65535
    return int(duty_cycle)


def set_motor_speed(top_percent, bottom_percent, verbose=True):
    """Set Motor Speed (0-100)"""
    top_percent = max(0, min(100, top_percent))
    bottom_percent = max(0, min(100, bottom_percent))

    us_top = ESC_MIN_DUTY_US + (ESC_MAX_DUTY_US - ESC_MIN_DUTY_US) * top_percent / 100
    pwm_esc_top.duty_u16(_esc_us_to_duty(us_top))

    us_bottom = (
        ESC_MIN_DUTY_US + (ESC_MAX_DUTY_US - ESC_MIN_DUTY_US) * bottom_percent / 100
    )
    pwm_esc_bottom.duty_u16(_esc_us_to_duty(us_bottom))

    if verbose:
        print(f" -> Set: Top={int(top_percent)}%, Bot={int(bottom_percent)}%")


def stop_all_instant():
    pwm_esc_top.duty_u16(_esc_us_to_duty(ESC_STOP_PULSE_US))
    pwm_esc_bottom.duty_u16(_esc_us_to_duty(ESC_STOP_PULSE_US))
    print(" -> Motors STOPPED (Instant).")


def linear_stop(start_top, start_bot):
    """Ramp down speed gradually to 0"""
    print(f" -> Stopping linearly from T:{start_top}% B:{start_bot}% ...")
    curr_t = start_top
    curr_b = start_bot

    while curr_t > 0 or curr_b > 0:
        if curr_t > 0:
            curr_t -= RAMP_STEP_PERCENT
            if curr_t < 0:
                curr_t = 0
        if curr_b > 0:
            curr_b -= RAMP_STEP_PERCENT
            if curr_b < 0:
                curr_b = 0

        set_motor_speed(curr_t, curr_b, verbose=False)
        utime.sleep_ms(RAMP_DELAY_MS)

    stop_all_instant()


# ==================== Main Program ====================
def main():
    print("================================================")
    print("   PC ESC Tuner V2.2 (Soft Stop + Fire Cmd)     ")
    print("================================================")
    print("Instructions:")
    print(" 1. Input '50'      -> Set BOTH motors to 50%")
    print(" 2. Input '60 80'   -> Top 60%, Bottom 80%")
    print(" 3. Input 'fire'    -> Trigger AUX Servo (f)")
    print(" 4. Input 'stop'    -> Soft Stop (Linear)")
    print(" 5. Input 'exit'    -> Exit")
    print("------------------------------------------------")

    init_servos_calibrated()

    print("Arming ESCs... Please wait 2 seconds.")
    stop_all_instant()
    utime.sleep(2)
    print("ESCs Armed! Ready.\n")

    current_top = 0
    current_bot = 0

    while True:
        try:
            user_input = (
                input(f"Current[{current_top}/{current_bot}] Input: ").strip().lower()
            )

            # --- FIRE COMMAND ---
            if user_input in ["fire", "shoot", "f"]:
                fire_shot()
                continue

            # --- STOP COMMAND ---
            if user_input in ["stop", "s", "0"]:
                if current_top > 0 or current_bot > 0:
                    linear_stop(current_top, current_bot)
                else:
                    print(" -> Already stopped.")
                current_top = 0
                current_bot = 0
                continue

            # --- EXIT COMMAND ---
            if user_input in ["exit", "quit", "q"]:
                stop_all_instant()
                print("Program exited.")
                break

            if not user_input:
                continue

            parts = user_input.split()

            # Single value
            if len(parts) == 1:
                val = int(parts[0])
                current_top = val
                current_bot = val
                set_motor_speed(val, val)

            # Dual value
            elif len(parts) == 2:
                val_top = int(parts[0])
                val_bot = int(parts[1])
                current_top = val_top
                current_bot = val_bot
                set_motor_speed(val_top, val_bot)
            else:
                print("Error: Input 1 or 2 integers, or 'fire'.")

        except ValueError:
            print("Error: Invalid integer.")
        except KeyboardInterrupt:
            print("\nInterrupted by user.")
            stop_all_instant()
            break
        except Exception as e:
            print(f"Error: {e}")
            stop_all_instant()
            break


if __name__ == "__main__":
    main()

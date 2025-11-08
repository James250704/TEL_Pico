# 控制無刷馬達 (固定 70% 輸出版本)

from machine import Pin, PWM
import time

# --- Config ---
PWM_FREQ = 50  # ESC expects 50 Hz

# ESC signal pins
PWM_PIN1 = 15  # Motor 1 -> ESC1 signal
PWM_PIN2 = 14  # Motor 2 -> ESC2 signal

# Throttle range
MIN_DUTY_US = 1000  # stop
MAX_DUTY_US = 2000  # full throttle
CALIBRATION_PULSE_US = 2000
STOP_PULSE_US = 1000

# --- Init PWM ---
pwm1 = PWM(Pin(PWM_PIN1))
pwm2 = PWM(Pin(PWM_PIN2))
pwm1.freq(PWM_FREQ)
pwm2.freq(PWM_FREQ)


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
    """(此函式保留備用)"""
    print(f"--- Calibrating ESC for Motor {motor_id} ---")
    pwm.duty_u16(us_to_duty(CALIBRATION_PULSE_US))
    time.sleep(3)
    pwm.duty_u16(us_to_duty(STOP_PULSE_US))
    print(f"Motor {motor_id} calibration done. Wait for ESC confirmation.")
    time.sleep(3)


def stop_motor(pwm, motor_id):
    print(f"--- Stopping/Arming Motor {motor_id} ---")
    pwm.duty_u16(us_to_duty(STOP_PULSE_US))


def main():
    try:
        # (中文解釋：啟動時解鎖電調)
        time.sleep(2)
        print("Arming ESCs... (Sending STOP signal)")
        stop_motor(pwm1, 1)
        stop_motor(pwm2, 2)
        time.sleep(2)  # (中文解釋：給電調 2 秒鐘時間啟動)

        print("ESCs Armed. Ready to go.")
        # (中文解釋：電調已解鎖，準備就緒。)

        # --- MODIFICATION (修改點) ---
        # (中文解釋：設定固定速度 70% 並移除使用者輸入)
        fixed_speed = 50
        print(f"Setting fixed speed to {fixed_speed}% for both motors.")
        set_speed(pwm1, fixed_speed)
        set_speed(pwm2, fixed_speed)

        print("Motors running. Press Ctrl+C to stop.")
        # (中文解釋：馬達運行中。按下 Ctrl+C 來停止。)

        # (中文解釋：保持程式運行，直到使用者按下 Ctrl+C)
        while True:
            time.sleep(1)  # Keep script running

    except KeyboardInterrupt:
        print("\nProgram interrupted by user.")
        # (中文解釋：程式已被使用者中斷。)
    finally:
        # (中文解釋：停止馬達並清理 GPIO)
        stop_motor(pwm1, 1)
        stop_motor(pwm2, 2)
        pwm1.deinit()
        pwm2.deinit()
        print("Program ended. Both PWM outputs disabled.")
        # (中文解釋：程式結束。所有 PWM 輸出均已禁用。)


if __name__ == "__main__":
    main()

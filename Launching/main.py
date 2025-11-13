# -*- coding: utf-8 -*-
# 檔案名稱: main_controller.py (V1.2 - 隱藏 malformed data 錯誤)
# 功能: 接收 'remote_receiver.py' 傳來的 UART 訊號，
#       並根據 CH3 和 CH7 控制伺服馬達與無刷馬達。
#       (此版本會忽略不完整的訊號，只印出成功的訊號)

from machine import Pin, PWM, UART
import utime

# ==================== 硬體腳位設定 (Hardware Pinout) ====================
UART_ID = 0
UART_BAUDRATE = 115200
UART_RX_PIN = 17

SERVO_PAN_PIN = 6

ESC_PIN_1 = 15
ESC_PIN_2 = 14

# ==================== 伺服馬達設定 (Servo Config) ====================
PWM_FREQ_SERVO = 50
SERVO_MIN_PULSE_US = 500
SERVO_MAX_PULSE_US = 2500
PAN_CALIBRATION_0_DEG = 45
PAN_CALIBRATION_90_DEG = 90
PAN_CALIBRATION_180_DEG = 150

# ==================== 無刷馬達設定 (ESC Config) ====================
PWM_FREQ_ESC = 50
ESC_MIN_DUTY_US = 1000
ESC_MAX_DUTY_US = 2000
ESC_STOP_PULSE_US = 1000

# ==================== 硬體初始化 (Hardware Init) ====================
uart = UART(UART_ID, baudrate=UART_BAUDRATE, rx=Pin(UART_RX_PIN))
print(f"Main Controller: Listening on UART {UART_ID} (RX: GP{UART_RX_PIN})")

pwm_pan = PWM(Pin(SERVO_PAN_PIN))
pwm_pan.freq(PWM_FREQ_SERVO)

pwm_esc1 = PWM(Pin(ESC_PIN_1))
pwm_esc2 = PWM(Pin(ESC_PIN_2))
pwm_esc1.freq(PWM_FREQ_ESC)
pwm_esc2.freq(PWM_FREQ_ESC)


# ==================== 伺服馬達控制函式 (Servo Functions) ====================


def _servo_raw_to_duty(raw_val: float) -> int:
    pulse_width = SERVO_MIN_PULSE_US + (raw_val / 180) * (
        SERVO_MAX_PULSE_US - SERVO_MIN_PULSE_US
    )
    duty = int((pulse_width / (1_000_000 / PWM_FREQ_SERVO)) * 65535)
    return duty


def set_pan_angle(angle: float):
    angle = max(-90, min(90, angle))
    in_min, in_max = -90, 90
    out_min, out_max = PAN_CALIBRATION_180_DEG, PAN_CALIBRATION_0_DEG
    raw_value = (angle - in_min) * (out_max - out_min) / (in_max - in_min) + out_min
    duty_value = _servo_raw_to_duty(raw_value)
    pwm_pan.duty_u16(duty_value)


# ==================== 無刷馬達控制函式 (ESC Functions) ====================


def _esc_us_to_duty(us):
    period_us = (1.0 / PWM_FREQ_ESC) * 1_000_000
    duty_cycle = (us / period_us) * 65535
    return int(duty_cycle)


def set_esc_speed(pwm, percent: int):
    percent = max(0, min(100, percent))
    us = ESC_MIN_DUTY_US + (ESC_MAX_DUTY_US - ESC_MIN_DUTY_US) * percent / 100
    pwm.duty_u16(_esc_us_to_duty(us))


def stop_esc(pwm):
    pwm.duty_u16(_esc_us_to_duty(ESC_STOP_PULSE_US))


# ==================== 主程式 (Main Program) ====================
def run_controller():
    print("--- Starting Main Controller ---")

    try:
        # --- 1. 解鎖電調 (Arming ESCs) ---
        print("Arming ESCs... (Sending STOP signal)")
        stop_esc(pwm_esc1)
        stop_esc(pwm_esc2)
        utime.sleep(2)
        print("ESCs Armed. Ready.")

        # --- 2. 初始狀態 ---
        set_pan_angle(0)
        current_esc_speed = 50
        set_esc_speed(pwm_esc1, current_esc_speed)
        set_esc_speed(pwm_esc2, current_esc_speed)
        print("Servo centered. ESCs set to initial speed (70%).")

        # --- 3. 主迴圈 (Main Loop) ---
        while True:
            try:
                if uart.any():
                    line = uart.readline()
                    if line:
                        # 解碼並清理字串
                        command_string = line.decode("utf-8").strip()

                        # 檢查清理後是否為空字串 (有時會發生)
                        if not command_string:
                            continue

                        parts = command_string.split(",")

                        # ====================【修改點 1】====================
                        # 檢查是否為 10 個通道的正確訊號
                        if len(parts) == 10:

                            # ====================【修改點 2】====================
                            # (新增) 只在訊號正確時印出
                            # print(f"Received OK: {command_string}")

                            # --- A. 伺服馬達 (CH3) ---
                            try:
                                ch3_val = int(parts[3])
                                target_angle = (ch3_val / 100.0) * 90.0
                                set_pan_angle(target_angle)
                            except ValueError:
                                # (我們保留這個錯誤，因為這代表訊號 *內容* 有誤)
                                print(f"Invalid CH3 data: {parts[3]}")

                            # --- B. 無刷馬達 (CH7) ---
                            try:
                                ch7_val = int(parts[7])
                                new_speed = current_esc_speed

                                if ch7_val < -50:
                                    new_speed = 90
                                elif ch7_val > 50:
                                    new_speed = 50
                                elif -20 <= ch7_val <= 20:
                                    new_speed = 70

                                if new_speed != current_esc_speed:
                                    current_esc_speed = new_speed
                                    set_esc_speed(pwm_esc1, current_esc_speed)
                                    set_esc_speed(pwm_esc2, current_esc_speed)

                            except ValueError:
                                # (我們保留這個錯誤，因為這代表訊號 *內容* 有誤)
                                print(f"Invalid CH7 data: {parts[7]}")

                utime.sleep_ms(5)

            except Exception as e:
                print(f"An error occurred in main loop: {e}")
                utime.sleep(1)

    except KeyboardInterrupt:
        print("\nProgram stopped by user.")

    except Exception as e:
        print(f"A critical error occurred: {e}")

    finally:
        # --- 4. 清理 (Cleanup) ---
        print("Stopping motors and disabling PWM...")
        set_esc_speed(pwm_esc1, 0)
        set_esc_speed(pwm_esc2, 0)
        pwm_pan.deinit()
        pwm_esc1.deinit()
        pwm_esc2.deinit()
        print("Program terminated.")


# --- 程式入口 ---
if __name__ == "__main__":
    run_controller()

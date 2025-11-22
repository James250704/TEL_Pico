# -*- coding: utf-8 -*-
# 檔案名稱: main_controller.py (V1.9 - 保活與穩壓版)
# 修改重點:
# 1. [保活機制] 每 800ms 強制重發 PWM 給 ESC，防止電調因超時而降速或停機。
# 2. [電壓補償] 在推球 (Servo動作) 瞬間，自動微幅增加馬達油門，補償電池壓降。

from machine import Pin, PWM, UART
import utime

# ==================== 使用者參數設定 ====================
# --- 轉速設定 ---
MODE_1_SPEED_TOP = 80
MODE_1_SPEED_BOTTOM = 25
MODE_2_SPEED_TOP = 45
MODE_2_SPEED_BOTTOM = 45
MODE_3_SPEED_TOP = 25
MODE_3_SPEED_BOTTOM = 82

# --- 推球與補償設定 ---
AUX_PUSH_ANGLE = 90
AUX_RETRACT_ANGLE = 0
AUX_HOLD_MS = 500  # 推球停留時間 (建議 200-300ms 即可)
SHOT_COOLDOWN_MS = 800  # 射擊冷卻

# [關鍵設定] 電壓補償 (Voltage Compensation)
# 當 Servo 動作時，電池電壓會下降，導致輪子變慢。
# 這裡設定在推球時，額外增加多少 % 的油門來抵抗掉速。
# 如果射出去的球還是太低，請試著增加此數值 (例如 3 或 5)
PUSH_VOLTAGE_COMPENSATION = 2

# [關鍵設定] ESC 保活訊號間隔 (毫秒)
# 為了滿足 "每秒都要傳送訊號"，設定 800ms (小於 1000ms 以策安全)
ESC_KEEPALIVE_INTERVAL_MS = 800

# ==================== 硬體腳位設定 ====================
UART_ID = 0
UART_BAUDRATE = 115200
UART_RX_PIN = 17

SERVO_PAN_PIN = 6
SERVO_AUX_PIN = 12
ESC_PIN_TOP = 15
ESC_PIN_BOTTOM = 14

# ==================== 初始化 (PWM/UART) ====================
PWM_FREQ_SERVO = 50
SERVO_MIN_PULSE_US = 500
SERVO_MAX_PULSE_US = 2500
PAN_CALIBRATION_0_DEG = 55
PAN_CALIBRATION_180_DEG = 150

PWM_FREQ_ESC = 55
ESC_MIN_DUTY_US = 1000
ESC_MAX_DUTY_US = 2000
ESC_STOP_PULSE_US = 1000

# 預先計算 PWM 轉換係數
SERVO_CONVERSION_FACTOR = (PWM_FREQ_SERVO * 65535) / 1_000_000
ESC_CONVERSION_FACTOR = (PWM_FREQ_ESC * 65535) / 1_000_000

uart = UART(UART_ID, baudrate=UART_BAUDRATE, rx=Pin(UART_RX_PIN))
pwm_pan = PWM(Pin(SERVO_PAN_PIN))
pwm_pan.freq(PWM_FREQ_SERVO)
pwm_aux = PWM(Pin(SERVO_AUX_PIN))
pwm_aux.freq(PWM_FREQ_SERVO)
pwm_esc_top = PWM(Pin(ESC_PIN_TOP))
pwm_esc_bottom = PWM(Pin(ESC_PIN_BOTTOM))
pwm_esc_top.freq(PWM_FREQ_ESC)
pwm_esc_bottom.freq(PWM_FREQ_ESC)


# ==================== 輔助函式 ====================
def _servo_raw_to_duty(raw_val: float) -> int:
    pulse_width = SERVO_MIN_PULSE_US + (raw_val / 180) * (
        SERVO_MAX_PULSE_US - SERVO_MIN_PULSE_US
    )
    return int(pulse_width * SERVO_CONVERSION_FACTOR)


def set_pan_angle(angle: float):
    angle = max(-90, min(90, angle))
    in_min, in_max = -90, 90
    out_min, out_max = PAN_CALIBRATION_180_DEG, PAN_CALIBRATION_0_DEG
    raw_value = (angle - in_min) * (out_max - out_min) / (in_max - in_min) + out_min
    pwm_pan.duty_u16(_servo_raw_to_duty(raw_value))


def set_aux_angle(angle: float):
    pwm_aux.duty_u16(_servo_raw_to_duty(angle))


def _esc_us_to_duty(us):
    return int(us * ESC_CONVERSION_FACTOR)


def set_esc_raw(pwm, percent: int):
    """設定 ESC 油門 (0-100%)"""
    percent = max(0, min(100, percent))
    us = ESC_MIN_DUTY_US + (ESC_MAX_DUTY_US - ESC_MIN_DUTY_US) * percent / 100
    pwm.duty_u16(_esc_us_to_duty(us))


def stop_escs():
    stop_duty = _esc_us_to_duty(ESC_STOP_PULSE_US)
    pwm_esc_top.duty_u16(stop_duty)
    pwm_esc_bottom.duty_u16(stop_duty)


# ==================== 主程式 ====================
def run_controller():
    # print("--- Main Controller V1.9 (Keep-Alive Active) ---")

    # 基礎目標速度 (由 Switch 決定)
    base_top = MODE_3_SPEED_TOP
    base_bottom = MODE_3_SPEED_BOTTOM

    # 實際輸出速度 (包含補償)
    output_top = base_top
    output_bottom = base_bottom

    aux_state = "IDLE"
    aux_timer_start = 0
    last_shot_time = 0
    trigger_active_prev = False

    # 這裡是關鍵：紀錄上次發送訊號的時間
    last_esc_refresh_time = utime.ticks_ms()

    try:
        # 解鎖
        stop_escs()
        utime.sleep(2)

        # 初始設定
        set_pan_angle(0)
        set_aux_angle(AUX_RETRACT_ANGLE)
        set_esc_raw(pwm_esc_top, base_top)
        set_esc_raw(pwm_esc_bottom, base_bottom)

        while True:
            current_time = utime.ticks_ms()

            # ==========================================================
            # [關鍵功能 1] ESC 保活訊號 (Keep-Alive)
            # 無論是否有收到新指令，每 0.8 秒強制發送一次 PWM 值
            # 這能防止 ESC 因認為訊號丟失而降速
            # ==========================================================
            if (
                utime.ticks_diff(current_time, last_esc_refresh_time)
                > ESC_KEEPALIVE_INTERVAL_MS
            ):
                set_esc_raw(pwm_esc_top, output_top)
                set_esc_raw(pwm_esc_bottom, output_bottom)
                last_esc_refresh_time = current_time
                # print("Keep-alive signal sent") # Debug 用，確認有在發送

            # --- 射擊狀態機 ---
            if aux_state == "PUSHING":
                # 檢查是否推完
                if utime.ticks_diff(current_time, aux_timer_start) > AUX_HOLD_MS:
                    # 1. 收回推桿
                    set_aux_angle(AUX_RETRACT_ANGLE)
                    aux_state = "IDLE"

                    # 2. 移除電壓補償 (恢復正常轉速)
                    output_top = base_top
                    output_bottom = base_bottom
                    set_esc_raw(pwm_esc_top, output_top)
                    set_esc_raw(pwm_esc_bottom, output_bottom)
                    # print("Shot done. Speed normalized.")

            # --- UART 指令處理 ---
            if uart.any():
                line = uart.readline()
                if line:
                    try:
                        cmd = line.decode("utf-8").strip()
                        parts = cmd.split(",")
                        if len(parts) == 10:
                            # 1. Pan Servo
                            ch3_val = int(parts[3])
                            set_pan_angle((ch3_val / 100.0) * 30.0)

                            # 2. 轉速模式切換 (CH7)
                            ch7_val = int(parts[7])
                            target_top = MODE_2_SPEED_TOP
                            target_bot = MODE_2_SPEED_BOTTOM

                            if ch7_val < -50:
                                target_top, target_bot = (
                                    MODE_1_SPEED_TOP,
                                    MODE_1_SPEED_BOTTOM,
                                )
                            elif ch7_val > 50:
                                target_top, target_bot = (
                                    MODE_3_SPEED_TOP,
                                    MODE_3_SPEED_BOTTOM,
                                )

                            # 只有當目標基礎速度改變時，才更新
                            if target_top != base_top or target_bot != base_bottom:
                                base_top = target_top
                                base_bottom = target_bot
                                # 如果目前沒有在射擊(無補償狀態)，直接更新輸出
                                if aux_state == "IDLE":
                                    output_top = base_top
                                    output_bottom = base_bottom
                                    set_esc_raw(pwm_esc_top, output_top)
                                    set_esc_raw(pwm_esc_bottom, output_bottom)
                                    last_esc_refresh_time = current_time  # 重置計時器

                            # 3. 射擊觸發 (CH8)
                            ch8_val = int(parts[8])
                            trigger_active = ch8_val > 50

                            if (
                                trigger_active
                                and not trigger_active_prev
                                and aux_state == "IDLE"
                                and utime.ticks_diff(current_time, last_shot_time)
                                > SHOT_COOLDOWN_MS
                            ):

                                # A. 加入電壓補償 (加速抵抗壓降)
                                output_top = min(
                                    100, base_top + PUSH_VOLTAGE_COMPENSATION
                                )
                                output_bottom = min(
                                    100, base_bottom + PUSH_VOLTAGE_COMPENSATION
                                )
                                set_esc_raw(pwm_esc_top, output_top)
                                set_esc_raw(pwm_esc_bottom, output_bottom)

                                # B. 執行射擊
                                set_aux_angle(AUX_PUSH_ANGLE)
                                aux_state = "PUSHING"
                                aux_timer_start = current_time
                                last_shot_time = current_time
                                # print(f"FIRE! Comp: +{PUSH_VOLTAGE_COMPENSATION}%")

                            trigger_active_prev = trigger_active

                    except ValueError:
                        pass

            # 短暫休息，避免 CPU 100% 佔用影響計時
            utime.sleep_ms(1)

    except KeyboardInterrupt:
        print("Stopped.")
    finally:
        stop_escs()
        pwm_pan.deinit()
        pwm_esc_top.deinit()
        pwm_esc_bottom.deinit()
        pwm_aux.deinit()


if __name__ == "__main__":
    run_controller()

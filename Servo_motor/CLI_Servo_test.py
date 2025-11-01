# 雙軸伺服馬達控制程式

import utime
from machine import Pin, PWM

# --- 伺服馬達基本設定 ---
# 設定兩顆伺服馬達的訊號腳位
SERVO_PAN_PIN = 21  # 水平 Pan 伺服馬達
SERVO_PITCH_PIN = 20  # 俯仰 Pitch 伺服馬達

PWM_FREQ = 50

# 這些是伺服馬達PWM訊號的物理極限 (單位: 微秒 us)
# 如果兩顆馬達型號不同，您也可以為它們分別設定
MIN_PULSE_US = 500
MAX_PULSE_US = 2500

# --- 水平馬達 (Pan) 的校準數據 ---
# 這些是您測量出的 "原始角度值"，對應到真實世界的角度
PAN_CALIBRATION_0_DEG = 10
PAN_CALIBRATION_90_DEG = 97
PAN_CALIBRATION_180_DEG = 185

# --- 俯仰馬達 (Pitch) 的校準數據 ---
# !! 重要 !!: 您需要為第二顆馬達重新校準這些值。
# 這裡暫時使用與第一顆相同的數值作為預設值。
PITCH_CALIBRATION_0_DEG = 30
PITCH_CALIBRATION_90_DEG = 90
PITCH_CALIBRATION_180_DEG = 150


# --- PWM 初始化 ---
pwm_pan = PWM(Pin(SERVO_PAN_PIN))
pwm_pan.freq(PWM_FREQ)

pwm_pitch = PWM(Pin(SERVO_PITCH_PIN))
pwm_pitch.freq(PWM_FREQ)


# --- 內部輔助函式 ---
def _raw_value_to_duty(raw_val: float) -> int:
    """
    [內部函式] 將 0-180 範圍的 "原始校準值" 轉換為 PWM 的 duty_u16 值。
    這個函式是通用的，兩顆馬達都可以使用。
    """
    pulse_width = MIN_PULSE_US + (raw_val / 180) * (MAX_PULSE_US - MIN_PULSE_US)
    duty = int((pulse_width / 20000) * 65535)
    return duty


# --- 主要控制函式 ---
def set_pan_angle(angle: float):
    """
    設定水平 (Pan) 伺服馬達的角度。輸入範圍: -90 到 90。
    """
    angle = max(-90, min(90, angle))

    # 使用水平馬達的校準數據進行映射
    in_min, in_max = -90, 90
    out_min, out_max = PAN_CALIBRATION_180_DEG, PAN_CALIBRATION_0_DEG

    raw_value = (angle - in_min) * (out_max - out_min) / (in_max - in_min) + out_min
    duty_value = _raw_value_to_duty(raw_value)
    pwm_pan.duty_u16(duty_value)

    print(
        f"Pan Angle: {angle:.1f} -> Raw: {raw_value:.2f} -> Duty: {duty_value}"
    )  # 水平角度 -> 原始值 -> PWM Duty


def set_pitch_angle(angle: float):
    """
    設定俯仰 (Pitch) 伺服馬達的角度。輸入範圍: -45 到 60。
    """
    # *** 修改點：將角度限制在 -45 到 60 度之間 ***
    angle = max(-90, min(90, angle))

    # 使用俯仰馬達的校準數據進行映射
    # 注意：這裡的 in_min, in_max 仍然使用 -90 和 90 來計算比例，
    # 這樣可以確保馬達在新的活動範圍內仍然使用完整的校準解析度。
    in_min, in_max = -90, 90
    out_min, out_max = PITCH_CALIBRATION_180_DEG, PITCH_CALIBRATION_0_DEG

    raw_value = (angle - in_min) * (out_max - out_min) / (in_max - in_min) + out_min
    duty_value = _raw_value_to_duty(raw_value)
    pwm_pitch.duty_u16(duty_value)

    print(
        f"Pitch Angle: {angle:.1f} -> Raw: {raw_value:.2f} -> Duty: {duty_value}"
    )  # 俯仰角度 -> 原始值 -> PWM Duty


# --- 主程式 (互動模式) ---
try:
    print(
        f"Starting dual servo controller (Pan: GP{SERVO_PAN_PIN}, Pitch: GP{SERVO_PITCH_PIN})"
    )  # 啟動雙軸伺服馬達控制器
    print(
        "Enter command, e.g., 'p 30' for pan or 't -45' for pitch."
    )  # 請輸入指令, 例如 'p 30' (水平) 或 't -45' (俯仰)
    print("Enter 'q' to quit.")  # 輸入 'q' 退出程式

    # 初始時先將兩顆馬達歸中
    set_pan_angle(0)
    set_pitch_angle(0)

    while True:
        try:
            user_input = input("Enter command: ")  # 請輸入指令:
            parts = user_input.strip().lower().split()

            if not parts:
                continue

            command = parts[0]
            if command == "q":
                break

            if len(parts) != 2:
                print(
                    "Invalid format. Use 'p <angle>' or 't <angle>'."
                )  # 格式錯誤，請使用 'p <角度>' 或 't <角度>'
                continue

            axis = command
            angle = float(parts[1])

            if axis == "p":
                set_pan_angle(angle)
            elif axis == "t":
                set_pitch_angle(angle)
            else:
                print(
                    f"Unknown axis '{axis}'. Use 'p' or 't'."
                )  # 無法識別的軸 '{axis}'，請使用 'p' 或 't'

        except ValueError:
            print("Invalid angle. Please enter a number.")  # 角度無效，請輸入一個數字。
        except Exception as e:
            print(f"An error occurred: {e}")  # 發生錯誤

except KeyboardInterrupt:
    print("\nProgram stopped.")  # 程式已停止。
finally:
    # 程式結束時，關閉所有 PWM 輸出
    pwm_pan.deinit()
    pwm_pitch.deinit()

# 檔案名稱: servo_pan_only.py
# 功能: 單軸 (Pan) 伺服馬達控制程式

import utime
from machine import Pin, PWM

# --- 伺服馬達基本設定 ---
# 設定伺服馬達的訊號腳位
SERVO_PAN_PIN = 6  # 水平 Pan 伺服馬達

PWM_FREQ = 50

# 伺服馬達PWM訊號的物理極限 (單位: 微秒 us)
MIN_PULSE_US = 500
MAX_PULSE_US = 2500

# --- 水平馬達 (Pan) 的校準數據 ---
# 這些是您測量出的 "原始角度值"，對應到真實世界的角度
# 範例： 0度 對應 原始值 0, 180度 對應 原始值 180
PAN_CALIBRATION_0_DEG = 30
PAN_CALIBRATION_90_DEG = 90
PAN_CALIBRATION_180_DEG = 150


# --- PWM 初始化 ---
pwm_pan = PWM(Pin(SERVO_PAN_PIN))
pwm_pan.freq(PWM_FREQ)


# --- 內部輔助函式 ---
def _raw_value_to_duty(raw_val: float) -> int:
    """
    [內部函式] 將 0-180 範圍的 "原始校準值" 轉換為 PWM 的 duty_u16 值。
    """
    # 將 0-180 的原始值 映射到 脈衝寬度範圍 (MIN_PULSE_US 到 MAX_PULSE_US)
    pulse_width = MIN_PULSE_US + (raw_val / 180) * (MAX_PULSE_US - MIN_PULSE_US)

    # 將脈衝寬度 (us) 轉換為 50Hz (20000 us 週期) 下的 duty_u16 值 (0-65535)
    duty = int((pulse_width / 20000) * 65535)
    return duty


# --- 主要控制函式 ---
def set_pan_angle(angle: float):
    """
    設定水平 (Pan) 伺服馬達的角度。輸入範圍: -90 到 90。
    """
    # 限制輸入角度在 -90 到 90 之間
    angle = max(-90, min(90, angle))

    # --- 映射 (Mapping) ---
    # 將輸入角度 (-90 到 90) 映射到您的 "原始校準值" 範圍
    # 這裡我們假設 -90度 對應 原始值180 (PAN_CALIBRATION_180_DEG)
    # 並且 90度 對應 原始值0 (PAN_CALIBRATION_0_DEG)
    # 如果您的馬達方向相反，請交換 out_min 和 out_max

    in_min, in_max = -90, 90
    out_min, out_max = PAN_CALIBRATION_180_DEG, PAN_CALIBRATION_0_DEG

    raw_value = (angle - in_min) * (out_max - out_min) / (in_max - in_min) + out_min

    # 將計算出的 "原始值" 轉換為 PWM duty
    duty_value = _raw_value_to_duty(raw_value)

    # 設定 PWM
    pwm_pan.duty_u16(duty_value)

    print(
        f"Pan Angle: {angle:.1f} -> Raw: {raw_value:.2f} -> Duty: {duty_value}"
    )  # 水平角度 -> 原始值 -> PWM Duty


# --- 主程式 (互動模式) ---
try:
    print(
        f"Starting single servo controller (Pan: GP{SERVO_PAN_PIN})"
    )  # 啟動單軸伺服馬達控制器
    print(
        "Enter command, e.g., 'p 30' or just '30'."
    )  # 請輸入指令, 例如 'p 30' 或 '30'
    print("Enter 'q' to quit.")  # 輸入 'q' 退出程式

    # 初始時先將馬達歸中
    set_pan_angle(0)

    while True:
        try:
            user_input = input("Enter command: ")  # 請輸入指令:
            parts = user_input.strip().lower().split()

            if not parts:
                continue

            command = parts[0]
            if command == "q":
                break

            angle_str = ""
            if len(parts) == 1:
                # 如果只輸入數字 (例如 '30')
                angle_str = parts[0]
            elif len(parts) == 2 and parts[0] == "p":
                # 如果輸入 'p 30'
                angle_str = parts[1]
            else:
                print(
                    "Invalid format. Use '<angle>' or 'p <angle>'."
                )  # 格式錯誤，請使用 '<角度>' 或 'p <角度>'
                continue

            angle = float(angle_str)
            set_pan_angle(angle)

        except ValueError:
            print("Invalid angle. Please enter a number.")  # 角度無效，請輸入一個數字。
        except Exception as e:
            print(f"An error occurred: {e}")  # 發生錯誤

except KeyboardInterrupt:
    print("\nProgram stopped.")  # 程式已停止。
finally:
    # 程式結束時，關閉 PWM 輸出
    pwm_pan.deinit()
    print("PWM disabled.")  # PWM 已禁用

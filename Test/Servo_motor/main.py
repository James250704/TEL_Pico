# 檔案名稱: servo_manual_two_state.py
# 功能: 單軸 (Pan) 伺服馬達控制程式 - 僅 0度 與 90度 兩段式手動控制

import utime
from machine import Pin, PWM

# --- 伺服馬達基本設定 ---
SERVO_PAN_PIN = 12  # 水平 Pan 伺服馬達
PWM_FREQ = 50

# 伺服馬達 PWM 訊號的物理極限 (單位: 微秒 us)
MIN_PULSE_US = 500
MAX_PULSE_US = 2500

# --- 0度 與 90度 的校準值 (原始值) ---
# 0度角度 對應的原始值為 90
PAN_RAW_VALUE_0_DEG = 0
# 90度角度 對應的原始值為 0
PAN_RAW_VALUE_90_DEG = 80


# --- PWM 初始化 ---
pwm_pan = PWM(Pin(SERVO_PAN_PIN))
pwm_pan.freq(PWM_FREQ)


# --- 內部輔助函式 ---
def _raw_value_to_duty(raw_val: float) -> int:
    """
    [內部函式] 將 0-180 範圍的 "原始校準值" 轉換為 PWM 的 duty_u16 值。
    """
    # 將原始值 (0 到 180) 映射到 脈衝寬度範圍 (MIN_PULSE_US 到 MAX_PULSE_US)
    pulse_width = MIN_PULSE_US + (raw_val / 180) * (MAX_PULSE_US - MIN_PULSE_US)

    # 將脈衝寬度 (us) 轉換為 50Hz (20000 us 週期) 下的 duty_u16 值 (0-65535)
    duty = int((pulse_width / 20000) * 65535)
    return duty


# --- 主要控制函式 ---
def set_pan_position(target_angle: int):
    """
    設定水平 (Pan) 伺服馬達的位置。只能是 0 或 90。
    """
    # 僅接受 0 或 90 的輸入
    if target_angle == 0:
        raw_value = PAN_RAW_VALUE_0_DEG
    elif target_angle == 90:
        raw_value = PAN_RAW_VALUE_90_DEG
    else:
        print(f"Angle {target_angle} is not allowed. Please enter 0 or 90.")
        return

    # 將計算出的 "原始值" 轉換為 PWM duty
    duty_value = _raw_value_to_duty(raw_value)

    # 設定 PWM
    pwm_pan.duty_u16(duty_value)

    print(f"✅ Angle Set: {target_angle} -> Duty: {duty_value}")


# --- 主程式 (手動互動模式) ---
try:
    print("--- 啟動雙狀態伺服馬達控制器 (GP16) ---")

    # [預設角度設定]：初始時先將馬達歸位到 0 度
    set_pan_position(0)
    utime.sleep(0.5)

    print("\n請輸入指令 '0' 或 '90' 來控制馬達位置。")
    print("輸入 'q' 退出程式。")

    while True:
        try:
            user_input = input("Enter 0 or 90: ")  # 請輸入指令:

            if user_input.lower().strip() == "q":
                break

            # 嘗試將輸入轉換為整數
            angle = int(user_input.strip())

            # 檢查輸入是否為允許的值
            # if angle not in [0, 90]:
            #     print("❌ 角度無效。請輸入 0 或 90。")
            #     continue

            set_pan_position(angle)

        except ValueError:
            print("❌ 角度無效。請輸入數字 0, 90, 或 'q'。")
        except Exception as e:
            print(f"發生錯誤: {e}")

except KeyboardInterrupt:
    print("\n程式已停止。")
finally:
    # 程式結束時，關閉 PWM 輸出
    pwm_pan.deinit()
    print("PWM 已禁用。")

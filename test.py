from machine import Pin
import time

# --- 參數設定 ---
# 速度控制 (單位：微秒)，數值越小，轉速越快。
# 建議從 1000 開始，若馬達抖動或發出噪音，請嘗試增加此數值。
STEP_DELAY_US = 1250

# --- 腳位定義 ---
# 只定義需要用到的 X 和 Y 軸馬達
motors = [
    {"step": Pin(0, Pin.OUT), "dir": Pin(1, Pin.OUT)},  # X 軸
    {"step": Pin(3, Pin.OUT), "dir": Pin(4, Pin.OUT)},  # Y 軸
]
en_pin = Pin(2, Pin.OUT)

# x -> 1, 2 y -> 3, 4 en -> 2

# --- 主程式 ---
print("Initializing motors...")

# 設定所有馬達方向 (1: high, 0: low)
# 您可以分開設定，例如 m[0]["dir"].value(1); m[1]["dir"].value(0)
for m in motors:
    m["dir"].value(0)

# 啟用馬達驅動器 (低電位致能)
en_pin.low()
print("Motors enabled. Press Ctrl+C to stop.")

try:
    # 無限迴圈，讓馬達持續轉動
    while True:
        # --- 發送一個步進脈衝 ---
        # 1. 將所有馬達的 STEP 拉高
        for m in motors:
            m["step"].value(1)
        # 脈衝寬度，5微秒即可
        time.sleep_us(5)
        # 2. 再將所有馬達的 STEP 拉低，完成一步
        for m in motors:
            m["step"].value(0)

        # --- 步進之間的延遲，用來控制速度 ---
        time.sleep_us(STEP_DELAY_US)

except KeyboardInterrupt:
    # 當使用者按下 Ctrl+C 時，會執行這裡
    print("\nStopping motors...")

finally:
    # 程式結束前，務必執行這一段來禁用馬達
    en_pin.high()
    print("Motors disabled. Program ended.")

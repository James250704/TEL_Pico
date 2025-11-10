# 僅保留水平伺服馬達控制程式 (整合 UART 遙控)

import utime
from machine import Pin, PWM, UART

# --- 伺服馬達基本設定 ---
SERVO_PAN_PIN = 21  # 水平 Pan 伺服馬達

PWM_FREQ = 50

# 這些是伺服馬達PWM訊號的物理極限 (單位: 微秒 us)
MIN_PULSE_US = 500
MAX_PULSE_US = 2500

# --- 水平馬達 (Pan) 的校準數據 ---
PAN_CALIBRATION_0_DEG = 20
PAN_CALIBRATION_90_DEG = 90
PAN_CALIBRATION_180_DEG = 190

# ==================== UART 接收設定 ====================
UART_ID = 0
UART_BAUDRATE = 115200
UART_RX_PIN = 17  # 用於接收訊號 (對應發訊號程式的 TX 8)
UART_TX_PIN = 16  # (對應發訊號程式的 RX 9)

# 初始化 UART
uart = UART(UART_ID, baudrate=UART_BAUDRATE, tx=Pin(UART_TX_PIN), rx=Pin(UART_RX_PIN))
# 用於儲存 UART 傳入的零碎數據
uart_buffer = bytearray()
# ==========================================================

# --- PWM 初始化 ---
pwm_pan = PWM(Pin(SERVO_PAN_PIN))
pwm_pan.freq(PWM_FREQ)


# --- 內部輔助函式 ---
def _raw_value_to_duty(raw_val: float) -> int:
    """
    [內部函式] 將 0-180 範圍的 "原始校準值" 轉換為 PWM 的 duty_u16 值。
    """
    pulse_width = MIN_PULSE_US + (raw_val / 180) * (MAX_PULSE_US - MIN_PULSE_US)
    duty = int((pulse_width / 20000) * 65535)
    return duty


# ==================== 數值映射函式 ====================
def map_value(value, in_min, in_max, out_min, out_max):
    """
    將一個值從一個範圍線性映射到另一個範圍。
    """
    # 確保 value 在 in_min 和 in_max 之間
    value = max(in_min, min(in_max, value))
    return (value - in_min) * (out_max - out_min) / (in_max - in_min) + out_min


# =========================================================


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

    # 為了避免洗版，您可以考慮註解掉這個 print
    # print(f"Pan Angle: {angle:.1f} -> Raw: {raw_value:.2f} -> Duty: {duty_value}")


# ==================== 主程式 (UART 迴圈) ====================

try:
    print(f"Starting horizontal servo controller (Pan: GP{SERVO_PAN_PIN})")
    print(
        f"Listening for commands on UART {UART_ID} (RX: GP{UART_RX_PIN}) at {UART_BAUDRATE} baud."
    )

    # 初始時先將馬達歸中
    set_pan_angle(0)
    led = Pin("LED", Pin.OUT)

    while True:
        # 1. 檢查 UART 是否有數據
        led.toggle()
        if uart.any():
            # 讀取所有可用的數據並附加到緩衝區
            data = uart.read(uart.any())
            if data:
                uart_buffer.extend(data)

                # 2. 檢查緩衝區中是否包含完整的行 (以 '\n' 結尾)
                newline_index = uart_buffer.find(b"\n")

                # 迴圈處理所有緩衝區中的完整行
                while newline_index != -1:
                    # 提取一行數據
                    line_bytes = uart_buffer[:newline_index]

                    # 從緩衝區移除已處理的行 (包含 '\n')
                    uart_buffer = uart_buffer[newline_index + 1 :]

                    try:
                        # 3. 解碼並解析
                        line_str = line_bytes.decode("utf-8").strip()

                        if not line_str:  # 忽略空行
                            continue

                        # 預期格式: "ch0,ch1,ch2,ch3,..."
                        parts = line_str.split(",")

                        # 根據 DBR4 規格，CH3 (索引 3) 控制雲台
                        # 確保至少有 4 個通道 (索引 0, 1, 2, 3)
                        if len(parts) >= 4:
                            # CH3 (index 3) -> 水平 (Pan)
                            ch3_val = int(parts[3])  # 水平

                            # 4. 映射數值到角度
                            # ch3 (Pan): -100 to 100 -> -90 to 90
                            pan_angle = map_value(ch3_val, -100, 100, -90, 90)

                            # 5. 控制伺服馬達
                            set_pan_angle(pan_angle)

                        else:
                            print(f"Received incomplete data: {line_str}")

                    except (ValueError, IndexError, UnicodeError) as e:
                        print(f"Error parsing data: {e}, Data: {line_bytes}")

                    # 檢查緩衝區是否還有下一行
                    newline_index = uart_buffer.find(b"\n")

        # 稍微延遲，避免 CPU 100% 空轉
        utime.sleep_ms(1)

except KeyboardInterrupt:
    print("\nProgram stopped.")
finally:
    # 程式結束時，關閉所有 PWM 輸出
    print("Deinitializing PWM and UART...")
    pwm_pan.deinit()
    uart.deinit()
    print("Cleanup complete. Exiting.")

# =================================================================

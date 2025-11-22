# -*- coding: utf-8 -*-
# 檔案名稱: main_controller.py
# 功能: 棒球機器人主控程式
#      1. 透過 UART 接收 "remote_receiver.py" 傳來的指令
#      2. 控制射擊雲台 (CH2, CH3) - (此版本先實作 CH2 油門)

from machine import Pin, PWM, UART
import time

# ==================== 馬達設定 (Motor Config) ====================
PWM_FREQ = 50  # ESC expects 50 Hz

# ESC signal pins
PWM_PIN1 = 15  # Motor 1 -> ESC1 signal
PWM_PIN2 = 14  # Motor 2 -> ESC2 signal

# Throttle range
MIN_DUTY_US = 1000  # stop
MAX_DUTY_US = 2000  # full throttle
STOP_PULSE_US = 1000

# ====================【修改點 1: 新增 UART 設定】====================
class CommConfig:
    UART_ID = 0
    BAUDRATE = 115200
    # (中文：RX 腳位 8 必須連接到 "接收Pico" 的 TX 腳位 8)
    # (RX Pin 8 must connect to "Receiver Pico's" TX Pin 8)
    RX_PIN_FROM_RECEIVER = 17
    # (中文：TX 腳位 9 必須連接到 "接收Pico" 的 RX 腳位 9)
    # (TX Pin 9 must connect to "Receiver Pico's" RX Pin 9)
    TX_PIN_TO_RECEIVER = 16


# =================================================================

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


def stop_motor(pwm, motor_id):
    print(f"--- Stopping/Arming Motor {motor_id} ---")
    pwm.duty_u16(us_to_duty(STOP_PULSE_US))


def main():
    # (中文：初始化與 "接收Pico" 通訊的 UART)
    comm_uart = UART(
        CommConfig.UART_ID,
        baudrate=CommConfig.BAUDRATE,
        tx=Pin(CommConfig.TX_PIN_TO_RECEIVER),
        rx=Pin(CommConfig.RX_PIN_FROM_RECEIVER),
    )
    # (中文：UART 讀取緩衝區)
    read_buffer = ""
    LED = Pin("LED", Pin.OUT)

    try:
        # (中文：發送停止訊號來解鎖電調)
        print("Arming ESCs... (Sending STOP signal)")
        time.sleep(2)
        stop_motor(pwm1, 1)
        stop_motor(pwm2, 2)
        time.sleep(2)  # (中文：給電調 2 秒鐘時間啟動)

        print("ESCs Armed. Ready to go.")
        # (中文：電調已解鎖，準備就緒。)
        print(
            f"Waiting for commands on UART {CommConfig.UART_ID} (RX:{CommConfig.RX_PIN_FROM_RECEIVER})..."
        )
        # (中文：正在 UART 1 (RX:8) 上等待指令...)

        # ====================【修改點 2: 替換主迴圈】====================
        # (中文：移除 input() 迴圈，改為 UART 讀取迴圈)
        while True:
            LED.toggle()
            # (中文：檢查 UART 是否有數據)
            if comm_uart.any():
                # (中文：讀取所有可用的字節)
                data = comm_uart.read()
                if data:
                    # (中文：將字節解碼為 utf-8 字串並添加到緩衝區)
                    read_buffer += data.decode("utf-8")

                    # (中文：檢查緩衝區中是否有一個完整的指令 (以 \n 結尾))
                    if "\n" in read_buffer:
                        # (中文：分離出第一條完整指令)
                        lines = read_buffer.split("\n")
                        command_string = lines.pop(0)

                        # (中文：將剩餘不完整的指令放回緩衝區)
                        read_buffer = "\n".join(lines)

                        # (中文：處理收到的指令)
                        try:
                            channels = command_string.split(",")

                            # (中文：我們需要 CH2 (索引 2) 來控制油門)
                            if len(channels) >= 3:
                                # (中文：將 CH2 的值 (0-100) 轉為整數)
                                ch2_speed = int(channels[2])

                                # (中文：設定兩顆馬達的轉速)
                                set_speed(pwm1, ch2_speed)
                                set_speed(pwm2, ch2_speed)

                                # (中文：打印除錯訊息)
                                # print(f"Motors set to CH2 value: {ch2_speed}%")
                            else:
                                print(f"Short command received: {command_string}")
                                # (中文：收到了過短的指令)

                        except (ValueError, IndexError):
                            print(f"Invalid data received: {command_string}")
                            # (中文：收到了無效的數據)

            # (中文：小延遲避免 CPU 佔用過高)
            time.sleep_ms(0)
        # =============================================================

    except KeyboardInterrupt:
        print("\nProgram interrupted by user.")
        # (中文：程式已被使用者中斷。)
    finally:
        stop_motor(pwm1, 1)
        stop_motor(pwm2, 2)
        pwm1.deinit()
        pwm2.deinit()
        comm_uart.deinit()  # (中文：關閉 UART)
        print("Program ended. Both PWM outputs and UART disabled.")
        # (中文：程式結束。PWM 輸出與 UART 均已禁用。)


if __name__ == "__main__":
    main()

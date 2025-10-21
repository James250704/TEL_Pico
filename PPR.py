# -*- coding: utf-8 -*-
from machine import Pin
import utime

# ==================== 設定 (Configuration) ====================
#
# 請修改這個腳位！
# 根據你提供的程式，你的編碼器腳位分別是 12, 13, 14, 15。
# 請選擇其中一個你想要測量的馬達所對應的編碼器腳位。
#
ENCODER_PIN_A = 12  # 例如：測量接在 Pin 12 上的左前輪 (LF)

# ==================== 全域變數 (Global Variable) ====================
# 用於儲存中斷計數的變數
# It's declared volatile to hint the compiler that it can change unexpectedly.
# In MicroPython, simply using a global is sufficient for this purpose.
volatile_pulse_count = 0

# ==================== 中斷處理函式 (Interrupt Service Routine) ====================
def encoder_irq_handler(pin):
    """
    這是一個特別的函式，每次編碼器腳位偵測到電位變化 (從低到高) 時，
    硬體就會自動呼叫它。我們在這裡只做一件事：將計數器加一。
    """
    global volatile_pulse_count
    volatile_pulse_count += 1

# ==================== 主程式 (Main Program) ====================
def measure_ppr():
    """
    主函式，用於初始化硬體並執行PPR測量。
    """
    global volatile_pulse_count
    volatile_pulse_count = 0 # 確保每次執行前都歸零

    # 1. 初始化編碼器腳位
    #    - Pin.IN: 設定為輸入模式
    #    - Pin.PULL_UP: 啟用內部上拉電阻，讓腳位在未觸發時保持高電位，訊號更穩定
    try:
        encoder_pin = Pin(ENCODER_PIN_A, Pin.IN, Pin.PULL_UP)
    except Exception as e:
        print(f"Error: Failed to initialize Pin {ENCODER_PIN_A}. Please check if the pin number is correct.")
        print(e)
        return

    # 2. 設定中斷觸發
    #    - trigger=Pin.IRQ_RISING: 設定當中斷腳位的電位從「低」變「高」(上升緣) 時觸發。
    #    - handler=encoder_irq_handler: 指定觸發後要執行的函式。
    encoder_pin.irq(trigger=Pin.IRQ_RISING, handler=encoder_irq_handler)

    # 3. 顯示說明，並開始測量迴圈
    print("=======================================")
    print(" Motor Encoder PPR Measurement Tool")
    print("=======================================")
    print(f"Listening on Pin {ENCODER_PIN_A}...")
    print("\n>>> When ready, rotate the wheel for ONE full, precise revolution.")
    print(">>> After one revolution, take note of the final number.")
    print(">>> Press Ctrl+C to stop the program.\n")

    last_printed_count = -1
    try:
        while True:
            # 為了避免螢幕被洗版，只有在計數有變化時才更新顯示
            if volatile_pulse_count != last_printed_count:
                # 使用 '\r' (carriage return) 讓數字在同一行更新，看起來更清爽
                print(f"Current PPR: {volatile_pulse_count}   ", end='\r')
                last_printed_count = volatile_pulse_count
            
            utime.sleep_ms(50) # 短暫延遲，降低 CPU 使用率

    except KeyboardInterrupt:
        # 當使用者在終端機按下 Ctrl+C 時，會觸發這個區塊
        print("\n\nProgram stopped.")
        print("---------------------------------------")
        print(f"Final measured PPR value: {volatile_pulse_count}")
        print("---------------------------------------")
        print("Please update the 'PULSES_PER_REVOLUTION' variable in your main script with this value.")
    finally:
        # 清理資源：在程式結束時，務必關閉中斷，避免產生不預期的行為
        encoder_pin.irq(handler=None)
        print("Interrupt handler disabled.")

# 程式的進入點
if __name__ == "__main__":
    measure_ppr()
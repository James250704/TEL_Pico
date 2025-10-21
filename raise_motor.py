from machine import Pin
import time

# 馬達對應的 STEP / DIR 腳位
motors = [
    {"step": Pin(0, Pin.OUT), "dir": Pin(1, Pin.OUT)},  # X
    {"step": Pin(3, Pin.OUT), "dir": Pin(4, Pin.OUT)},  # Y
    {"step": Pin(5, Pin.OUT), "dir": Pin(6, Pin.OUT)},  # Z
    {"step": Pin(7, Pin.OUT), "dir": Pin(8, Pin.OUT)},  # A
]

# EN 低電位啟用
EN = Pin(2, Pin.OUT)
EN.value(0)

# 設定全部方向為順時針 (1)
for m in motors:
    m["dir"].value(1)

# 讓全部馬達同時旋轉
while True:
    for m in motors:
        m["step"].value(1)
    time.sleep_us(1000)  # 速度控制
    for m in motors:
        m["step"].value(0)
    time.sleep_us(1000)

from machine import UART
import time

# ===== UART 初始化 =====
uart = UART(0, baudrate=420000, tx=12, rx=13, bits=8, parity=None, stop=1)

# ===== CRSF 常數 =====
CRSF_SYNC = 0xC8
CRSF_TYPE_CHANNELS = 0x16
CHANNEL_NUM = 16
CENTER = 992
RANGE = 820

# 最新通道值
latest_channels = [CENTER] * CHANNEL_NUM

# 暫存緩衝區
buf = bytearray(128)


def parse_channels(data):
    """解析 CRSF channel 資料 (22 bytes)"""
    if len(data) < 22:
        return None
    values = []
    bits = 0
    bitcount = 0
    for b in data:
        bits |= b << bitcount
        bitcount += 8
        while bitcount >= 11:
            values.append(bits & 0x7FF)
            bits >>= 11
            bitcount -= 11
    return values[:CHANNEL_NUM]


def poll_uart():
    """從 UART buffer 擷取並解析 CRSF 封包"""
    n = uart.readinto(buf)
    if not n:
        return None
    data = memoryview(buf)[:n]
    for i in range(len(data)):
        if data[i] == CRSF_SYNC and i + 2 < len(data):
            length = data[i + 1]
            if i + 2 + length <= len(data):
                payload = data[i + 2 : i + 2 + length]
                if payload[0] == CRSF_TYPE_CHANNELS:
                    return parse_channels(payload[1:-1])
    return None


def normalize(val):
    """將 CRSF 原始值轉成 -100 ~ +100"""
    return int((val - CENTER) * 100 / RANGE)


# ===== 主迴圈 =====
last = time.ticks_ms()
while True:
    ch = poll_uart()
    if ch:
        latest_channels = ch

    # 每 20 ms 更新一次輸出
    if time.ticks_diff(time.ticks_ms(), last) > 20:

        # rx = normalize(latest_channels[0])
        # ry = normalize(latest_channels[1])
        # lx = normalize(latest_channels[2])
        # ly = normalize(latest_channels[3])
        # ch5 SA ch6 SB ch7 SC ch8 SD ch9 SI
        # print(f"Right: ({rx},{ry}), Left: ({lx},{ly})")
        print("CH:", [normalize(c) for c in latest_channels[:10]])

        last = time.ticks_ms()

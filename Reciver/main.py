# -*- coding: utf-8 -*-
# 檔案名稱: remote_receiver.py
# 功能: 專門接收 DBR4 遙控器訊號，並將指令透過 UART 傳送給主控 Pico
# 版本: 1.7 (將跳變過濾器套用到所有 10 個通道，並修正過濾器邏輯)

from machine import Pin, UART
import utime

# ==================== 常數與設定 (Constants & Configuration) ====================


class CrsfConfig:
    UART_ID = 0
    BAUDRATE = 420000
    TX_PIN_DBR4 = 16
    RX_PIN_DBR4 = 17
    SYNC_BYTE = 0xC8
    TYPE_CHANNELS = 0x16
    CHANNEL_NUM = 16
    RC_CENTER = 992
    RC_RANGE = 820
    RC_DEADBAND = 20

    # ====================【修改點 1】====================
    # 降低閾值以捕捉更小的雜訊跳變
    # 最小的雜訊 (CH5: -97 -> -72) 原始值跳變約 229
    # 200 < 229，可以過濾掉所有日誌中的雜訊
    MAX_RAW_CHANGE_PER_STEP = 200
    # ====================================================


class CommConfig:
    UART_ID = 1
    BAUDRATE = 115200
    TX_PIN_TO_MAIN = 8
    RX_PIN_FROM_MAIN = 9


# ==================== 遙控器訊號處理 ====================
class RemoteControlTransmitter:
    def __init__(self):
        self.dbr4_uart = UART(
            CrsfConfig.UART_ID,
            baudrate=CrsfConfig.BAUDRATE,
            tx=Pin(CrsfConfig.TX_PIN_DBR4),
            rx=Pin(CrsfConfig.RX_PIN_DBR4),
            bits=8,
            parity=None,
            stop=1,
        )
        self.main_uart = UART(
            CommConfig.UART_ID,
            baudrate=CommConfig.BAUDRATE,
            tx=Pin(CommConfig.TX_PIN_TO_MAIN),
            rx=Pin(CommConfig.RX_PIN_FROM_MAIN),
        )

        self.buf = bytearray(128)
        self.latest_channels = [CrsfConfig.RC_CENTER] * CrsfConfig.CHANNEL_NUM

        # ====================【修改點 2】====================
        # 將原本 omega 的追蹤變數改為 10 個通道的列表
        self.last_valid_raw = [CrsfConfig.RC_CENTER] * 10
        self.last_final = [0] * 10
        # ====================================================

        print("Remote receiver initialized.")
        print(
            f"Listening to DBR4 on UART {CrsfConfig.UART_ID} (RX:{CrsfConfig.RX_PIN_DBR4})"
        )
        print(
            f"Transmitting commands on UART {CommConfig.UART_ID} (TX:{CommConfig.TX_PIN_TO_MAIN})"
        )

    def _parse_channels(self, data: memoryview):
        if len(data) < 22:
            return None
        values, bits, bitcount = [], 0, 0
        for b in data:
            bits |= b << bitcount
            bitcount += 8
            while bitcount >= 11:
                values.append(bits & 0x7FF)
                bits >>= 11
                bitcount -= 11
        return values[: CrsfConfig.CHANNEL_NUM]

    def _poll_dbr4_uart(self):
        n = self.dbr4_uart.readinto(self.buf)
        if not n:
            return None
        data = memoryview(self.buf)[:n]
        for i in range(len(data)):
            if data[i] == CrsfConfig.SYNC_BYTE and i + 2 < len(data):
                length = data[i + 1]
                if i + 2 + length <= len(data):
                    payload = data[i + 2 : i + 2 + length]
                    if payload and payload[0] == CrsfConfig.TYPE_CHANNELS:
                        return self._parse_channels(payload[1:-1])
        return None

    def _normalize(self, val: int) -> int:
        val = max(
            CrsfConfig.RC_CENTER - CrsfConfig.RC_RANGE,
            min(val, CrsfConfig.RC_CENTER + CrsfConfig.RC_RANGE),
        )
        return int((val - CrsfConfig.RC_CENTER) * 100 / CrsfConfig.RC_RANGE)

    def _deadband(self, val: int) -> int:
        return 0 if abs(val) < CrsfConfig.RC_DEADBAND else val

    # ====================【修改點 3】====================
    # 將過濾器邏輯套用到所有 10 個通道
    def update_and_transmit(self):
        ch = self._poll_dbr4_uart()
        if ch:
            self.latest_channels = ch

        processed_values = []

        # 迴圈處理 10 個通道
        for i in range(10):
            raw_val = self.latest_channels[i]

            # 1. 計算與 "上次" 原始值的變化量
            change = abs(raw_val - self.last_valid_raw[i])

            # 2. 檢查變化量是否過大 (判斷為突波)
            if change > CrsfConfig.MAX_RAW_CHANGE_PER_STEP:
                # 變化太大，視為雜訊。
                # *使用* 上次處理過的值 (self.last_final[i])
                final_val = self.last_final[i]
            else:
                # 變化在允許範圍內 (正常操作或微小雜訊)
                # *處理* 這次的新值
                final_val = self._deadband(self._normalize(raw_val))
                # 並更新「上次處理過的值」
                self.last_final[i] = final_val

            # 3. 【關鍵邏輯】: *無論如何* 都要更新 "上次" 原始值
            # 這樣才能在下一個循環中正確比較
            # (如果是突波，下個循環會偵測到 "跳回來" 的巨大變化，再次過濾)
            # (如果是開關，下個循環 change 會是 0，就會採用新值)
            self.last_valid_raw[i] = raw_val

            # 4. 將最終值存入列表
            processed_values.append(final_val)

        # 5. 將 10 個值轉換為字串並發送
        command_string = ",".join(map(str, processed_values)) + "\n"

        # ====================================================

        # ====================【印出最終封包】====================
        # print(f"{command_string.strip()}")
        # ==========================================================

        # 0     1      2     3      4   5   6   7   8   9
        # 右水平 右垂直  左垂直 左水平  SA  SD  SB  SC  SE  SI

        # 透過 UART 發送給主控板
        self.main_uart.write(command_string.encode("utf-8"))


# ==================== 主程式入口 ====================
if __name__ == "__main__":
    led = Pin("LED", Pin.OUT)
    transmitter = RemoteControlTransmitter()

    while True:
        try:
            led.toggle()
            transmitter.update_and_transmit()
            utime.sleep_ms(20)
        except KeyboardInterrupt:
            print("Program stopped.")
            break
        except Exception as e:
            print(f"An error occurred: {e}")

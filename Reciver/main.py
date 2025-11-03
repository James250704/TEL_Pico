# -*- coding: utf-8 -*-
# 檔案名稱: remote_receiver.py
# 功能: 專門接收 DBR4 遙控器訊號，並將指令透過 UART 傳送給主控 Pico
# 版本: 1.9 (為 CH9 設定 20 的死區，其餘通道設定 10 的死區)

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

    # ====================【修改點 1】====================
    # 設定不同的死區
    RC_DEADBAND_DEFAULT = 3  # (中文：CH0-8 的預設死區)
    RC_DEADBAND_CH9 = 15  # (中文：CH9 (旋轉) 的死區)
    # ====================================================

    # 【v1.7】雜訊過濾閾值
    MAX_RAW_CHANGE_PER_STEP = 200

    # 【v1.8】Failsafe 逾時設定
    FAILSAFE_TIMEOUT_MS = 500


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

        # 【v1.7】: 雜訊過濾器狀態
        self.last_valid_raw = [CrsfConfig.RC_CENTER] * 10
        self.last_final = [0] * 10

        # 【v1.8】: Failsafe 狀態變數
        self.last_packet_time = utime.ticks_ms()
        self.failsafe_active = False

        print("Remote receiver initialized.")
        # (中文：遙控接收器已初始化。)
        print(
            f"Listening to DBR4 on UART {CrsfConfig.UART_ID} (RX:{CrsfConfig.RX_PIN_DBR4})"
        )
        # (中文：正在 UART 0 (RX:17) 上監聽 DBR4 訊號)
        print(
            f"Transmitting commands on UART {CommConfig.UART_ID} (TX:{CommConfig.TX_PIN_TO_MAIN})"
        )
        # (中文：正在 UART 1 (TX:8) 上傳送指令)

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

    # ====================【修改點 2】====================
    # 函式簽名增加 channel_index 參數
    def _deadband(self, val: int, channel_index: int) -> int:
        # 根據通道索引套用不同的死區
        if channel_index == 9:
            threshold = CrsfConfig.RC_DEADBAND_CH9
        else:
            threshold = CrsfConfig.RC_DEADBAND_DEFAULT

        return 0 if abs(val) < threshold else val

    # ====================================================

    def update_and_transmit(self):
        ch = self._poll_dbr4_uart()

        # 1. (v1.8) 檢查是否有新封包
        if ch:
            self.latest_channels = ch
            self.last_packet_time = utime.ticks_ms()

        # 2. (v1.8) 檢查 Failsafe 逾時
        current_time = utime.ticks_ms()
        time_since_last_packet = utime.ticks_diff(current_time, self.last_packet_time)

        processed_values = []

        if time_since_last_packet > CrsfConfig.FAILSAFE_TIMEOUT_MS:
            # *** 進入 Failsafe 狀態 ***
            if not self.failsafe_active:
                print("FAILSAFE: RC signal lost. Setting all channels to 0.")
                # (中文：Failsafe：遙控訊號遺失。將所有通道設為 0。)
                self.failsafe_active = True

            processed_values = [0] * 10
            self.last_valid_raw = [CrsfConfig.RC_CENTER] * 10
            self.last_final = [0] * 10

        else:
            # *** 訊號正常 ***
            if self.failsafe_active:
                print("FAILSAFE: RC signal recovered.")
                # (中文：Failsafe：遙控訊號已恢復。)
                self.failsafe_active = False

            # (v1.7) 迴圈處理 10 個通道
            for i in range(10):
                raw_val = self.latest_channels[i]

                # (v1.7) 雜訊過濾
                change = abs(raw_val - self.last_valid_raw[i])

                if change > CrsfConfig.MAX_RAW_CHANGE_PER_STEP:
                    final_val = self.last_final[i]
                else:
                    # ====================【修改點 3】====================
                    # 呼叫 _deadband 時，傳入通道索引 i
                    normalized_val = self._normalize(raw_val)
                    final_val = self._deadband(normalized_val, i)
                    # ====================================================

                    self.last_final[i] = final_val

                self.last_valid_raw[i] = raw_val
                processed_values.append(final_val)

        # 3. 將 10 個值轉換為字串並發送
        command_string = ",".join(map(str, processed_values)) + "\n"

        self.main_uart.write(command_string.encode("utf-8"))

        print(f"{command_string.strip()}")

        # 0     1      2     3      4   5   6   7   8   9
        # 右水平 右垂直  左垂直 左水平  SA  SD  SB  SC  SE  SI


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
            # (中文：程式已停止。)
            break
        except Exception as e:
            print(f"An error occurred: {e}")
            # (中文：發生錯誤：{e})
            utime.sleep_ms(1000)

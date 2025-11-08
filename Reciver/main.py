# -*- coding: utf-8 -*-
# 檔案名稱: remote_receiver.py
# 功能: 專門接收 DBR4 遙控器訊號，並將指令透過 UART 傳送給主控 Pico
# 版本: 2.3 (新增 CRSF CRC-8 封包校驗，修復雜訊導致的 "抽一下" 問題)

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
    RC_DEADBAND_DEFAULT = 3
    RC_DEADBAND_CH9 = 15
    MAX_RAW_CHANGE_PER_STEP = 200
    FAILSAFE_TIMEOUT_MS = 500


class CommConfig:
    UART_ID = 1
    BAUDRATE = 115200
    TX_PIN_TO_MAIN = 8
    RX_PIN_FROM_MAIN = 9


# ==================== 遙控器訊號處理 ====================
class RemoteControlTransmitter:
    def __init__(self):
        # ( ... __init__ 內容保持不變 ...)
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
        self.last_valid_raw = [CrsfConfig.RC_CENTER] * 10
        self.last_final = [0] * 10
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

    # ====================【修改點 1: 新增 CRC 函式】====================
    def _crc8_d5(self, data: memoryview) -> int:
        """
        (中文：計算 CRSF 的 CRC-8/D5 校驗碼)
        (Calculate CRSF CRC-8/D5 checksum)
        """
        crc = 0
        for byte in data:
            crc ^= byte
            for _ in range(8):
                if crc & 0x80:
                    crc = (crc << 1) ^ 0xD5
                else:
                    crc = crc << 1
                crc &= 0xFF
        return crc

    # ===============================================================

    def _parse_channels(self, data: memoryview):
        # ( ... 此函式保持不變 ...)
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

                    # ====================【修改點 2: 執行 CRC 校驗】====================

                    # (中文：提取完整的 CRSF 封包 (包含 Type 到 CRC))
                    # (Extract the full CRSF packet (from Type to CRC))
                    payload_with_crc = data[i + 2 : i + 2 + length]

                    # (中文：要計算 CRC 的部分 (從 Type 到 Channel data))
                    # (The part to calculate CRC on (from Type to Channel data))
                    payload_data = payload_with_crc[:-1]

                    # (中文：封包末尾的 CRC 字節)
                    # (The CRC byte at the end of the packet)
                    payload_crc_byte = payload_with_crc[-1]

                    # (中文：計算我們收到的數據的 CRC)
                    # (Calculate the CRC of the data we received)
                    calculated_crc = self._crc8_d5(payload_data)

                    # (中文：如果計算出的 CRC 與封包中的 CRC 不符，丟棄這個封包)
                    # (If calculated CRC does not match the packet's CRC, drop this packet)
                    if calculated_crc != payload_crc_byte:
                        # print("CRC Error! Packet dropped.")
                        # (中文：CRC 錯誤！封包已丟棄。)
                        continue  # (中文：跳過此損毀封包)

                    # (中文：CRC 校驗通過！ payload_data 是有效的。)
                    # (CRC OK! payload_data is valid.)
                    payload = payload_data
                    # ===============================================================

                    if payload and payload[0] == CrsfConfig.TYPE_CHANNELS:
                        # (中文：注意：我們現在傳入 payload[1:] 而不是 payload[1:-1])
                        # (Note: We now pass payload[1:] instead of payload[1:-1])
                        return self._parse_channels(payload[1:])
        return None

    def _normalize(self, val: int) -> int:
        # ( ... 此函式保持不變 ...)
        val = max(
            CrsfConfig.RC_CENTER - CrsfConfig.RC_RANGE,
            min(val, CrsfConfig.RC_CENTER + CrsfConfig.RC_RANGE),
        )
        return int((val - CrsfConfig.RC_CENTER) * 100 / CrsfConfig.RC_RANGE)

    def _normalize_throttle(self, val: int) -> int:
        # ( ... 此函式保持不變 ...)
        min_raw = CrsfConfig.RC_CENTER - CrsfConfig.RC_RANGE  # (172)
        max_raw = CrsfConfig.RC_CENTER + CrsfConfig.RC_RANGE  # (1812)
        val_clamped = max(min_raw, min(val, max_raw))
        total_range = CrsfConfig.RC_RANGE * 2  # (1640)
        percentage = (val_clamped - min_raw) * 100.0 / total_range
        return int(round(percentage))

    def _deadband(self, val: int, channel_index: int) -> int:
        # ( ... 此函式保持不變 ...)
        if channel_index == 9:
            threshold = CrsfConfig.RC_DEADBAND_CH9
        else:
            threshold = CrsfConfig.RC_DEADBAND_DEFAULT
        return 0 if abs(val) < threshold else val

    def update_and_transmit(self):
        # ( ... 此函式 (v2.2) 邏輯完全不變 ...)
        ch = self._poll_dbr4_uart()

        if ch:
            self.latest_channels = ch
            self.last_packet_time = utime.ticks_ms()

        current_time = utime.ticks_ms()
        time_since_last_packet = utime.ticks_diff(current_time, self.last_packet_time)

        processed_values = []

        if time_since_last_packet > CrsfConfig.FAILSAFE_TIMEOUT_MS:
            if not self.failsafe_active:
                print("FAILSAFE: RC signal lost. Setting all channels to 0.")
                # (中文：Failsafe：遙控訊號遺失。將所有通道設為 0。)
                self.failsafe_active = True
            processed_values = [0] * 10
            self.last_valid_raw = [CrsfConfig.RC_CENTER] * 10
            self.last_final = [0] * 10
        else:
            if self.failsafe_active:
                print("FAILSAFE: RC signal recovered.")
                # (中文：Failsafe：遙控訊號已恢復。)
                self.failsafe_active = False

            for i in range(10):
                raw_val = self.latest_channels[i]
                change = abs(raw_val - self.last_valid_raw[i])

                if change > CrsfConfig.MAX_RAW_CHANGE_PER_STEP:
                    final_val = self.last_final[i]
                else:
                    if i == 2:  # (CH2, 左垂直, 油門)
                        final_val = self._normalize_throttle(raw_val)
                        self.last_final[i] = final_val
                    else:
                        normalized_val = self._normalize(raw_val)
                        final_val = self._deadband(normalized_val, i)
                        self.last_final[i] = final_val

                if i == 6:  # CH6 (SB)
                    final_val = -final_val

                self.last_valid_raw[i] = raw_val
                processed_values.append(final_val)

        command_string = ",".join(map(str, processed_values)) + "\n"
        self.main_uart.write(command_string.encode("utf-8"))
        print(f"{command_string.strip()}")


# ==================== 主程式入口 ====================
if __name__ == "__main__":
    # ( ... 此函式保持不變 ...)
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

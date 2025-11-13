# -*- coding: utf-8 -*-
# 檔案名稱: remote_receiver.py
# 功能: 接收 DBR4 訊號 -> UART 轉發 -> 具備記憶體優化 (移除 WDT)
# 版本: 2.5 (級別 1 記憶體優化，無看門狗)

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
        self.processed_values = [0] * 10

        print("Remote receiver initialized.")
        print(
            f"Listening to DBR4 on UART {CrsfConfig.UART_ID} (RX:{CrsfConfig.RX_PIN_DBR4})"
        )
        print(
            f"Transmitting commands on UART {CommConfig.UART_ID} (TX:{CommConfig.TX_PIN_TO_MAIN})"
        )

    def _crc8_d5(self, data: memoryview) -> int:
        """計算 CRSF CRC-8/D5 校驗碼"""
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
                    payload_with_crc = data[i + 2 : i + 2 + length]
                    payload_data = payload_with_crc[:-1]
                    payload_crc_byte = payload_with_crc[-1]
                    calculated_crc = self._crc8_d5(payload_data)

                    if calculated_crc != payload_crc_byte:
                        continue

                    payload = payload_data
                    if payload and payload[0] == CrsfConfig.TYPE_CHANNELS:
                        return self._parse_channels(payload[1:])
        return None

    def _normalize(self, val: int) -> int:
        val = max(
            CrsfConfig.RC_CENTER - CrsfConfig.RC_RANGE,
            min(val, CrsfConfig.RC_CENTER + CrsfConfig.RC_RANGE),
        )
        return int((val - CrsfConfig.RC_CENTER) * 100 / CrsfConfig.RC_RANGE)

    def _normalize_throttle(self, val: int) -> int:
        min_raw = CrsfConfig.RC_CENTER - CrsfConfig.RC_RANGE
        max_raw = CrsfConfig.RC_CENTER + CrsfConfig.RC_RANGE
        val_clamped = max(min_raw, min(val, max_raw))
        total_range = CrsfConfig.RC_RANGE * 2
        percentage = (val_clamped - min_raw) * 100.0 / total_range
        return int(round(percentage))

    def _deadband(self, val: int, channel_index: int) -> int:
        if channel_index == 9:
            threshold = CrsfConfig.RC_DEADBAND_CH9
        else:
            threshold = CrsfConfig.RC_DEADBAND_DEFAULT
        return 0 if abs(val) < threshold else val

    def update_and_transmit(self):
        ch = self._poll_dbr4_uart()

        if ch:
            self.latest_channels = ch
            self.last_packet_time = utime.ticks_ms()

        current_time = utime.ticks_ms()
        time_since_last_packet = utime.ticks_diff(current_time, self.last_packet_time)

        if time_since_last_packet > CrsfConfig.FAILSAFE_TIMEOUT_MS:
            if not self.failsafe_active:
                print("FAILSAFE: RC signal lost. Setting all channels to 0.")
                self.failsafe_active = True

            for i in range(10):
                self.processed_values[i] = 0

            self.last_valid_raw = [CrsfConfig.RC_CENTER] * 10
            self.last_final = [0] * 10
        else:
            if self.failsafe_active:
                print("FAILSAFE: RC signal recovered.")
                self.failsafe_active = False

            for i in range(10):
                raw_val = self.latest_channels[i]
                change = abs(raw_val - self.last_valid_raw[i])

                if change > CrsfConfig.MAX_RAW_CHANGE_PER_STEP:
                    final_val = self.last_final[i]
                else:
                    if i == 2:  # Throttle
                        final_val = self._normalize_throttle(raw_val)
                        self.last_final[i] = final_val
                    else:
                        normalized_val = self._normalize(raw_val)
                        final_val = self._deadband(normalized_val, i)
                        self.last_final[i] = final_val

                if i == 6:  # Invert CH6
                    final_val = -final_val

                self.last_valid_raw[i] = raw_val

                # 【優化點應用】: 直接賦值給預先配置的列表
                self.processed_values[i] = final_val

        # 使用優化後的列表進行字串組合
        command_string = ",".join(map(str, self.processed_values)) + "\n"
        self.main_uart.write(command_string.encode("utf-8"))
        # print(f"{command_string.strip()}")


# ==================== 主程式入口 ====================
if __name__ == "__main__":
    try:
        led = Pin("LED", Pin.OUT)
    except:
        led = Pin(25, Pin.OUT)

    transmitter = RemoteControlTransmitter()

    print("System Started (Memory Optimized, No WDT).")

    while True:
        try:
            led.toggle()
            transmitter.update_and_transmit()
            utime.sleep_ms(5)

        except KeyboardInterrupt:
            print("Program stopped.")
            break

        except Exception as e:
            print(f"An error occurred: {e}")
            utime.sleep_ms(500)

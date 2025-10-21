# -*- coding: utf-8 -*-
# 檔案名稱: remote_receiver.py
# 功能: 專門接收 DBR4 遙控器訊號，並將指令透過 UART 傳送給主控 Pico
# 版本: 1.4 (加入最終發送封包的打印功能)

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
    RC_DEADBAND = 30
    MAX_RAW_CHANGE_PER_STEP = 800


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

        self.last_valid_raw_omega = CrsfConfig.RC_CENTER
        self.last_final_omega = 0

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

    def update_and_transmit(self):
        ch = self._poll_dbr4_uart()
        if ch:
            self.latest_channels = ch

        vx = self._deadband(self._normalize(self.latest_channels[1]))
        vy = self._deadband(self._normalize(self.latest_channels[0]))
        pitch = self._deadband(self._normalize(self.latest_channels[2]))
        pan = self._deadband(self._normalize(self.latest_channels[3]))

        omega_raw = self.latest_channels[9]
        change = abs(omega_raw - self.last_valid_raw_omega)

        if change > CrsfConfig.MAX_RAW_CHANGE_PER_STEP:
            omega = self.last_final_omega
        else:
            omega = self._deadband(self._normalize(omega_raw))
            self.last_valid_raw_omega = omega_raw
            self.last_final_omega = omega

        # 為了除錯，暫時關閉之前的 omega debug print
        # print(f"Omega Debug -> Raw CH9: {omega_raw}, Final Omega: {omega}")

        command_string = f"CMD:{pan},{pitch},{omega},{vx},{vy}\n"

        # ====================【新增：印出最終封包】====================
        # .strip() 是為了讓序列埠監控的輸出更整潔，不影響實際發送的 "\n"
        print(f"Final Packet Sent: {command_string.strip()}")
        # ==========================================================

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

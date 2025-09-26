# -*- coding: utf-8 -*-
# 檔案名稱: remote_receiver.py
# 功能: 專門接收 DBR4 遙控器訊號，並將指令透過 UART 傳送給主控 Pico

from machine import Pin, UART
import utime

# ==================== 常數與設定 (Constants & Configuration) ====================


# --- DBR4 CRSF 接收器設定 ---
class CrsfConfig:
    UART_ID = 1
    BAUDRATE = 420000
    TX_PIN_DBR4 = 8
    RX_PIN_DBR4 = 9
    SYNC_BYTE = 0xC8
    TYPE_CHANNELS = 0x16
    CHANNEL_NUM = 16
    RC_CENTER = 992
    RC_RANGE = 820
    RC_DEADBAND = 30


# --- 與主控 Pico 通訊的 UART 設定 ---
class CommConfig:
    UART_ID = 0
    BAUDRATE = 115200
    TX_PIN_TO_MAIN = 16  # 這個 TX 要接到主控板的 RX
    RX_PIN_FROM_MAIN = 17  # 這個 RX 接到主控板的 TX (備用)


# ==================== 遙控器訊號處理 ====================
class RemoteControlTransmitter:
    """
    處理 CRSF 遙控訊號的接收、解析，並將其格式化後傳送出去
    """

    def __init__(self):
        # 初始化 DBR4 接收器 UART
        self.dbr4_uart = UART(
            CrsfConfig.UART_ID,
            baudrate=CrsfConfig.BAUDRATE,
            tx=Pin(CrsfConfig.TX_PIN_DBR4),
            rx=Pin(CrsfConfig.RX_PIN_DBR4),
            bits=8,
            parity=None,
            stop=1,
        )

        # 初始化與主控 Pico 通訊的 UART
        self.main_uart = UART(
            CommConfig.UART_ID,
            baudrate=CommConfig.BAUDRATE,
            tx=Pin(CommConfig.TX_PIN_TO_MAIN),
            rx=Pin(CommConfig.RX_PIN_FROM_MAIN),
        )

        self.buf = bytearray(128)
        self.latest_channels = [CrsfConfig.RC_CENTER] * CrsfConfig.CHANNEL_NUM
        print("Remote receiver initialized.")  # 中文解釋: 遙控接收器已初始化。
        print(
            f"Listening to DBR4 on UART {CrsfConfig.UART_ID} (RX:{CrsfConfig.RX_PIN_DBR4})"
        )  # 中文解釋: 正在 UART {CrsfConfig.UART_ID} (RX:{CrsfConfig.RX_PIN_DBR4}) 上監聽 DBR4 訊號
        print(
            f"Transmitting commands on UART {CommConfig.UART_ID} (TX:{CommConfig.TX_PIN_TO_MAIN})"
        )  # 中文解釋: 正在 UART {CommConfig.UART_ID} (TX:{CommConfig.TX_PIN_TO_MAIN}) 上傳送指令

    def _parse_channels(self, data: memoryview):
        """解析 CRSF channel 資料"""
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
        """從 DBR4 UART buffer 擷取並解析 CRSF 封包"""
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
        """將 CRSF 原始值轉成 -100 ~ +100"""
        return int((val - CrsfConfig.RC_CENTER) * 100 / CrsfConfig.RC_RANGE)

    def _deadband(self, val: int) -> int:
        """應用搖桿死區"""
        return 0 if abs(val) < CrsfConfig.RC_DEADBAND else val

    def update_and_transmit(self):
        """主更新函式：讀取遙控器，處理訊號，並將指令傳送給主控 Pico"""
        ch = self._poll_dbr4_uart()
        if ch:
            self.latest_channels = ch

        # CH0 (vy), CH1 (vx) 控制機器人平台的輪胎控制
        vx = self._deadband(self._normalize(self.latest_channels[1]))  # Y軸 (前後)
        vy = self._deadband(self._normalize(self.latest_channels[0]))  # X軸 (左右)

        # CH9 控制機器人底盤的原地旋轉
        omega = self._deadband(self._normalize(self.latest_channels[9]))  # 旋轉

        # CH2 (pan), CH3 (pitch) 要控制由伺服馬達驅動的射擊雲台
        pan = self._deadband(self._normalize(self.latest_channels[3]))  # 水平
        pitch = self._deadband(self._normalize(self.latest_channels[2]))  # 俯仰

        # 格式化指令字串，例如："CMD:-100,50,0,80,-25\n"
        # 格式: "CMD:vx,vy,omega,pan,pitch\n"
        command_string = f"CMD:{vx},{vy},{omega},{pan},{pitch}\n"

        # 透過 UART 發送給主控板，確保完整發送
        try:
            self.main_uart.write(command_string.encode("utf-8"))
            # 短暫延遲確保傳送完成
            utime.sleep_ms(1)
        except:
            pass  # 忽略傳送錯誤

        # print(f"Sent: {command_string.strip()}") # 用於除錯，可以取消註解


# ==================== 主程式入口 ====================
if __name__ == "__main__":
    led = Pin(25, Pin.OUT)
    transmitter = RemoteControlTransmitter()

    while True:
        try:
            led.toggle()
            transmitter.update_and_transmit()
            utime.sleep_ms(10)  # 每 10ms 發送一次指令，相當於 100Hz
        except KeyboardInterrupt:
            print("Program stopped.")  # 中文解釋: 程式已停止。
            break

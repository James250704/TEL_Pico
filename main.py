# -*- coding: utf-8 -*-
import sys
import utime
from machine import Pin, PWM, Timer, UART
import time


# 常數與設定 (Constants & Configuration)
# DBR4 CRSF 接收器設定
class CrsfConfig:
    UART_ID = 0
    BAUDRATE = 420000
    TX_PIN = 16
    RX_PIN = 17
    SYNC_BYTE = 0xC8
    TYPE_CHANNELS = 0x16
    CHANNEL_NUM = 16
    RC_CENTER = 992
    RC_RANGE = 820


# --- 機器人硬體設定 ---
class HardwareConfig:
    # (PWM Pin, DIR Pin, Reversed)
    MOTOR_PINS = [
        (2, 3, False),  # 0: 左前輪 (LF)
        (6, 7, True),  # 1: 右前輪 (RF)
        (8, 9, False),  # 2: 右後輪 (RR)
        (4, 5, True),  # 3: 左後輪 (LR)
    ]
    # (Encoder A Pin)
    ENCODER_PINS = [
        12,  # 0: LF Encoder
        13,  # 1: RF Encoder
        14,  # 2: RR Encoder
        15,  # 3: LR Encoder
    ]


# --- 機器人參數 ---
class RobotParams:
    CONTROL_DT_MS = 8  # 控制迴圈間隔 (ms)
    # SPEED_LOG_DT_MS = 2000  # 速度日誌列印間隔 (ms)
    PPR = 16  # Encoder 每轉脈衝數 (Pulses Per Revolution)
    PID_KP = 0.25
    PID_KI = 0.0
    PID_KD = 0.0
    PID_OUT_LIM = 50.0  # PID 輸出限制 (百分比)
    MAX_DUTY = 40000  # PWM 最大 Duty
    DEFAULT_MOTOR_SCALE = [1.0105, 0.9965, 0.9931, 1.0001]  # 馬達校準值


# ==================== Radio 遙控器轉換器 ====================
class RadioControl:
    """處理 CRSF 遙控訊號的接收與解析"""

    RC_DEADBAND = 30

    def __init__(self, robot: "MecanumRobot"):
        self.robot = robot
        self.uart = UART(
            CrsfConfig.UART_ID,
            baudrate=CrsfConfig.BAUDRATE,
            tx=Pin(CrsfConfig.TX_PIN),
            rx=Pin(CrsfConfig.RX_PIN),
            bits=8,
            parity=None,
            stop=1,
        )
        self.buf = bytearray(128)
        self.latest_channels = [CrsfConfig.RC_CENTER] * CrsfConfig.CHANNEL_NUM

    def _parse_channels(self, data: memoryview) -> "Optional[List[int]]":
        """解析 CRSF channel 資料 (22 bytes)"""
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

    def _poll_uart(self) -> "Optional[List[int]]":
        """從 UART buffer 擷取並解析 CRSF 封包"""
        n = self.uart.readinto(self.buf)
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
        return 0 if abs(val) < self.RC_DEADBAND else val

    def update(self):
        """主更新函式：讀取遙控器並控制機器人"""
        ch = self._poll_uart()
        if ch:
            self.latest_channels = ch

        # 讀取遙控器輸入
        vx = self._deadband(self._normalize(self.latest_channels[1]))  # Y軸控制前後
        vy = self._deadband(self._normalize(self.latest_channels[0]))  # X軸控制左右
        omega = self._deadband(self._normalize(self.latest_channels[3]))  # 旋轉控制

        # 將遙控器數據直接轉換為運動學指令
        self.robot.apply_kinematics(vx, vy, omega)


# ==================== PID 控制器 ====================
class PID:
    def __init__(
        self,
        kp: float,
        ki: float,
        kd: float,
        out_limit: float,
        integral_zone: float = 50.0,
        slew_rate: float = 1000.0,
        d_filter_alpha: float = 0.2,
    ):
        self.kp, self.ki, self.kd = kp, ki, kd
        self.out_limit = float(out_limit)
        self.integral_zone = integral_zone
        self.slew_rate = slew_rate
        self.d_filter_alpha = d_filter_alpha
        self.reset()

    def reset(self):
        self.i = 0.0
        self.prev_e = 0.0
        self.prev_u = 0.0
        self.d_filtered = 0.0

    def update(self, e: float, dt: float) -> float:
        """根據誤差 e 和時間間隔 dt 計算 PID 輸出"""
        if dt <= 1e-9:
            return self.prev_u

        # --- 積分項 (Integral) ---
        if abs(e) < self.integral_zone and abs(self.prev_u) < self.out_limit * 0.95:
            self.i += e * dt

        # --- 微分項 (Derivative) ---
        de = (e - self.prev_e) / dt
        self.d_filtered = (
            1 - self.d_filter_alpha
        ) * self.d_filtered + self.d_filter_alpha * de
        self.prev_e = e

        # --- PID 輸出組合 ---
        out = self.kp * e + self.ki * self.i + self.kd * self.d_filtered

        # --- 輸出處理 ---
        du = out - self.prev_u
        max_du = self.slew_rate * dt
        du = max(-max_du, min(max_du, du))
        out = self.prev_u + du

        out = max(-self.out_limit, min(self.out_limit, out))

        self.prev_u = out
        return out


# ==================== 硬體抽象層 ====================
class Motor:
    """單一馬達的 PWM 與方向控制"""

    def __init__(
        self, pwm_pin: int, dir_pin: int, reversed: bool = False, max_duty: int = 40000
    ):
        self.pwm = PWM(Pin(pwm_pin))
        self.pwm.freq(1000)
        self.dir = Pin(dir_pin, Pin.OUT)
        self.reversed = reversed
        self.max_duty = max_duty

    def set_speed_percent(self, pct: float):
        p = max(-100.0, min(100.0, pct))
        if self.reversed:
            p = -p

        self.dir.value(1 if p > 0 else 0)
        duty = int(abs(p) / 100.0 * self.max_duty)
        self.pwm.duty_u16(min(65535, duty))

    def stop(self):
        self.pwm.duty_u16(0)


class SafeEncoder:
    """線程安全 (中斷安全) 的編碼器計數器"""

    def __init__(self, pin_a: int):
        self.pulse_count = 0
        try:
            self.pin = Pin(pin_a, Pin.IN, Pin.PULL_UP)
            self.pin.irq(trigger=Pin.IRQ_RISING, handler=self._irq_handler)
        except (ImportError, NameError):
            self.pin = None

    def _irq_handler(self, pin):
        self.pulse_count += 1

    def take_delta(self) -> int:
        """原子操作：取出計數並清零"""
        try:
            state = machine.disable_irq()
            count = self.pulse_count
            self.pulse_count = 0
            machine.enable_irq(state)
        except (ImportError, NameError):
            count = self.pulse_count
            self.pulse_count = 0
        return count

    def deinit(self):
        if self.pin:
            self.pin.irq(handler=None)


# ==================== 麥克納姆輪機器人主類別 (整合全向運動學) ====================
class MecanumRobot:
    def __init__(self, params: RobotParams):
        self.params = params
        self.motors = [
            Motor(*pins, max_duty=params.MAX_DUTY) for pins in HardwareConfig.MOTOR_PINS
        ]
        self.encs = [SafeEncoder(pin) for pin in HardwareConfig.ENCODER_PINS]

        self.pids = [
            PID(params.PID_KP, params.PID_KI, params.PID_KD, params.PID_OUT_LIM)
            for _ in range(4)
        ]

        self.current_scale = 100.0
        self.enable_pid = True
        self.base_cmd = [0, 0, 0, 0]
        self.motor_scale = list(params.DEFAULT_MOTOR_SCALE)
        self.rps_filtered = [0.0] * 4

        self.RPS_FILTER_ALPHA = 0.2
        self.MAX_LOG_LEN = 200
        self.pid_ignore_ms = 0
        self.switch_ms = utime.ticks_ms()
        self._last_us = utime.ticks_us()
        self._last_speed_log_ms = utime.ticks_ms()

        self._timer = Timer()
        self._timer.init(
            period=self.params.CONTROL_DT_MS,
            mode=Timer.PERIODIC,
            callback=self._timer_cb,
        )

    def _timer_cb(self, t: Timer):
        """定時器中斷回呼函式"""
        self._scheduled_update(0)

    def _scheduled_update(self, _arg):
        """核心控制迴圈，計算 RPS 並更新 PID"""
        now = utime.ticks_us()
        dt = utime.ticks_diff(now, self._last_us) / 1_000_000.0
        if dt <= 1e-9:
            return
        self._last_us = now

        pulses = [e.take_delta() for e in self.encs]
        raw_rps = [abs(p) / (self.params.PPR * dt) for p in pulses]
        for i in range(4):
            self.rps_filtered[i] = (1 - self.RPS_FILTER_ALPHA) * self.rps_filtered[
                i
            ] + self.RPS_FILTER_ALPHA * raw_rps[i]

        w = [abs(x) for x in self.base_cmd]
        sgn = [1 if x > 0 else (-1 if x < 0 else 0) for x in self.base_cmd]

        idx_pos = [i for i, val in enumerate(self.base_cmd) if val > 0]
        idx_neg = [i for i, val in enumerate(self.base_cmd) if val < 0]

        def mean_norm(idxs: list) -> float:
            if not idxs:
                return 0.0
            vals = [(self.rps_filtered[i] / max(1.0, w[i])) for i in idxs]
            return sum(vals) / len(vals)

        mn_pos = mean_norm(idx_pos)
        mn_neg = mean_norm(idx_neg)

        for i in range(4):
            if w[i] < 1:
                self.motors[i].set_speed_percent(0)
                self.pids[i].reset()
                continue

            u_ff = (self.base_cmd[i] * self.current_scale) / 100.0
            mn = mn_pos if sgn[i] > 0 else mn_neg

            if mn <= 1e-6:
                self.motors[i].set_speed_percent(u_ff)
                self.pids[i].reset()
                continue

            err_pct_mag = w[i] - (self.rps_filtered[i] / mn)
            err_pct = sgn[i] * err_pct_mag

            elapsed = utime.ticks_diff(utime.ticks_ms(), self.switch_ms)
            use_pid = self.enable_pid and (elapsed >= self.pid_ignore_ms)
            du = self.pids[i].update(err_pct, dt) if use_pid else 0.0

            u = max(-100.0, min(100.0, u_ff + du))
            u *= self.motor_scale[i]
            self.motors[i].set_speed_percent(u)

    def set_command(self, target_cmd: list, ramp_ms: int = 20):
        """設定四輪目標速度的底層方法，可選斜坡加速"""
        for p in self.pids:
            p.reset()

        # 由於遙控器訊號是連續的，我們不需要手動的斜坡函數
        # 直接更新指令，PID 控制器會處理平滑性
        self.base_cmd = list(target_cmd)
        self.switch_ms = utime.ticks_ms()

    def apply_kinematics(self, vx: float, vy: float, omega: float):
        """
        根據機身速度(vx, vy)和角速度(omega)計算四輪目標速度百分比。
        """
        # 修正後的麥克納姆輪運動學公式
        # 注意: 這裡的符號需要與你的硬體接線和馬達反向設置匹配
        # 標準公式:
        # lf = vx + vy + omega
        # rf = vx - vy - omega
        # lr = vx - vy + omega
        # rr = vx + vy - omega

        # 根據你的硬體設置（MOTOR_PINS），vx 和 vy 的方向需要調整。
        # 假設 vx 是前後 (ch1), vy 是左右 (ch0), omega 是旋轉 (ch3)
        # 實際應用中，需要根據機器人的輪子安裝方向和馬達反向進行測試和調整。

        lf = vy + vx + omega
        rf = vy - vx - omega
        rr = vy - vx + omega
        lr = vy + vx - omega

        # 正規化，確保最大值不超過 100
        max_val = max(abs(lf), abs(rf), abs(rr), abs(lr))
        if max_val > 100.0:
            scale = 100.0 / max_val
            lf *= scale
            rf *= scale
            rr *= scale
            lr *= scale

        # 直接設定最終指令
        self.set_command([int(lf), int(rf), int(rr), int(lr)], ramp_ms=20)

    def deinit(self):
        if hasattr(self, "_timer"):
            self._timer.deinit()
        # 停止所有馬達
        for motor in self.motors:
            motor.stop()
        for enc in self.encs:
            enc.deinit()


# ==================== 主程式入口 ====================
if __name__ == "__main__":
    try:
        # 1. 初始化
        params = RobotParams()
        robot = MecanumRobot(params)
        radio = RadioControl(robot)
        led = Pin(25, Pin.OUT)

        # --- 遙控模式 (主迴圈) ---
        while True:
            led.value(1)
            radio.update()

    except KeyboardInterrupt:
        pass
    finally:
        if "robot" in locals():
            robot.deinit()

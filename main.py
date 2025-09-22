# -*- coding: utf-8 -*-
import sys
import utime
from machine import Pin, PWM, Timer, UART
import time


# ==================== 常數與設定 (Constants & Configuration) ====================
# --- DBR4 CRSF 接收器設定 ---
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
        (2, 3, True),  # 0: 左前輪 (LF)
        (6, 7, True),  # 1: 右前輪 (RF)
        (8, 9, False),  # 2: 右後輪 (RR)
        (4, 5, False),  # 3: 左後輪 (LR)
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
    CONTROL_DT_MS = 10  # 控制迴圈間隔 (ms)
    SPEED_LOG_DT_MS = 2000  # 速度日誌列印間隔 (ms)
    PPR = 16  # Encoder 每轉脈衝數 (Pulses Per Revolution)
    PID_KP = 0.25
    PID_KI = 0.0
    PID_KD = 0.0
    PID_OUT_LIM = 25.0  # PID 輸出限制 (百分比)
    MAX_DUTY = 40000  # PWM 最大 Duty
    # DEFAULT_MOTOR_SCALE = [0.9645, 0.9593, 0.6969, 0.7054]  # 馬達校準值
    # DEFAULT_MOTOR_SCALE = [1.0, 1.0, 1.0, 1.0]  # 馬達校準值
    DEFAULT_MOTOR_SCALE = [1.0105, 0.9965, 0.9931, 1.0001]  # 馬達校準值


# ==================== Radio 遙控器轉換器 (FSM 版) ====================
class RobotState:
    STOP = 0
    TURN_LEFT = 1
    TURN_RIGHT = 2
    MOVE = 3


class RadioControl:
    """處理 CRSF 遙控訊號的接收與解析 (FSM 版)"""

    RC_DEADBAND = 30
    RC_TURN_THRESHOLD = 1700
    RC_STOP_THRESHOLD = 1500

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
        self.state = RobotState.STOP

        # 狀態對應動作
        self.handlers = {
            RobotState.STOP: lambda: self.robot.set_command([0, 0, 0, 0], ramp_ms=50),
            RobotState.TURN_LEFT: lambda: self.robot.turn_left(ramp_ms=10),
            RobotState.TURN_RIGHT: lambda: self.robot.turn_right(ramp_ms=10),
            RobotState.MOVE: self._handle_move,
        }

    def _parse_channels(self, data: memoryview) -> Optional[List[int]]:
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

    def _poll_uart(self) -> Optional[List[int]]:
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

    def _handle_move(self):
        """一般移動 (麥克納姆運動學)"""
        x = self._deadband(self._normalize(self.latest_channels[0]))
        y = self._deadband(self._normalize(self.latest_channels[1]))
        lf, rf, rr, lr = y + x, y - x, y - x, y + x
        max_val = max(abs(lf), abs(rf), abs(rr), abs(lr), 100.0)
        lf, rf, rr, lr = [int(v * 100 / max_val) for v in (lf, rf, rr, lr)]
        self.robot.set_command([lf, rf, rr, lr], ramp_ms=10)

    def update(self):
        """主更新函式：決定狀態，執行狀態對應行為"""
        ch = self._poll_uart()
        if ch:
            self.latest_channels = ch

        # 讀取遙控器輸入
        x = self._deadband(self._normalize(self.latest_channels[0]))
        y = self._deadband(self._normalize(self.latest_channels[1]))
        ch5, ch6 = self.latest_channels[5], self.latest_channels[6]

        # 狀態轉換規則
        if (
            abs(x) < 10
            and abs(y) < 10
            and ch5 < self.RC_STOP_THRESHOLD
            and ch6 < self.RC_STOP_THRESHOLD
        ):
            self.state = RobotState.STOP
        elif ch5 > self.RC_TURN_THRESHOLD:
            self.state = RobotState.TURN_LEFT
        elif ch6 > self.RC_TURN_THRESHOLD:
            self.state = RobotState.TURN_RIGHT
        else:
            self.state = RobotState.MOVE

        # 執行對應行為
        self.handlers[self.state]()


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
        # 積分分離 + 抗積分飽和 (Anti-windup)
        # 只有當誤差和輸出都在一定範圍內時才累積積分
        if abs(e) < self.integral_zone and abs(self.prev_u) < self.out_limit * 0.95:
            self.i += e * dt

        # --- 微分項 (Derivative) ---
        de = (e - self.prev_e) / dt
        # 低通濾波，防止微分項因雜訊劇烈震盪
        self.d_filtered = (
            1 - self.d_filter_alpha
        ) * self.d_filtered + self.d_filter_alpha * de
        self.prev_e = e

        # --- PID 輸出組合 ---
        out = self.kp * e + self.ki * self.i + self.kd * self.d_filtered

        # --- 輸出處理 ---
        # 斜率限制 (Slew Rate Limiting)
        du = out - self.prev_u
        max_du = self.slew_rate * dt
        du = max(-max_du, min(max_du, du))
        out = self.prev_u + du

        # 輸出限幅 (Output Clamping)
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
            print(f"Encoder pin {pin_a} -> IRQ mode")
        except (ImportError, NameError):
            print("micropython module not found, running in non-IRQ-safe mode.")
            self.pin = None

    def _irq_handler(self, pin):
        # micropython.schedule(self._increment, None) # 如果計數複雜，可用 schedule
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
    # --- 離散運動向量定義 (用於 CLI 或預設動作) ---
    VEC_STOP = (0, 0, 0, 0)
    VEC_FORWARD = (100, 100, 100, 100)
    VEC_BACKWARD = (-100, -100, -100, -100)
    VEC_LEFT = (-100, 100, 100, -100)
    VEC_RIGHT = (100, -100, -100, 100)
    VEC_TURN_LEFT = (-100, 100, -100, 100)  # 原地左旋
    VEC_TURN_RIGHT = (100, -100, 100, -100)  # 原地右旋
    VEC_FORWARD_LEFT = (0, 100, 100, 0)
    VEC_FORWARD_RIGHT = (100, 0, 0, 100)
    VEC_BACKWARD_LEFT = (-100, 0, 0, -100)
    VEC_BACKWARD_RIGHT = (0, -100, -100, 0)

    def __init__(self, params: RobotParams):
        self.params = params
        print("🚀 Initializing mecanum robot...")

        # --- 初始化硬體 ---
        # 假設 HardwareConfig 類別已在程式碼上方定義
        self.motors = [
            Motor(*pins, max_duty=params.MAX_DUTY) for pins in HardwareConfig.MOTOR_PINS
        ]
        self.encs = [SafeEncoder(pin) for pin in HardwareConfig.ENCODER_PINS]

        self.pids = [
            PID(params.PID_KP, params.PID_KI, params.PID_KD, params.PID_OUT_LIM)
            for _ in range(4)
        ]

        # --- 狀態變數 ---
        self.current_scale = 100.0
        self.enable_pid = True
        self.base_cmd = [0, 0, 0, 0]
        self.motor_scale = list(params.DEFAULT_MOTOR_SCALE)
        self.rps_filtered = [0.0] * 4
        self.rps_log = []

        # --- 計時器與濾波 ---
        self.RPS_FILTER_ALPHA = 0.2
        self.MAX_LOG_LEN = 200
        self.pid_ignore_ms = 0
        self.switch_ms = utime.ticks_ms()
        self._last_us = utime.ticks_us()
        self._last_speed_log_ms = utime.ticks_ms()

        # --- 啟動主控制迴圈 ---
        self._timer = Timer()
        self._timer.init(
            period=self.params.CONTROL_DT_MS,
            mode=Timer.PERIODIC,
            callback=self._timer_cb,
        )
        print("✅ Mecanum robot controller initialized")

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

        # 1. 讀取編碼器，計算瞬時 RPS 並進行低通濾波
        pulses = [e.take_delta() for e in self.encs]
        raw_rps = [abs(p) / (self.params.PPR * dt) for p in pulses]
        for i in range(4):
            self.rps_filtered[i] = (1 - self.RPS_FILTER_ALPHA) * self.rps_filtered[
                i
            ] + self.RPS_FILTER_ALPHA * raw_rps[i]

        self.rps_log.append(list(self.rps_filtered))
        if len(self.rps_log) > self.MAX_LOG_LEN:
            self.rps_log.pop(0)

        # 2. PID 控制邏輯 (採用您獨特的相對誤差法)
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

        # 3. 更新每個馬達
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

        # 4. 定期列印日誌
        if (
            utime.ticks_diff(utime.ticks_ms(), self._last_speed_log_ms)
            >= self.params.SPEED_LOG_DT_MS
        ):
            if any(self.base_cmd):
                print("RPS:", [f"{r:.2f}" for r in self.rps_filtered])
            self._last_speed_log_ms = utime.ticks_ms()

    def set_command(self, target_cmd: list, ramp_ms: int = 100):
        """設定四輪目標速度的底層方法，可選斜坡加速"""
        for p in self.pids:
            p.reset()

        steps = max(1, int(ramp_ms // self.params.CONTROL_DT_MS))
        start_cmd = self.base_cmd[:]

        for k in range(1, steps + 1):
            alpha = k / steps
            self.base_cmd = [
                start_cmd[i] + (target_cmd[i] - start_cmd[i]) * alpha for i in range(4)
            ]
            utime.sleep_ms(self.params.CONTROL_DT_MS)

        self.base_cmd = list(target_cmd)
        self.switch_ms = utime.ticks_ms()

    # ==================== 新增：全向運動學控制介面 ====================
    def apply_kinematics(self, vx: float, vy: float, omega: float):
        """
        根據機身速度(vx, vy)和角速度(omega)計算四輪目標速度百分比。
        這是用於平滑遙控的主要方法。
        vx: 前進/後退速度 (-100 to 100)
        vy: 向左/向右平移速度 (-100 to 100)
        omega: 逆時針/順時針旋轉速度 (-100 to 100)
        """
        # 標準麥克納姆輪反向運動學公式 (X 型佈局)
        lf = vx - vy - omega
        rf = vx + vy + omega
        rr = vx - vy + omega
        lr = vx + vy - omega

        # 根據您的原始碼 `rr = y - x` 和 `lr = y + x`，您的後輪接線或定義可能與常見配置不同。
        # 如果您的車向右平移時，是右前和左後輪向前轉，左前和右後輪向後轉，那上述公式就是正確的。
        # 如果不是，可能需要調整 rr 和 lr 的公式。

        # 正規化，確保最大值不超過 100
        max_val = max(abs(lf), abs(rf), abs(rr), abs(lr))
        if max_val > 100.0:
            scale = 100.0 / max_val
            lf *= scale
            rf *= scale
            rr *= scale
            lr *= scale

        # 直接設定最終指令，斜坡加速由遙控器訊號的自然變化來完成
        # 如果需要更柔和的啟動，可以增加 ramp_ms
        self.set_command([int(lf), int(rf), int(rr), int(lr)], ramp_ms=20)

    # ==================== 離散運動指令 (高層級命令) ====================
    def forward(self, ramp_ms=100):
        self.set_command(self.VEC_FORWARD, ramp_ms)

    def backward(self, ramp_ms=100):
        self.set_command(self.VEC_BACKWARD, ramp_ms)

    def left(self, ramp_ms=100):
        self.set_command(self.VEC_LEFT, ramp_ms)

    def right(self, ramp_ms=100):
        self.set_command(self.VEC_RIGHT, ramp_ms)

    def turn_left(self, ramp_ms=100):
        self.set_command(self.VEC_TURN_LEFT, ramp_ms)

    def turn_right(self, ramp_ms=100):
        self.set_command(self.VEC_TURN_RIGHT, ramp_ms)

    def forward_left(self, ramp_ms=100):
        self.set_command(self.VEC_FORWARD_LEFT, ramp_ms)

    def forward_right(self, ramp_ms=100):
        self.set_command(self.VEC_FORWARD_RIGHT, ramp_ms)

    def backward_left(self, ramp_ms=100):
        self.set_command(self.VEC_BACKWARD_LEFT, ramp_ms)

    def backward_right(self, ramp_ms=100):
        self.set_command(self.VEC_BACKWARD_RIGHT, ramp_ms)

    def stop(self, ramp_ms=50):
        self.set_command(self.VEC_STOP, ramp_ms)

    # ==================== 其他工具方法 ====================
    def set_speed(self, percent: str):
        try:
            self.current_scale = max(0.0, min(100.0, float(percent)))
            print(f"Set speed to {self.current_scale}%")
        except ValueError:
            print("Invalid speed value. Please enter a number.")

    def pid_on(self):
        self.enable_pid = True
        [p.reset() for p in self.pids]
        print("PID: ON")

    def pid_off(self):
        self.enable_pid = False
        print("PID: OFF")

    def status(self):
        print("=== System Status ===")
        print(f"PID Enabled: {self.enable_pid}")
        print(f"Current Speed Scale: {self.current_scale}%")
        print("Motor Scale:", [f"{s:.4f}" for s in self.motor_scale])
        print("Filtered RPS:", [f"{r:.2f}" for r in self.rps_filtered])
        print("Base Command:", [f"{c:.1f}" for c in self.base_cmd])

    def deinit(self):
        print("🛑 Shutting down...")
        if hasattr(self, "_timer"):
            self._timer.deinit()
        self.stop(ramp_ms=0)
        for enc in self.encs:
            enc.deinit()
        print("Shutdown complete.")


# ==================== 校準器 ====================
class HybridCalibrator:
    def __init__(self, robot: MecanumRobot):
        self.robot = robot

    def run(self, duration: int = 5, runs: int = 3):
        print("=== Starting Auto Calibration ===")

        auto_scales = []
        for r in range(runs):
            print(f"\n--- Auto Cal Run {r+1}/{runs} ---")
            self.robot.rps_log.clear()
            self.robot.forward(ramp_ms=500)
            utime.sleep(duration)
            self.robot.stop()

            if not self.robot.rps_log:
                print("⚠️ No data, skipping this run")
                continue

            # 每輪平均 RPS
            avg_rps = [
                sum(sample[i] for sample in self.robot.rps_log)
                / len(self.robot.rps_log)
                for i in range(4)
            ]
            target_rps = sum(avg_rps) / 4
            scale = [target_rps / v if v > 1e-6 else 1.0 for v in avg_rps]
            auto_scales.append(scale)
            print(f"Run {r+1} motor_scale:", [f"{val:.4f}" for val in scale])
            utime.sleep(1)

        if not auto_scales:
            print("⚠️ Auto Cal failed, motor_scale unchanged")
            return

        # 取平均
        final_scale = [
            sum(s[i] for s in auto_scales) / len(auto_scales) for i in range(4)
        ]
        self.robot.motor_scale = final_scale

        print(
            "\n✅ Final Auto Cal motor_scale:",
            [f"{val:.4f}" for val in self.robot.motor_scale],
        )


# ==================== 命令列介面 (CLI) ====================
class CLI:
    def __init__(self, robot, calibrator):
        self.robot = robot
        self.calibrator = calibrator
        self.commands = {
            "w": self.robot.forward,
            "s": self.robot.backward,
            "a": self.robot.left,
            "d": self.robot.right,
            "e": self.robot.turn_left,
            "q": self.robot.turn_right,
            "x": self.robot.stop,
            "wa": self.robot.forward_left,
            "wd": self.robot.forward_right,
            "sa": self.robot.backward_left,
            "sd": self.robot.backward_right,
            # "test": self.robot.test_motor,
            # "flip": self.robot.flip_motor,
            "scale": self.robot.set_speed,
            "pid_on": self.robot.pid_on,
            "pid_off": self.robot.pid_off,
            # "set_pid": self.robot.set_pid,
            # "set_movement": self.robot.set_movement,
            "status": self.robot.status,
            "help": self.print_help,
            "exit": self.robot.deinit,
            "auto_cal": self.calibrator.run,
        }

    def print_help(self):
        print("\n=== Robot Commands ===")
        print("w/s/a/d/wa/wd/sa/sd/q/e/x  - Movement control")
        # print("test <N>                   - Test motor (N=1~4)")
        # print("flip <N>                   - Flip motor direction (N=1~4)")
        print("scale <P>                  - Set speed percentage (P=0~100)")
        print("pid_on/pid_off             - Enable/Disable PID")
        print("set_pid [kp=V] [ki=V] [kd=V] - Set PID parameters")
        print("set_movement m1 m2 m3 m4   - Custom wheel speeds")
        print("auto_cal                   - Auto calibrate motor_scale")
        print("status                     - Show system status")
        print("help                       - Show this help")
        print("exit                       - Safe shutdown")
        print("=========================")

    def run(self):
        self.print_help()
        while True:
            try:
                cmd = input("robot> ").strip()
                if not cmd:
                    continue
                parts = cmd.split()
                command, args = parts[0].lower(), parts[1:]
                if command in self.commands:
                    self.commands[command](*args) if args else self.commands[command]()
                    if command == "exit":
                        break
                else:
                    print(f"Unknown command: {command}")
            except Exception as e:
                sys.print_exception(e)


# ==================== 主程式入口 ====================
if __name__ == "__main__":
    try:
        # 1. 初始化
        params = RobotParams()
        robot = MecanumRobot(params)
        calibrator = HybridCalibrator(robot)
        radio = RadioControl(robot)
        led = Pin(25, Pin.OUT)

        # --- 遙控模式 (預設) ---
        while True:
            led.value(1)  # 使用 toggle 讓 LED 閃爍，方便觀察是否正常運行
            radio.update()
            # time.sleep_ms(5)  # 主迴圈稍微延遲，避免 CPU 佔用率 100%
            # cli = CLI(robot, calibrator)
            # cli.run()

    except KeyboardInterrupt:
        print("\nScript interrupted by user.")
    finally:
        if "robot" in locals():
            robot.deinit()
        print("Program terminated.")

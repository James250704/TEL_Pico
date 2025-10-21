# -*- coding: utf-8 -*-
# 檔案名稱: main.py
# 功能: 接收來自遙控 Pico 的 UART 指令，控制麥克納姆輪平台與伺服馬達雲台

import utime
from machine import Pin, PWM, Timer, UART, disable_irq, enable_irq

# ==================== 常數與設定 (Constants & Configuration) ====================


# --- 與遙控 Pico 通訊的 UART 設定 ---
class CommConfig:
    UART_ID = 0
    BAUDRATE = 115200
    TX_PIN_TO_REMOTE = 16  # 這個 TX 接到遙控板的 RX (備用)
    RX_PIN_FROM_REMOTE = 17  # 這個 RX 要接到遙控板的 TX


# --- 機器人硬體設定 ---
class HardwareConfig:
    # (PWM Pin, DIR Pin, Reversed)
    MOTOR_PINS = [
        (2, 3, False),  # 0: 左前輪 (LF)
        (6, 7, True),  # 1: 右前輪 (RF)
        (10, 11, False),  # 2: 右後輪 (RR)
        (4, 5, True),  # 3: 左後輪 (LR)
    ]

    # (Encoder A Pin)
    ENCODER_PINS = [12, 13, 14, 15]

    # 伺服馬達雲台 Pin 腳
    SERVO_PAN_PIN = 21  # 水平
    SERVO_PITCH_PIN = 20  # 俯仰


# --- 機器人參數 ---
class RobotParams:
    CONTROL_DT_MS = 8
    PPR = 16
    PID_KP = 0.25
    PID_KI = 0.0
    PID_KD = 0.0
    PID_OUT_LIM = 50.0
    MAX_DUTY = 40000
    # DEFAULT_MOTOR_SCALE = [1.0105, 0.9965, 0.9931, 1.0001]
    DEFAULT_MOTOR_SCALE = [1.0, 1.0, 1.0, 1.0]


# --- 伺服馬達參數 ---
class ServoParams:
    PWM_FREQ = 50
    MIN_PULSE_US = 500
    MAX_PULSE_US = 2500
    # 角度限制
    PAN_MIN_ANGLE = -20  # 水平最小角度
    PAN_MAX_ANGLE = 20  # 水平最大角度
    PITCH_MIN_ANGLE = -90  # 俯仰最小角度
    PITCH_MAX_ANGLE = 90  # 俯仰最大角度


# ==================== PID 控制器 (與原版相同) ====================
class PID:
    def __init__(self, kp, ki, kd, out_limit, **kwargs):
        self.kp, self.ki, self.kd = kp, ki, kd
        self.out_limit = float(out_limit)
        self.integral_zone = kwargs.get("integral_zone", 50.0)
        self.slew_rate = kwargs.get("slew_rate", 1000.0)
        self.d_filter_alpha = kwargs.get("d_filter_alpha", 0.2)
        self.reset()

    def reset(self):
        self.i = 0.0
        self.prev_e = 0.0
        self.prev_u = 0.0
        self.d_filtered = 0.0

    def update(self, e, dt):
        if dt <= 1e-9:
            return self.prev_u
        if abs(e) < self.integral_zone and abs(self.prev_u) < self.out_limit * 0.95:
            self.i += e * dt
        de = (e - self.prev_e) / dt
        self.d_filtered = (
            1 - self.d_filter_alpha
        ) * self.d_filtered + self.d_filter_alpha * de
        self.prev_e = e
        out = self.kp * e + self.ki * self.i + self.kd * self.d_filtered
        du = out - self.prev_u
        max_du = self.slew_rate * dt
        du = max(-max_du, min(max_du, du))
        out = self.prev_u + du
        out = max(-self.out_limit, min(self.out_limit, out))
        self.prev_u = out
        return out


# ==================== 硬體抽象層 (與原版相同) ====================
class Motor:
    def __init__(self, pwm_pin, dir_pin, reversed=False, max_duty=40000):
        self.pwm = PWM(Pin(pwm_pin))
        self.pwm.freq(1000)
        self.dir = Pin(dir_pin, Pin.OUT)
        self.reversed = reversed
        self.max_duty = max_duty

    def set_speed_percent(self, pct):
        p = max(-100.0, min(100.0, pct))
        if self.reversed:
            p = -p
        self.dir.value(1 if p > 0 else 0)
        duty = int(abs(p) / 100.0 * self.max_duty)
        self.pwm.duty_u16(min(65535, duty))

    def stop(self):
        self.pwm.duty_u16(0)


class SafeEncoder:
    def __init__(self, pin_a):
        self.pulse_count = 0
        self.pin = Pin(pin_a, Pin.IN, Pin.PULL_UP)
        self.pin.irq(trigger=Pin.IRQ_RISING, handler=self._irq_handler)

    def _irq_handler(self, pin):
        self.pulse_count += 1

    def take_delta(self):
        state = disable_irq()
        count = self.pulse_count
        self.pulse_count = 0
        enable_irq(state)
        return count

    def deinit(self):
        self.pin.irq(handler=None)


# ==================== 伺服馬達雲台控制 ====================
class ServoGimbal:
    def __init__(self):
        self.pwm_pan = PWM(Pin(HardwareConfig.SERVO_PAN_PIN))
        self.pwm_pan.freq(ServoParams.PWM_FREQ)

        self.pwm_pitch = PWM(Pin(HardwareConfig.SERVO_PITCH_PIN))
        self.pwm_pitch.freq(ServoParams.PWM_FREQ)

    def _angle_to_duty(self, angle: float) -> int:
        """將角度轉換為 PWM duty"""
        # 將角度映射到脈衝寬度
        pulse_us = ServoParams.MIN_PULSE_US + ((angle + 90) / 180) * (
            ServoParams.MAX_PULSE_US - ServoParams.MIN_PULSE_US
        )
        duty = int((pulse_us / (1_000_000 / ServoParams.PWM_FREQ)) * 65535)
        return duty

    def set_pan(self, pan_percent: float):
        """設定水平伺服馬達位置 (-100 to 100)"""
        # 將百分比轉換為角度
        angle = (pan_percent / 100.0) * ServoParams.PAN_MAX_ANGLE
        # 套用角度限制
        angle = max(ServoParams.PAN_MIN_ANGLE, min(ServoParams.PAN_MAX_ANGLE, angle))
        duty = self._angle_to_duty(angle)
        self.pwm_pan.duty_u16(duty)

    def set_pitch(self, pitch_percent: float):
        """設定俯仰伺服馬達位置 (-100 to 100)"""
        # 將百分比轉換為角度
        angle = (pitch_percent / 100.0) * ServoParams.PITCH_MAX_ANGLE
        # 套用角度限制
        angle = max(
            ServoParams.PITCH_MIN_ANGLE, min(ServoParams.PITCH_MAX_ANGLE, angle)
        )
        duty = self._angle_to_duty(angle)
        self.pwm_pitch.duty_u16(duty)

    def deinit(self):
        self.pwm_pan.deinit()
        self.pwm_pitch.deinit()


# ==================== 麥克納姆輪機器人主類別 (與原版相似) ====================
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
        self.pid_ignore_ms = 0
        self.switch_ms = utime.ticks_ms()
        self._last_us = utime.ticks_us()
        self._timer = Timer()
        self._timer.init(
            period=self.params.CONTROL_DT_MS,
            mode=Timer.PERIODIC,
            callback=self._timer_cb,
        )

    def _timer_cb(self, t: Timer):
        self._scheduled_update(0)

    # _scheduled_update 核心控制迴圈與原版完全相同
    def _scheduled_update(self, _arg):
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

        def mean_norm(idxs):
            if not idxs:
                return 0.0
            return sum([(self.rps_filtered[i] / max(1.0, w[i])) for i in idxs]) / len(
                idxs
            )

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
            err_pct = sgn[i] * (w[i] - (self.rps_filtered[i] / mn))
            elapsed = utime.ticks_diff(utime.ticks_ms(), self.switch_ms)
            use_pid = self.enable_pid and (elapsed >= self.pid_ignore_ms)
            du = self.pids[i].update(err_pct, dt) if use_pid else 0.0
            u = max(-100.0, min(100.0, u_ff + du)) * self.motor_scale[i]
            self.motors[i].set_speed_percent(u)

    def set_command(self, target_cmd: list):
        for p in self.pids:
            p.reset()
        self.base_cmd = list(target_cmd)
        self.switch_ms = utime.ticks_ms()

    def apply_kinematics(self, vx: float, vy: float, omega: float):
        lf = vy + vx + omega
        rf = vy - vx - omega
        rr = vy - vx + omega
        lr = vy + vx - omega
        max_val = max(abs(lf), abs(rf), abs(rr), abs(lr))
        if max_val > 100.0:
            scale = 100.0 / max_val
            lf *= scale
            rf *= scale
            rr *= scale
            lr *= scale
        self.set_command([int(lf), int(rf), int(rr), int(lr)])

    def deinit(self):
        if hasattr(self, "_timer"):
            self._timer.deinit()
        for motor in self.motors:
            motor.stop()
        for enc in self.encs:
            enc.deinit()


# ==================== 主程式入口 (最終版 - 使用緩衝區解決封包切割問題) ====================
if __name__ == "__main__":
    robot = None
    gimbal = None
    try:
        # 1. 初始化硬體
        print("Initializing hardware...")  # 中文解釋: 正在初始化硬體...
        params = RobotParams()
        robot = MecanumRobot(params)
        gimbal = ServoGimbal()
        led = Pin("LED", Pin.OUT)

        # 2. 初始化 UART 接收器
        comm_uart = UART(
            CommConfig.UART_ID,
            baudrate=CommConfig.BAUDRATE,
            tx=Pin(CommConfig.TX_PIN_TO_REMOTE),
            rx=Pin(CommConfig.RX_PIN_FROM_REMOTE),
        )
        print(
            "Main controller ready. Waiting for commands..."
        )  # 中文解釋: 主控制器已就緒，等待指令中...

        # 建立一個空的位元組緩衝區來累積收到的資料
        command_buffer = b""

        # --- 主迴圈: 監聽 UART 指令並執行 ---
        while True:
            # led.toggle() # 在高速迴圈中，LED閃爍會太快看不見，可以先關閉

            # 步驟 1: 盡可能快地將所有UART數據讀入緩衝區
            if comm_uart.any():
                new_data = comm_uart.read()
                if new_data:
                    command_buffer += new_data

            # 步驟 2: 使用 "while" 迴圈，一次性處理完緩衝區中所有完整的指令
            while True:
                newline_pos = command_buffer.find(b"\n")
                if newline_pos == -1:
                    # 如果緩衝區中沒有找到換行符，代表沒有完整指令了，跳出內層迴圈
                    break

                # 提取一條完整指令
                full_command_bytes = command_buffer[: newline_pos + 1]
                # 從緩衝區移除已處理的指令
                command_buffer = command_buffer[newline_pos + 1 :]

                try:
                    command = full_command_bytes.decode("utf-8").strip()
                    if command.startswith("CMD:"):
                        parts = command[4:].split(",")
                        if len(parts) == 5:
                            pan, pitch, omega, vx, vy = [int(p.strip()) for p in parts]
                            # print(f"Executing: vx={vx}, vy={vy}, omega={omega}, pan={pan}, pitch={pitch}") # 除錯時再打開
                            robot.apply_kinematics(vx, vy, omega)
                            gimbal.set_pan(pan)
                            gimbal.set_pitch(pitch)
                except Exception as e:
                    # print(f"Error processing command: {e}") # 除錯時再打開
                    pass

            # 步驟 3: 將 sleep 時間大幅縮短或移除
            utime.sleep_ms(0)  # 可嘗試 1 或 0

    except KeyboardInterrupt:
        print("\nProgram stopped by user.")  # 中文解釋: 使用者已停止程式。
    finally:
        if robot:
            robot.deinit()
            print("Robot deinitialized.")  # 中文解釋: 機器人已取消初始化。
        if gimbal:
            gimbal.deinit()
            print("Gimbal deinitialized.")  # 中文解釋: 雲台已取消初始化。

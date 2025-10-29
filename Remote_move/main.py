# -*- coding: utf-8 -*-
# 檔案名稱: main_mecanum_working_debug.py
# 功能: 接收 10 通道 UART 指令，使用正確的運動學模型和座標轉換
#       並將原始字串透過 pin 8,9 UART 完整傳送出去

import utime
from machine import Pin, PWM, Timer, UART, disable_irq, enable_irq


# ==================== 常數與設定 ====================
class CommConfig:
    UART_ID = 0
    BAUDRATE = 115200
    TX_PIN_TO_REMOTE = 16
    RX_PIN_FROM_REMOTE = 17


class HardwareConfig:
    MOTOR_PINS = [
        (2, 3, False),  # 左前輪 (LF)
        (6, 7, False),  # 右前輪 (RF)
        (10, 11, False),  # 右後輪 (RR)
        (4, 5, False),  # 左後輪 (LR)
    ]
    ENCODER_PINS = [12, 13, 14, 15]


class RobotParams:
    CONTROL_DT_MS = 8
    PPR = 16
    PID_KP = 0.25
    PID_KI = 0.0
    PID_KD = 0.0
    PID_OUT_LIM = 50.0
    MAX_DUTY = 40000
    DEFAULT_MOTOR_SCALE = [1.0, 1.0, 1.0, 1.0]


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


# ==================== 麥克納姆輪機器人主類別 ====================
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

    # _scheduled_update 核心控制迴圈 (與原版相同)
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

    # ====================【修正 1: 運動學公式】====================
    # 套用 File 2 (正確的) 的運動學公式
    def apply_kinematics(self, vx: float, vy: float, omega: float):
        lf = vx + vy + omega
        rf = vx - vy - omega
        rr = vx + vy - omega
        lr = vx - vy + omega  # <--- 已修正為 File 2 的版本

        max_val = max(abs(lf), abs(rf), abs(rr), abs(lr))
        if max_val > 100.0:
            scale = 100.0 / max_val
            lf *= scale
            rf *= scale
            rr *= scale
            lr *= scale
        self.set_command([int(lf), int(rf), int(rr), int(lr)])

    # ==========================================================

    def deinit(self):
        if hasattr(self, "_timer"):
            self._timer.deinit()
        for motor in self.motors:
            motor.stop()
        for enc in self.encs:
            enc.deinit()


# ==================== 主程式 ====================
if __name__ == "__main__":
    robot = None
    try:
        # 初始化硬體
        params = RobotParams()
        robot = MecanumRobot(params)
        led = Pin("LED", Pin.OUT)

        # 接收 UART
        comm_uart = UART(
            CommConfig.UART_ID,
            baudrate=CommConfig.BAUDRATE,
            tx=Pin(CommConfig.TX_PIN_TO_REMOTE),
            rx=Pin(CommConfig.RX_PIN_FROM_REMOTE),
        )

        # 輸出 UART (用於 debug 轉發)
        uart_out = UART(
            1,
            baudrate=CommConfig.BAUDRATE,
            tx=Pin(8),
            rx=Pin(9),
        )

        # ==================== 主迴圈 ====================
        command_buffer = b""
        pid_enabled = False
        while True:
            led.toggle()
            if comm_uart.any():
                new_data = comm_uart.read()
                if new_data:
                    command_buffer += new_data

            while b"\n" in command_buffer:
                newline_pos = command_buffer.find(b"\n")
                line_bytes = command_buffer[:newline_pos]
                command_buffer = command_buffer[newline_pos + 1 :]

                try:
                    line_str = line_bytes.decode("utf-8").strip()
                    parts = line_str.split(",")
                    if len(parts) != 10:
                        continue
                    values = [int(p) for p in parts]

                    # 轉發原始字串 (Debug 功能保留)
                    uart_out.write(line_bytes + b"\n")

                    # PID 開關控制 (第 8 個值，index 7)
                    pid_enabled = True if values[7] else False
                    robot.enable_pid = pid_enabled

                    # ====================【修正 2: 座標系轉換】====================
                    # 遙控器傳來的 10 個值
                    vx_remote = values[0]
                    vy_remote = values[1]
                    omega_remote = values[9]
                    print(f"vx={vx_remote}, vy={vy_remote}, omega={omega_remote}")
                    robot.apply_kinematics(vx_remote, vy_remote, omega_remote)
                    # ============================================================

                except Exception:
                    pass

            utime.sleep_ms(0)

    except KeyboardInterrupt:
        pass
    finally:
        if robot:
            robot.deinit()

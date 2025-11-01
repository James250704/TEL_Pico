# -*- coding: utf-8 -*-
# 檔案名稱: Remote_move/main.py
# 功能: 接收 10 通道 UART 指令，使用 FF+PID 閉迴路控制 (僅底盤)

import utime
from machine import Pin, PWM, Timer, UART, disable_irq, enable_irq

ENABLE_PID = True  # (中文解釋: 啟用我們調校好的 PID 控制)


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

    # --- (來自 PID 調校的黃金參數) ---
    PPR = 432  # (16 * 27)
    KF_GAIN = 21.5  # (前饋增益)
    PID_KP = 1.0  # (P 增益)
    PID_KI = 1.0  # (I 增益)
    PID_KD = 0.0  # (D 增益)
    # -----------------------------------

    # (中文解釋: 設定遙控器 100% 對應的最大 RPS)
    MAX_RPS = 7.0

    PID_OUT_LIM = 30.0  # PID 修正的百分比上限
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

        self.enable_pid = ENABLE_PID
        self.base_cmd = [0.0, 0.0, 0.0, 0.0]  # (中文解釋: 指令是 RPS 浮點數)
        self.motor_scale = list(params.DEFAULT_MOTOR_SCALE)
        self.rps_filtered = [0.0] * 4
        self.RPS_FILTER_ALPHA = 0.2
        self._last_us = utime.ticks_us()
        self._timer = Timer()
        self._timer.init(
            period=self.params.CONTROL_DT_MS,
            mode=Timer.PERIODIC,
            callback=self._timer_cb,
        )

    def _timer_cb(self, t: Timer):
        self._scheduled_update(0)

    # ====================【FF+PID 控制迴圈】====================
    # (中文解釋: 使用 pid_tuner.py 驗證過的 FF+PID 邏輯)
    def _scheduled_update(self, _arg):
        now = utime.ticks_us()
        dt = utime.ticks_diff(now, self._last_us) / 1_000_000.0
        if dt <= 1e-9:
            return
        self._last_us = now

        # 1. 讀取編碼器並計算 RPS
        pulses = [e.take_delta() for e in self.encs]
        # (中文解釋: 你的 SafeEncoder 只用 A 相，無法判斷方向。)
        # (PID 只需處理速度大小，方向由 set_speed_percent 處理)
        raw_rps = [abs(p) / (self.params.PPR * dt) for p in pulses]

        for i in range(4):
            self.rps_filtered[i] = (1 - self.RPS_FILTER_ALPHA) * self.rps_filtered[
                i
            ] + self.RPS_FILTER_ALPHA * raw_rps[i]

        # 2. 套用 FF+PID 控制迴圈 (對4個馬達各做一次)
        for i in range(4):
            target_rps = self.base_cmd[i]
            measured_rps = self.rps_filtered[i]

            # (中文解釋: 誤差只計算速度大小的差異)
            error_rps = abs(target_rps) - measured_rps

            # 3. 計算前饋 FF (使用大小)
            u_ff = abs(target_rps) * self.params.KF_GAIN

            # 4. 計算 PID 修正量
            du = self.pids[i].update(error_rps, dt) if self.enable_pid else 0.0

            # 5. 總和並限制
            total_power_magnitude = u_ff + du

            # (中文解釋: 替 target_rps 加上正負號)
            if target_rps < 0:
                total_power = -total_power_magnitude
            else:
                total_power = total_power_magnitude

            total_power = max(-100.0, min(100.0, total_power))

            # 6. 驅動馬達
            if abs(target_rps) < 0.05:  # (中文解釋: 設置一個死區)
                self.motors[i].stop()
                self.pids[i].reset()
            else:
                self.motors[i].set_speed_percent(total_power * self.motor_scale[i])

    # ==========================================================

    def set_command(self, target_cmd: list):
        # (中文解釋: 只有在目標大幅改變時才重置 PID)
        for i in range(4):
            if abs(target_cmd[i]) < 0.05 and abs(self.base_cmd[i]) > 0.05:
                self.pids[i].reset()

        self.base_cmd = list(target_cmd)

    # ====================【修正後的運動學】====================
    # (中文解釋: 修正運動學公式，並計算目標 RPS)
    def apply_kinematics(self, vx: float, vy: float, omega: float):
        # 假設: vx, vy, omega 範圍是 -100.0 到 100.0

        # 1. 將 -100~100 的指令轉換為 -MAX_RPS ~ +MAX_RPS
        vx_rps = (vx / 100.0) * self.params.MAX_RPS
        vy_rps = (vy / 100.0) * self.params.MAX_RPS
        omega_rps = (omega / 100.0) * self.params.MAX_RPS

        # 2. 標準麥克納姆輪運動學 (LF, RF, RR, LR)
        lf_rps = vx_rps + vy_rps + omega_rps
        rf_rps = vx_rps - vy_rps - omega_rps
        rr_rps = vx_rps + vy_rps - omega_rps
        lr_rps = vx_rps - vy_rps + omega_rps

        # 3. 限制最大速度
        max_val = max(abs(lf_rps), abs(rf_rps), abs(rr_rps), abs(lr_rps))
        if max_val > self.params.MAX_RPS:
            scale = self.params.MAX_RPS / max_val
            lf_rps *= scale
            rf_rps *= scale
            rr_rps *= scale
            lr_rps *= scale

        # 4. 將目標 RPS (浮點數) 設為指令
        self.set_command([lf_rps, rf_rps, rr_rps, lr_rps])

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

        # 接收 UART (來自遙控器)
        comm_uart = UART(
            CommConfig.UART_ID,
            baudrate=CommConfig.BAUDRATE,
            tx=Pin(CommConfig.TX_PIN_TO_REMOTE),
            rx=Pin(CommConfig.RX_PIN_FROM_REMOTE),
        )

        command_buffer = b""

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

                    vx_remote = values[0]
                    vy_remote = values[1]
                    omega_remote = values[9]

                    robot.apply_kinematics(vx_remote, vy_remote, omega_remote)
                    # print(f"vx={vx_remote}, vy={vy_remote}, omega={omega_remote}")

                except (ValueError, IndexError) as e:
                    print(f"Parse Error: {e}, Data: '{line_str}'")

            utime.sleep_ms(0)  # (中文解釋: 讓出 CPU)

    except KeyboardInterrupt:
        print("Program stopped by user.")  # (中文解釋: 程式已被使用者停止)
    finally:
        if robot:
            robot.deinit()
        # print("Hardware deinitialized.")  # (中文解釋: 硬體已解除初始化)

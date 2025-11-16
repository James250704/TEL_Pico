# -*- coding: utf-8 -*-
# 檔案名稱: Remote_move/main.py
# 版本: 2.0 (ISR 安全修正 + UART 腳位修正)
# 功能: 接收 10 通道 UART 指令，使用 FF+PID 閉迴路控制 (僅底盤)

import utime
from machine import Pin, PWM, Timer, UART, disable_irq, enable_irq
import micropython

# 用於在中斷發生錯誤時提供更多資訊 (除錯好習慣)
micropython.alloc_emergency_exception_buf(100)

ENABLE_PID = True


# ==================== 通訊設定 (修正為配合 Remote Receiver) ====================
class CommConfig:
    # 【重要修正】
    # 遙控器接收端 (remote_receiver.py) 是使用 UART 1 (TX=8, RX=9)
    # 所以這裡必須交叉連接：接收端 TX(8) -> 底盤 RX(9)
    UART_ID = 0
    BAUDRATE = 115200
    TX_PIN_TO_REMOTE = 16  # 對應遙控端的 RX
    RX_PIN_FROM_REMOTE = 17  # 對應遙控端的 TX (Pin 8)


class HardwareConfig:
    MOTOR_PINS = [
        (2, 3, False),  # 左前 (LF)
        (6, 7, True),  # 右前 (RF)
        (10, 11, False),  # 右後 (RR)
        (4, 5, True),  # 左後 (LR)
    ]
    ENCODER_PINS = [12, 13, 14, 15]


class RobotParams:
    CONTROL_DT_MS = 8  # PID 控制週期 8ms (約 125Hz)

    # --- PID 調校參數 ---
    PPR = 432
    KF_GAIN = 21.5
    PID_KP = 1.0
    PID_KI = 1.0
    PID_KD = 0.0
    # -------------------

    MAX_RPS = 7.0
    PID_OUT_LIM = 30.0
    MAX_DUTY = 40000
    DEFAULT_MOTOR_SCALE = [1.0, 0.97, 0.97, 1.0]


# ==================== PID 控制器 ====================
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


# ==================== 硬體抽象層 ====================
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

    # 編碼器中斷必須非常快，這裡只做加法是安全的
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
        self.base_cmd = [0.0, 0.0, 0.0, 0.0]
        self.motor_scale = list(params.DEFAULT_MOTOR_SCALE)
        self.rps_filtered = [0.0] * 4
        self.RPS_FILTER_ALPHA = 0.2

        self._last_us = utime.ticks_us()

        # 【修改點 1: 新增 PID 旗標】
        self.pid_trigger = False  # 當它為 True 時，主迴圈會執行計算

        # 設定 Timer
        self._timer = Timer()
        self._timer.init(
            period=self.params.CONTROL_DT_MS,
            mode=Timer.PERIODIC,
            callback=self._timer_cb,
        )

    # 【修改點 2: 中斷回調函式 (ISR)】
    # 這個函式現在極度簡短，只負責設定旗標，非常安全
    def _timer_cb(self, t: Timer):
        self.pid_trigger = True

    # 【修改點 3: 實際的 PID 計算邏輯】
    # 將原本 _scheduled_update 的內容移到這裡，並在主迴圈呼叫
    def run_pid_cycle(self):
        now = utime.ticks_us()
        dt = utime.ticks_diff(now, self._last_us) / 1_000_000.0

        # 簡單的 DT 保護
        if dt <= 0:
            return

        self._last_us = now

        # 1. 讀取編碼器
        pulses = [e.take_delta() for e in self.encs]
        raw_rps = [abs(p) / (self.params.PPR * dt) for p in pulses]

        # 2. 濾波
        for i in range(4):
            self.rps_filtered[i] = (1 - self.RPS_FILTER_ALPHA) * self.rps_filtered[
                i
            ] + self.RPS_FILTER_ALPHA * raw_rps[i]

        # 3. PID 計算與馬達控制
        for i in range(4):
            target_rps = self.base_cmd[i]
            measured_rps = self.rps_filtered[i]
            error_rps = abs(target_rps) - measured_rps

            # FF
            u_ff = abs(target_rps) * self.params.KF_GAIN

            # PID
            du = self.pids[i].update(error_rps, dt) if self.enable_pid else 0.0

            # 加總
            total_power_magnitude = u_ff + du

            # 方向處理
            if target_rps < 0:
                total_power = -total_power_magnitude
            else:
                total_power = total_power_magnitude

            total_power = max(-100.0, min(100.0, total_power))

            # 寫入馬達
            if abs(target_rps) < 0.05:
                self.motors[i].stop()
                self.pids[i].reset()
            else:
                self.motors[i].set_speed_percent(total_power * self.motor_scale[i])

    def set_command(self, target_cmd: list):
        for i in range(4):
            if abs(target_cmd[i]) < 0.05 and abs(self.base_cmd[i]) > 0.05:
                self.pids[i].reset()
        self.base_cmd = list(target_cmd)

    def apply_kinematics(self, vx: float, vy: float, omega: float):
        # 運動學計算 (保持不變)
        vx_rps = (vy / 100.0) * self.params.MAX_RPS
        vy_rps = (vx / 100.0) * self.params.MAX_RPS
        omega_rps = (omega / 100.0) * self.params.MAX_RPS

        lf_rps = vx_rps + vy_rps + omega_rps
        rf_rps = vx_rps - vy_rps - omega_rps
        rr_rps = vx_rps + vy_rps - omega_rps
        lr_rps = vx_rps - vy_rps + omega_rps

        max_val = max(abs(lf_rps), abs(rf_rps), abs(rr_rps), abs(lr_rps))
        if max_val > self.params.MAX_RPS:
            scale = self.params.MAX_RPS / max_val
            lf_rps *= scale
            rf_rps *= scale
            rr_rps *= scale
            lr_rps *= scale

        self.set_command([lf_rps, rf_rps, rr_rps, lr_rps])

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
        print("Initializing Robot System...")
        params = RobotParams()
        robot = MecanumRobot(params)
        led = Pin("LED", Pin.OUT)

        # 初始化 UART (使用修正後的 GP8/GP9)
        comm_uart = UART(
            CommConfig.UART_ID,
            baudrate=CommConfig.BAUDRATE,
            tx=Pin(CommConfig.TX_PIN_TO_REMOTE),
            rx=Pin(CommConfig.RX_PIN_FROM_REMOTE),
        )
        print(
            f"UART Listening on ID={CommConfig.UART_ID}, RX={CommConfig.RX_PIN_FROM_REMOTE}"
        )

        command_buffer = b""

        while True:
            # --- 任務 1: 檢查 PID 旗標 ---
            # 這是「旗標式」處理的核心：在主迴圈中執行重負載工作
            if robot.pid_trigger:
                robot.pid_trigger = False  # 降下旗標
                robot.run_pid_cycle()  # 執行 PID 計算

            # --- 任務 2: 處理 UART ---
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
                    if len(parts) == 10:
                        values = [int(p) for p in parts]
                        vx_remote = values[0]
                        vy_remote = values[1]
                        omega_remote = values[9]  # 假設 CH10 是旋轉

                        # 執行運動學解算
                        robot.apply_kinematics(vx_remote, vy_remote, omega_remote)

                        # 視覺反饋 (可移除)
                        led.toggle()

                except (ValueError, IndexError) as e:
                    # 忽略損毀的資料包
                    pass

            # --- 任務 3: 休息 ---
            # 使用 sleep_ms(1) 而不是 0，大幅降低 CPU 發熱，同時保留對 USB/中斷 的響應
            utime.sleep_ms(1)

    except KeyboardInterrupt:
        print("Program stopped by user.")
    except Exception as e:
        print(f"Critical Error: {e}")
    finally:
        if robot:
            robot.deinit()
        print("Hardware deinitialized.")

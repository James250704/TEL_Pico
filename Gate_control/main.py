import machine
import utime
import _thread  # 載入多核心模組

# -----------------------------------------------------------------
# 1. 參數設定
# -----------------------------------------------------------------
VALID_DISTANCE_MIN_CM = 10.0  # 10cm
VALID_DISTANCE_MAX_CM = 300.0  # 300cm

# (1) 準備階段：確認填滿球的時間
LOADING_CONFIRM_DURATION_S = 3.0

# (2) G1-G4 閥門：開啟前的防誤判時間
SAFETY_DURATION_S = 3.0

# (3) Final 閥門：開啟前的獨立防誤判時間
FINAL_SAFETY_DURATION_S = 3.0

# (4) 馬達與伺服動作參數
MOTOR_RUN_TIME_S = 1.5  # (修改) 馬達轉動時間 (1.5秒)
MOTOR_SPEED_30_PERCENT = 19661  # 30%
FINAL_GATE_HOLD_TIME_S = 5.0  # Final 閥門動作後停留時間

# -----------------------------------------------------------------
# 2. 核心通訊 & 硬體定義
# -----------------------------------------------------------------
command_lock = _thread.allocate_lock()
shared_command = None

# L298N
m1_in1 = machine.Pin(2, machine.Pin.OUT)
m1_in2 = machine.Pin(3, machine.Pin.OUT)
pwm_m1 = machine.PWM(machine.Pin(4))
m2_in3 = machine.Pin(5, machine.Pin.OUT)
m2_in4 = machine.Pin(6, machine.Pin.OUT)
pwm_m2 = machine.PWM(machine.Pin(7))
m3_in1 = machine.Pin(8, machine.Pin.OUT)
m3_in2 = machine.Pin(9, machine.Pin.OUT)
pwm_m3 = machine.PWM(machine.Pin(10))
m4_in3 = machine.Pin(11, machine.Pin.OUT)
m4_in4 = machine.Pin(12, machine.Pin.OUT)
pwm_m4 = machine.PWM(machine.Pin(13))
# Sensor
sensor_trig = machine.Pin(14, machine.Pin.OUT)
sensor_echo = machine.Pin(15, machine.Pin.IN)
# Servo (GP18)
servo_pin = machine.Pin(18)
servo_pwm = machine.PWM(servo_pin)
servo_pwm.freq(50)

# -----------------------------------------------------------------
# 3. 控制函式
# -----------------------------------------------------------------
all_pwms = [pwm_m1, pwm_m2, pwm_m3, pwm_m4]
for pwm in all_pwms:
    pwm.freq(1000)
    pwm.duty_u16(0)


def motor_reverse(in_a, in_b, pwm):
    """逆時針 30%"""
    in_a.low()
    in_b.high()
    pwm.duty_u16(MOTOR_SPEED_30_PERCENT)


def motor_stop(in_a, in_b, pwm):
    """停止"""
    in_a.low()
    in_b.low()
    pwm.duty_u16(0)


motor_M1 = (m1_in1, m1_in2, pwm_m1)
motor_M2 = (m2_in3, m2_in4, pwm_m2)
motor_M3 = (m3_in1, m3_in2, pwm_m3)
motor_M4 = (m4_in3, m4_in4, pwm_m4)

# 伺服馬達設定
SERVO_DEFAULT_POS_NS = 1266000  # 60 度 (常態)
SERVO_FINAL_POS_NS = 600000  # 0 度 (動作)


def set_servo_initial_pos():
    servo_pwm.duty_ns(SERVO_DEFAULT_POS_NS)
    utime.sleep(1)


def activate_final_gate_sequence():
    print("    [Servo] Moving to 0 deg...")
    servo_pwm.duty_ns(SERVO_FINAL_POS_NS)
    utime.sleep(FINAL_GATE_HOLD_TIME_S)  # 停留 5 秒
    print("    [Servo] Returning to 60 deg...")
    servo_pwm.duty_ns(SERVO_DEFAULT_POS_NS)
    utime.sleep(1)


def get_distance():
    sensor_trig.low()
    utime.sleep_us(2)
    sensor_trig.high()
    utime.sleep_us(10)
    sensor_trig.low()
    start_time = utime.ticks_us()
    while sensor_echo.value() == 0:
        if utime.ticks_diff(utime.ticks_us(), start_time) > 5000:
            return -1
    pulse_start_time = utime.ticks_us()
    while sensor_echo.value() == 1:
        if utime.ticks_diff(utime.ticks_us(), pulse_start_time) > 23200:
            return -2
    pulse_end_time = utime.ticks_us()
    return (utime.ticks_diff(pulse_end_time, pulse_start_time) * 0.0343) / 2


# -----------------------------------------------------------------
# 4. 核心 1 (馬達)
# -----------------------------------------------------------------
def motor_control_loop():
    global shared_command
    print("[Sys] Motor Thread Ready.")
    try:
        while True:
            cmd = None
            with command_lock:
                if shared_command:
                    cmd = shared_command
                    shared_command = None

            if cmd:
                motor = None
                # 順序: G1->M4, G2->M3, G3->M2, G4->M1
                if cmd == "G1":
                    motor = motor_M4
                elif cmd == "G2":
                    motor = motor_M3
                elif cmd == "G3":
                    motor = motor_M2
                elif cmd == "G4":
                    motor = motor_M1

                if motor:
                    # 執行逆時針 1.5 秒
                    print(f"  >> Motor {cmd}: Running CCW for {MOTOR_RUN_TIME_S}s...")
                    motor_reverse(motor[0], motor[1], motor[2])
                    utime.sleep(MOTOR_RUN_TIME_S)

                    motor_stop(motor[0], motor[1], motor[2])
                    print(f"  >> Motor {cmd}: DONE.")
            utime.sleep_ms(100)
    except Exception:
        pass
    finally:
        motor_stop(motor_M1[0], motor_M1[1], motor_M1[2])
        motor_stop(motor_M2[0], motor_M2[1], motor_M2[2])
        motor_stop(motor_M3[0], motor_M3[1], motor_M3[2])
        motor_stop(motor_M4[0], motor_M4[1], motor_M4[2])


# -----------------------------------------------------------------
# 5. 核心 0 (主程式)
# -----------------------------------------------------------------
_thread.start_new_thread(motor_control_loop, ())
utime.sleep(1)

set_servo_initial_pos()  # 伺服馬達轉到 60 度

print("\n=== Pipe System V9 (Motor 1.5s) ===")
print(f"Empty: {VALID_DISTANCE_MIN_CM}-{VALID_DISTANCE_MAX_CM}cm")
print(
    f"Wait Times: Load={LOADING_CONFIRM_DURATION_S}s, Game={SAFETY_DURATION_S}s, Final={FINAL_SAFETY_DURATION_S}s"
)
print("Waiting for balls to be loaded (Must hold FULL for 3s)...")

current_stage = -1
packet_names = ["G1", "G2", "G3", "G4"]
is_timer_running = False
empty_start_time = 0
last_timer_print_time = 0

try:
    while True:
        dist = get_distance()
        is_empty = dist >= VALID_DISTANCE_MIN_CM and dist <= VALID_DISTANCE_MAX_CM
        is_full = not is_empty

        # --- 階段 -1: 準備模式 ---
        if current_stage == -1:
            if is_full:
                if not is_timer_running:
                    is_timer_running = True
                    empty_start_time = utime.ticks_ms()
                    last_timer_print_time = 0
                else:
                    elapsed = (
                        utime.ticks_diff(utime.ticks_ms(), empty_start_time) / 1000.0
                    )
                    if utime.ticks_diff(utime.ticks_ms(), last_timer_print_time) > 1000:
                        print(
                            f"  [Loading] Confirming FULL... {elapsed:.1f}s / {LOADING_CONFIRM_DURATION_S}s"
                        )
                        last_timer_print_time = utime.ticks_ms()

                    if elapsed >= LOADING_CONFIRM_DURATION_S:
                        print("\n[Sys] Loading Confirmed! System ARMED.")
                        current_stage = 0
                        is_timer_running = False
            else:
                is_timer_running = False

        # --- 階段 8: 最終閥門 ---
        elif current_stage == 8:
            if is_empty:
                if not is_timer_running:
                    is_timer_running = True
                    empty_start_time = utime.ticks_ms()
                    last_timer_print_time = 0
                else:
                    elapsed = (
                        utime.ticks_diff(utime.ticks_ms(), empty_start_time) / 1000.0
                    )
                    if utime.ticks_diff(utime.ticks_ms(), last_timer_print_time) > 1000:
                        print(
                            f"  [Final] Empty detected... {elapsed:.1f}s / {FINAL_SAFETY_DURATION_S}s"
                        )
                        last_timer_print_time = utime.ticks_ms()

                    if elapsed >= FINAL_SAFETY_DURATION_S:
                        print("[Sys] ACTIVATING FINAL GATE SEQUENCE.")
                        activate_final_gate_sequence()
                        current_stage = 9
                        is_timer_running = False
                        print("[Sys] Mission Complete. Gate back at 60 deg.")
            else:
                is_timer_running = False

        elif current_stage == 9:
            pass

        # --- G1-G4 循環 ---
        else:
            wait_empty = current_stage % 2 == 0
            pkt = packet_names[current_stage // 2]

            if wait_empty:  # 等待淨空
                if is_empty:
                    if not is_timer_running:
                        is_timer_running = True
                        empty_start_time = utime.ticks_ms()
                        last_timer_print_time = 0
                    else:
                        elapsed = (
                            utime.ticks_diff(utime.ticks_ms(), empty_start_time)
                            / 1000.0
                        )
                        if (
                            utime.ticks_diff(utime.ticks_ms(), last_timer_print_time)
                            > 1000
                        ):
                            print(
                                f"  [{pkt}] Empty detected... {elapsed:.1f}s / {SAFETY_DURATION_S}s"
                            )
                            last_timer_print_time = utime.ticks_ms()

                        if elapsed >= SAFETY_DURATION_S:
                            print(f"[{pkt}] TRIGGERED! Sending command.")
                            with command_lock:
                                shared_command = pkt
                            current_stage += 1
                            is_timer_running = False
                else:
                    is_timer_running = False

            else:  # 等待填滿
                if is_full:
                    print(f"[{pkt}] Pipe Refilled. Waiting for next cycle...")
                    current_stage += 1

        utime.sleep_ms(100)

except KeyboardInterrupt:
    print("\nStopped.")
finally:
    print("HALTED.")

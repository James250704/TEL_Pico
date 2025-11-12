import machine
import utime
import _thread

VALID_DISTANCE_MIN_CM = 18.0
VALID_DISTANCE_MAX_CM = 50.0

SAFETY_DURATION_S = 5.0
MOTOR_RUN_TIME_S = 5.0
MOTOR_SPEED_30_PERCENT = 19661

command_lock = _thread.allocate_lock()
shared_command = None

# Hardware pin definitions
m1_in1 = machine.Pin(2, machine.Pin.OUT); 
m1_in2 = machine.Pin(3, machine.Pin.OUT); 
pwm_m1 = machine.PWM(machine.Pin(4))
m2_in3 = machine.Pin(5, machine.Pin.OUT); 
m2_in4 = machine.Pin(6, machine.Pin.OUT); 
pwm_m2 = machine.PWM(machine.Pin(7))
m3_in1 = machine.Pin(8, machine.Pin.OUT); 
m3_in2 = machine.Pin(9, machine.Pin.OUT); 
pwm_m3 = machine.PWM(machine.Pin(10))
m4_in3 = machine.Pin(11, machine.Pin.OUT); 
m4_in4 = machine.Pin(12, machine.Pin.OUT); 
pwm_m4 = machine.PWM(machine.Pin(13))

sensor_trig = machine.Pin(14, machine.Pin.OUT)
sensor_echo = machine.Pin(15, machine.Pin.IN)

servo_pin = machine.Pin(18)
servo_pwm = machine.PWM(servo_pin)
servo_pwm.freq(50)

# Motor helpers
all_pwms = [pwm_m1, pwm_m2, pwm_m3, pwm_m4]
for pwm in all_pwms:
    pwm.freq(1000); pwm.duty_u16(0)

def motor_reverse(in_a, in_b, pwm):
    in_a.low(); in_b.high()
    pwm.duty_u16(MOTOR_SPEED_30_PERCENT)

def motor_stop(in_a, in_b, pwm):
    in_a.low(); in_b.low()
    pwm.duty_u16(0)

motor_M1 = (m1_in1, m1_in2, pwm_m1)
motor_M2 = (m2_in3, m2_in4, pwm_m2)
motor_M3 = (m3_in1, m3_in2, pwm_m3)
motor_M4 = (m4_in3, m4_in4, pwm_m4)

# SG90 servo positions (ns)
SERVO_START_POS_NS = 1600000
SERVO_END_POS_NS   = 600000

def set_servo_initial_pos():
    """Set the final gate to initial 90 degree position."""
    print(f"[Core 0] Setting Final Gate to initial 90 DEG position ({SERVO_START_POS_NS} ns)...")
    servo_pwm.duty_ns(SERVO_START_POS_NS)
    utime.sleep(1)

def activate_final_gate():
    """Move servo to 0 degree to open the final gate."""
    print(f"[Core 0] Moving Final Gate to OPEN position (0 deg) ({SERVO_END_POS_NS} ns)...")
    servo_pwm.duty_ns(SERVO_END_POS_NS)
    utime.sleep(1)

def motor_control_loop():
    global shared_command
    print("[Core 1] Motor Control Thread... STARTED.")
    try:
        while True:
            command_to_execute = None
            with command_lock:
                if shared_command is not None:
                    command_to_execute = shared_command; shared_command = None

            if command_to_execute:
                print(f"[Core 1] Action: Received command '{command_to_execute}'")
                motor_to_run = None

                if command_to_execute == "G1": motor_to_run = motor_M4
                elif command_to_execute == "G2": motor_to_run = motor_M3
                elif command_to_execute == "G3": motor_to_run = motor_M2
                elif command_to_execute == "G4": motor_to_run = motor_M1

                if motor_to_run:
                    print(f"  -> Running {command_to_execute}'s motor (Reverse, {MOTOR_RUN_TIME_S}s, 30%)")
                    motor_reverse(motor_to_run[0], motor_to_run[1], motor_to_run[2])
                    utime.sleep(MOTOR_RUN_TIME_S)
                    motor_stop(motor_to_run[0], motor_to_run[1], motor_to_run[2])
                    print(f"[Core 1] Action: Motor run complete for '{command_to_execute}'.")
                else:
                    print(f"[Core 1] Error: Unknown command '{command_to_execute}'.")
                print("[Core 1] Waiting for next command...")
            utime.sleep_ms(100)
    except Exception as e:
        print(f"[Core 1] FATAL ERROR: {e}")
    finally:
        print("[Core 1] HALTING. Stopping all motors.")
        motor_stop(motor_M1[0], motor_M1[1], motor_M1[2]); motor_stop(motor_M2[0], motor_M2[1], motor_M2[2])
        motor_stop(motor_M3[0], motor_M3[1], motor_M3[2]); motor_stop(motor_M4[0], motor_M4[1], motor_M4[2])

def get_distance():
    sensor_trig.low(); utime.sleep_us(2); sensor_trig.high(); utime.sleep_us(10); sensor_trig.low()
    start_time = utime.ticks_us()
    while sensor_echo.value() == 0:
        if utime.ticks_diff(utime.ticks_us(), start_time) > 5000: return -1
    pulse_start_time = utime.ticks_us()
    while sensor_echo.value() == 1:
        if utime.ticks_diff(utime.ticks_us(), pulse_start_time) > 23200: return -2
    pulse_end_time = utime.ticks_us()
    pulse_duration = utime.ticks_diff(pulse_end_time, pulse_start_time)
    distance_cm = (pulse_duration * 0.0343) / 2
    return distance_cm

_thread.start_new_thread(motor_control_loop, ())
print("[Core 0] Starting Core 1 (Motor Control Thread)...")
utime.sleep(1)

set_servo_initial_pos()

print("=" * 40)
print("[Core 0] Pipe Empty Detection System... STARTED.")
print(f"Target Range (Pipe Empty): {VALID_DISTANCE_MIN_CM} - {VALID_DISTANCE_MAX_CM} cm")
print(f"Safety Wait Time: {SAFETY_DURATION_S} seconds")
print("=" * 40)

current_stage = 0
packet_names = ["G1", "G2", "G3", "G4"]
is_timer_running = False
empty_start_time = 0
last_printed_state = 0
last_timer_print_time = 0

try:
    while True:
        distance = get_distance()

        is_pipe_empty = (distance >= VALID_DISTANCE_MIN_CM and 
                         distance <= VALID_DISTANCE_MAX_CM)
        is_pipe_full = not is_pipe_empty

        if current_stage == 8:
            packet_name = "FinalGate"
            if is_pipe_empty:
                if not is_timer_running:
                    is_timer_running = True; empty_start_time = utime.ticks_ms(); last_timer_print_time = utime.ticks_ms()
                    print(f"[{packet_name}] Event: Final Empty detected. Starting {SAFETY_DURATION_S}s timer. (Dist: {distance:.2f} cm)")
                    last_printed_state = 8
                else:
                    elapsed_seconds = utime.ticks_diff(utime.ticks_ms(), empty_start_time) / 1000.0
                    if utime.ticks_diff(utime.ticks_ms(), last_timer_print_time) > 1000:
                        print(f"  [{packet_name}] Timer: {elapsed_seconds:.1f}s / {SAFETY_DURATION_S}s... (Dist: {distance:.2f} cm)")
                        last_timer_print_time = utime.ticks_ms()

                    if elapsed_seconds >= SAFETY_DURATION_S:
                        print(f"[{packet_name}] Action: *** OPENING FINAL GATE (Moving to 0 deg) ***")
                        activate_final_gate()
                        current_stage = 9
                        is_timer_running = False
                        print(f"[{packet_name}] Status: Final Gate Opened. System complete.")
                        last_printed_state = 9
            else:
                if is_timer_running:
                    is_timer_running = False
                    print(f"[{packet_name}] Event: Timer reset (Pipe refilled early). (Dist: {distance:.2f} cm)")
                if last_printed_state != 7:
                    print(f"[{packet_name}] Status: Monitoring (Pipe is Full). Waiting for FINAL empty... (Dist: {distance:.2f} cm)")
                    last_printed_state = 7

        elif current_stage == 9:
            if last_printed_state != 9:
                print("All stages (G1-G4) and Final Gate are complete. Halting sensor loop.")
                last_printed_state = 9
            utime.sleep(1)

        else:
            is_waiting_for_empty_state = (current_stage % 2 == 0)
            packet_index = current_stage // 2
            current_packet_name = packet_names[packet_index]

            if is_waiting_for_empty_state:
                if is_pipe_empty:
                    if not is_timer_running:
                        is_timer_running = True; empty_start_time = utime.ticks_ms(); last_timer_print_time = utime.ticks_ms()
                        print(f"[{current_packet_name}] Event: Pipe Empty detected. Starting {SAFETY_DURATION_S}s timer. (Dist: {distance:.2f} cm)")
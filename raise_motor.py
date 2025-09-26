# tmc2209_continuous_forward.py
# MicroPython on Raspberry Pi Pico
# Continuous forward rotation for TMC2209 driver (step/dir)
# Default pins: DIR=GP22, STEP=GP23, EN=GP24
# 修改最上方常數以配合你的接線

from machine import Pin
import time

# ------------------ 使用者設定（若實際接線不同請改這裡） ------------------
DIR_PIN = 14  # GP22
STEP_PIN = 13  # GP23
EN_PIN = 15  # GP24 (EN/EM, 常見 active LOW => 0 = enabled)

STEPS_PER_REV = 200.0  # 馬達整步數 (1.8° -> 200)
MICROSTEP_FACTOR = 8  # 驅動器 microstep 設定 (1,2,4,8) <-- 依 MS1/MS2 設定
DESIRED_RPM = 60.0  # 要持續正轉的轉速 (RPM)，改此值即可
MIN_PULSE_US = 2  # STEP 脈衝最小高電位寬度 (us)
MAX_SPS = 20000  # safety clamp: 最大步/秒（避免太小週期導致問題）
# -----------------------------------------------------------------------

# 初始化腳位
dir_pin = Pin(DIR_PIN, Pin.OUT, value=1)  # set forward by default
step_pin = Pin(STEP_PIN, Pin.OUT, value=0)
en_pin = Pin(
    EN_PIN, Pin.OUT, value=1
)  # start disabled (1=disabled for active-low boards)


def enable_driver(active_low=True):
    if active_low:
        en_pin.value(0)
    else:
        en_pin.value(1)
    time.sleep_ms(5)


def disable_driver(active_low=True):
    if active_low:
        en_pin.value(1)
    else:
        en_pin.value(0)
    time.sleep_ms(5)


def rpm_to_sps(rpm, microstep):
    """Convert RPM to steps per second (microsteps considered)."""
    return (microstep * STEPS_PER_REV * rpm) / 60.0


def continuous_forward(rpm):
    """持續正轉，blocking loop。Ctrl+C 中斷會回到 main 並關閉驅動器。"""
    # 計算 steps/sec 並做安全限制
    sps = rpm_to_sps(rpm, MICROSTEP_FACTOR)
    if sps <= 0:
        print("Invalid sps (<=0). Stop.")
        return
    if sps > MAX_SPS:
        print(
            "Requested speed too high, clamping. requested sps:",
            sps,
            "clamped to",
            MAX_SPS,
        )
        sps = MAX_SPS

    period_us = int(1_000_000.0 / sps)  # 一個完整 step 週期 (us)
    if period_us <= 2 * MIN_PULSE_US:
        # period 不足以放兩個 min pulse，必須降低速率
        min_period = 2 * MIN_PULSE_US + 1
        sps = int(1_000_000.0 / min_period)
        period_us = min_period
        print("Adjusted sps to meet min pulse width:", sps, "period_us:", period_us)

    high_us = MIN_PULSE_US
    low_us = period_us - high_us
    if low_us < MIN_PULSE_US:
        low_us = MIN_PULSE_US

    print(
        "Continuous forward: RPM =",
        rpm,
        ", sps =",
        sps,
        ", period_us =",
        period_us,
        ", high_us =",
        high_us,
        ", low_us =",
        low_us,
    )

    # 設方向為正向（依你硬體 DIR 高/低定義，若反向請 swap）
    dir_pin.value(1)

    # 啟用驅動器（大多數板是 active LOW）
    enable_driver(active_low=True)

    try:
        # 持續送出 STEP 脈衝
        while True:
            step_pin.value(1)
            time.sleep_us(high_us)
            step_pin.value(0)
            time.sleep_us(low_us)
    except KeyboardInterrupt:
        # 使用者中斷 (Ctrl+C)
        print("\nInterrupted by user, stopping.")
    finally:
        # 停止並 disable 驅動器（若你想保持驅動器 enable 可移除下一行）
        disable_driver(active_low=True)
        print("Driver disabled, exiting.")


if __name__ == "__main__":
    print("Starting continuous forward demo.")
    print("Pins: DIR={}, STEP={}, EN={}".format(DIR_PIN, STEP_PIN, EN_PIN))
    print("Microstep factor:", MICROSTEP_FACTOR, "Desired RPM:", DESIRED_RPM)
    # 直接開始持續正轉；如需變速，可把 DESIRED_RPM 改為其他值或把此呼叫包在迴圈中
    continuous_forward(DESIRED_RPM)

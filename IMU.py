#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Lettura completa BNO085 su Raspberry Pi + calcolo forze G.
- Legge: Accel, Linear Accel, Gyro, Mag, Gravity, Quaternion
- Converte quaternion -> Euler (yaw/pitch/roll) in gradi
- Calcola forze G: Gx, Gy, Gz, |G| da Linear Accel (m/s^2)
- Stampa a ~50 Hz in formato compatto
"""

import math
import time
import sys

import board
import busio

from adafruit_bno08x import BNO08X_I2C
from adafruit_bno08x.i2c import (
    BNO_REPORT_ACCELEROMETER,
    BNO_REPORT_LINEAR_ACCELERATION,
    BNO_REPORT_GYROSCOPE,
    BNO_REPORT_MAGNETOMETER,
    BNO_REPORT_GRAVITY,
    BNO_REPORT_ROTATION_VECTOR,
    BNO_REPORT_GAME_ROTATION_VECTOR,
)

G0 = 9.80665  # m/s^2

def quat_to_euler_deg(qi, qj, qk, qr):
    """
    Converte un quaternione unitario (x=qi, y=qj, z=qk, w=qr)
    in angoli ZYX (yaw, pitch, roll) in gradi.
    """
    # yaw (Z)
    siny_cosp = 2.0 * (qr * qk + qi * qj)
    cosy_cosp = 1.0 - 2.0 * (qj * qj + qk * qk)
    yaw = math.degrees(math.atan2(siny_cosp, cosy_cosp))

    # pitch (Y)
    sinp = 2.0 * (qr * qj - qk * qi)
    if abs(sinp) >= 1:
        pitch = math.degrees(math.copysign(math.pi / 2, sinp))  # 90° clamp
    else:
        pitch = math.degrees(math.asin(sinp))

    # roll (X)
    sinr_cosp = 2.0 * (qr * qi + qj * qk)
    cosr_cosp = 1.0 - 2.0 * (qi * qi + qj * qj)
    roll = math.degrees(math.atan2(sinr_cosp, cosr_cosp))

    return yaw, pitch, roll


def fmt(v, wid=7, prec=3):
    return f"{v:{wid}.{prec}f}"


def main():
    # ---- Init I2C + sensore
    i2c = busio.I2C(board.SCL, board.SDA)
    sensor = BNO08X_I2C(i2c)  # indirizzo default 0x4B (o 0x4A se ADR)

    # ---- Abilita report (100 Hz = 10_000 us; qui 50 Hz ≈ 20_000 us)
    us_50hz = 20_000
    for feat in (
        BNO_REPORT_ACCELEROMETER,
        BNO_REPORT_LINEAR_ACCELERATION,
        BNO_REPORT_GYROSCOPE,
        BNO_REPORT_MAGNETOMETER,
        BNO_REPORT_GRAVITY,
        BNO_REPORT_ROTATION_VECTOR,
        BNO_REPORT_GAME_ROTATION_VECTOR,
    ):
        try:
            sensor.enable_feature(feat, report_interval_us=us_50hz)
        except Exception as e:
            print(f"Impossibile abilitare feature {feat}: {e}", file=sys.stderr)

    print("BNO085 avviato. Premere CTRL+C per uscire.")
    print("-" * 120)
    header = (
        "ACC[m/s^2] ax ay az | LINACC[m/s^2] lax lay laz | G[m/s^2] gx gy gz | "
        "GYRO[°/s] wx wy wz | MAG[µT] mx my mz | EULER[°] yaw pitch roll | "
        "G-FORCES Gx Gy Gz |G|"
    )
    print(header)
    print("-" * 120)

    # ---- loop di lettura
    target_hz = 50.0
    dt = 1.0 / target_hz
    next_t = time.monotonic()

    try:
        while True:
            # --- Letture (ognuna può essere None finché non c'è un nuovo campione)
            ax, ay, az = sensor.acceleration or (None, None, None)            # m/s^2 incl. gravità
            lax, lay, laz = sensor.linear_acceleration or (None, None, None)  # m/s^2 senza gravità
            gx, gy, gz = sensor.gravity or (None, None, None)                 # m/s^2 gravità stimata
            wx, wy, wz = sensor.gyro or (None, None, None)                    # °/s
            mx, my, mz = sensor.magnetic or (None, None, None)                # microTesla
            qi, qj, qk, qr = sensor.quaternion or (None, None, None, None)    # unit quaternions

            # --- Euler (se quaternion disponibile)
            if None not in (qi, qj, qk, qr):
                yaw, pitch, roll = quat_to_euler_deg(qi, qj, qk, qr)
            else:
                yaw = pitch = roll = None

            # --- Forze G dalle accelerazioni lineari (senza gravità)
            if None not in (lax, lay, laz):
                Gx, Gy, Gz = (lax / G0, lay / G0, laz / G0)
                Gtot = math.sqrt(Gx * Gx + Gy * Gy + Gz * Gz)
            else:
                Gx = Gy = Gz = Gtot = None

            # --- Stampa riga compatta
            def safe(v):  # formatta numeri o mostra "--"
                return fmt(v) if v is not None else "   --  "

            line = (
                f"{safe(ax)} {safe(ay)} {safe(az)} | "
                f"{safe(lax)} {safe(lay)} {safe(laz)} | "
                f"{safe(gx)} {safe(gy)} {safe(gz)} | "
                f"{safe(wx)} {safe(wy)} {safe(wz)} | "
                f"{safe(mx)} {safe(my)} {safe(mz)} | "
                f"{safe(yaw)} {safe(pitch)} {safe(roll)} | "
                f"{safe(Gx)} {safe(Gy)} {safe(Gz)} |{safe(Gtot)}"
            )
            print(line)

            # pacing
            next_t += dt
            sleep_t = next_t - time.monotonic()
            if sleep_t > 0:
                time.sleep(sleep_t)
            else:
                next_t = time.monotonic()

    except KeyboardInterrupt:
        print("\nUscita. Bye!")


if __name__ == "__main__":
    main()

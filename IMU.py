#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import math
import time
import sys

import board
import busio

from adafruit_bno08x.i2c import BNO08X_I2C
from adafruit_bno08x import (
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
    siny_cosp = 2.0 * (qr * qk + qi * qj)
    cosy_cosp = 1.0 - 2.0 * (qj * qj + qk * qk)
    yaw = math.degrees(math.atan2(siny_cosp, cosy_cosp))
    sinp = 2.0 * (qr * qj - qk * qi)
    if abs(sinp) >= 1:
        pitch = math.degrees(math.copysign(math.pi / 2, sinp))
    else:
        pitch = math.degrees(math.asin(sinp))
    sinr_cosp = 2.0 * (qr * qi + qj * qk)
    cosr_cosp = 1.0 - 2.0 * (qi * qi + qj * qj)
    roll = math.degrees(math.atan2(sinr_cosp, cosr_cosp))
    return yaw, pitch, roll

def fmt(v, wid=7, prec=3):
    return f"{v:{wid}.{prec}f}"

def _read_safe(getter, count):
    try:
        vals = getter
        if vals is None:
            return (None,) * count
        # alcune versioni sollevano RuntimeError "No ... report found"
        return vals
    except RuntimeError:
        return (None,) * count

def main():
    # ---- Init I2C + sensore (default addr 0x4A; usa address=0x4B se ADR alto)
    i2c = busio.I2C(board.SCL, board.SDA)
    sensor = BNO08X_I2C(i2c)

    # ---- Abilita report con compatibilità versione
    us_50hz = 20_000  # ~50 Hz
    features = (
        BNO_REPORT_ACCELEROMETER,
        BNO_REPORT_LINEAR_ACCELERATION,
        BNO_REPORT_GYROSCOPE,
        BNO_REPORT_MAGNETOMETER,
        BNO_REPORT_GRAVITY,
        BNO_REPORT_ROTATION_VECTOR,
        BNO_REPORT_GAME_ROTATION_VECTOR,
    )

    for feat in features:
        ok = False
        try:
            # vecchie versioni: parametro posizionale
            sensor.enable_feature(feat, us_50hz)
            ok = True
        except TypeError:
            # versioni che non accettano l'intervallo
            try:
                sensor.enable_feature(feat)
                ok = True
            except Exception as e:
                print(f"Impossibile abilitare feature {feat}: {e}", file=sys.stderr)
        except Exception as e:
            print(f"Impossibile abilitare feature {feat}: {e}", file=sys.stderr)

    # piccolo warm-up per permettere la prima pubblicazione dei report
    time.sleep(0.2)

    print("BNO085 avviato. Premere CTRL+C per uscire.")
    print("-" * 120)
    header = (
        "ACC[m/s^2] ax ay az | LINACC[m/s^2] lax lay laz | G[m/s^2] gx gy gz | "
        "GYRO[°/s] wx wy wz | MAG[µT] mx my mz | EULER[°] yaw pitch roll | "
        "G-FORCES Gx Gy Gz |G|"
    )
    print(header)
    print("-" * 120)

    target_hz = 50.0
    dt = 1.0 / target_hz
    next_t = time.monotonic()

    try:
        while True:
            ax, ay, az = _read_safe(sensor.acceleration, 3)            # m/s^2 incl. gravità
            lax, lay, laz = _read_safe(sensor.linear_acceleration, 3)  # m/s^2 senza gravità
            gx, gy, gz = _read_safe(sensor.gravity, 3)                 # m/s^2 gravità stimata
            wx, wy, wz = _read_safe(sensor.gyro, 3)                    # °/s
            mx, my, mz = _read_safe(sensor.magnetic, 3)                # µT
            qi, qj, qk, qr = _read_safe(sensor.quaternion, 4)          # unit quaternions

            if None not in (qi, qj, qk, qr):
                yaw, pitch, roll = quat_to_euler_deg(qi, qj, qk, qr)
            else:
                yaw = pitch = roll = None

            if None not in (lax, lay, laz):
                Gx, Gy, Gz = (lax / G0, lay / G0, laz / G0)
                Gtot = math.sqrt(Gx * Gx + Gy * Gy + Gz * Gz)
            else:
                Gx = Gy = Gz = Gtot = None

            def safe(v):
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
            print(line, flush=True)

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

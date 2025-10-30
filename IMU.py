#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import math, time, sys
import board, busio

from adafruit_bno08x.i2c import BNO08X_I2C
from adafruit_bno08x import (
    BNO_REPORT_ACCELEROMETER,
    BNO_REPORT_LINEAR_ACCELERATION,
    BNO_REPORT_GYROSCOPE,
    BNO_REPORT_MAGNETOMETER,
    BNO_REPORT_GRAVITY,
    BNO_REPORT_ROTATION_VECTOR,
)

G0 = 9.80665  # m/s^2

# ---------- Utils ----------
def fmt(v, w=7, p=3): return f"{v:{w}.{p}f}" if v is not None else "   --  "

def quat_to_euler_deg(qi, qj, qk, qr):
    # yaw(Z), pitch(Y), roll(X) in gradi
    siny_cosp = 2.0 * (qr * qk + qi * qj)
    cosy_cosp = 1.0 - 2.0 * (qj * qj + qk * qk)
    yaw = math.degrees(math.atan2(siny_cosp, cosy_cosp))
    sinp = 2.0 * (qr * qj - qk * qi)
    pitch = math.degrees(math.copysign(math.pi/2, sinp)) if abs(sinp) >= 1 else math.degrees(math.asin(sinp))
    sinr_cosp = 2.0 * (qr * qi + qj * qk)
    cosr_cosp = 1.0 - 2.0 * (qi * qi + qj * qj)
    roll = math.degrees(math.atan2(sinr_cosp, cosr_cosp))
    return yaw, pitch, roll

def rad(d): return d * math.pi / 180.0

def rot_rx(v, roll_deg):
    # ruota v attorno a X di -roll (per "levellare")
    r = rad(roll_deg)
    cr, sr = math.cos(-r), math.sin(-r)
    x, y, z = v
    return (x, cr*y - sr*z, sr*y + cr*z)

def rot_ry(v, pitch_deg):
    # ruota v attorno a Y di -pitch (per "levellare")
    p = rad(pitch_deg)
    cp, sp = math.cos(-p), math.sin(-p)
    x, y, z = v
    return (cp*x + sp*z, y, -sp*x + cp*z)

def rotate_xy(vxy, yaw_deg):
    # ruota nel piano XY di un angolo (per compensare offset di montaggio)
    y = rad(yaw_deg)
    c, s = math.cos(y), math.sin(y)
    x, yv = vxy
    return (c*x - s*yv, s*x + c*yv)

def enable_compat(bno, feature, report_us=20_000):
    # tenta con parametro posizionale, poi senza
    try:
        bno.enable_feature(feature, report_us)
    except TypeError:
        bno.enable_feature(feature)

def read_safe(callable_obj, n):
    try:
        vals = callable_obj
        if vals is None: return (None,) * n
        return vals
    except RuntimeError:
        return (None,) * n

# ---------- Main ----------
def main():
    # I2C init (addr default 0x4A; se ADR alto: BNO08X_I2C(i2c, address=0x4B))
    i2c = busio.I2C(board.SCL, board.SDA)
    bno = BNO08X_I2C(i2c)

    # Abilita report in modo conservativo: 50 Hz accel, 50 Hz linacc, 50 Hz gyro, 25 Hz mag, 50 Hz gravity, 50 Hz rotation
    try_rates = {
        BNO_REPORT_ACCELEROMETER:       20_000,  # ~50 Hz
        BNO_REPORT_LINEAR_ACCELERATION: 20_000,
        BNO_REPORT_GYROSCOPE:           20_000,
        BNO_REPORT_MAGNETOMETER:       40_000,  # ~25 Hz
        BNO_REPORT_GRAVITY:             20_000,
        BNO_REPORT_ROTATION_VECTOR:     20_000,
    }
    for feat, us in try_rates.items():
        try:
            enable_compat(bno, feat, us)
        except Exception as e:
            print(f"Impossibile abilitare feature {feat}: {e}", file=sys.stderr)

    time.sleep(0.4)  # warm-up

    # ---------- Calibrazione offset di montaggio (orizzontale)
    # Media yaw per ~2s (kart fermo, diritto). Useremo questo yaw come "asse avanti" del veicolo.
    yaw_acc = 0.0
    n_yaw = 0
    t0 = time.monotonic()
    while time.monotonic() - t0 < 2.0:
        qi, qj, qk, qr = read_safe(bno.quaternion, 4)
        if None not in (qi, qj, qk, qr):
            yaw, pitch, roll = quat_to_euler_deg(qi, qj, qk, qr)
            yaw_acc += yaw
            n_yaw += 1
        time.sleep(0.02)
    mount_yaw_offset = (yaw_acc / n_yaw) if n_yaw > 0 else 0.0
    # Normalizza a [-180, 180]
    if mount_yaw_offset > 180: mount_yaw_offset -= 360
    if mount_yaw_offset < -180: mount_yaw_offset += 360

    print("BNO085 avviato. Premere CTRL+C per uscire.")
    print(f"(Offset montaggio orizzontale stimato: {mount_yaw_offset:+.1f}°)")
    print("-" * 150)
    header = (
        "ACC[m/s^2] ax ay az | LINACC[m/s^2] lax lay laz | G[m/s^2] gx gy gz | "
        "GYRO[°/s] wx wy wz | MAG[µT] mx my mz | EULER[°] yaw pitch roll | "
        "G long lat vert | |G|"
    )
    print(header)
    print("-" * 150)

    target_hz = 50.0
    dt = 1.0 / target_hz
    next_t = time.monotonic()

    try:
        while True:
            # --- Letture robuste
            ax, ay, az   = read_safe(bno.acceleration, 3)
            lax, lay, laz = read_safe(bno.linear_acceleration, 3)
            gx, gy, gz   = read_safe(bno.gravity, 3)
            wx, wy, wz   = read_safe(bno.gyro, 3)
            mx, my, mz   = read_safe(bno.magnetic, 3)
            qi, qj, qk, qr = read_safe(bno.quaternion, 4)

            # Euler
            if None not in (qi, qj, qk, qr):
                yaw, pitch, roll = quat_to_euler_deg(qi, qj, qk, qr)
            else:
                yaw = pitch = roll = None

            # --------- G longitudinali/laterali/verticali
            # 1) usiamo linear_acceleration (già senza gravità)
            # 2) "levelliamo" con roll e pitch (rimuove pendenze del sensore)
            # 3) compensiamo l'offset di montaggio sul piano orizzontale con mount_yaw_offset
            if None not in (lax, lay, laz) and None not in (yaw, pitch, roll):
                # de-tilt: rimuovi roll/pitch (appl. rotazioni inverse su X e Y)
                ax_lvl, ay_lvl, az_lvl = rot_rx((lax, lay, laz), roll)
                ax_lvl, ay_lvl, az_lvl = rot_ry((ax_lvl, ay_lvl, az_lvl), pitch)
                # compensa la rotazione di montaggio in orizzontale
                ax_v, ay_v = rotate_xy((ax_lvl, ay_lvl), -mount_yaw_offset)
                # G in frame veicolo: long ~ avanti, lat ~ sinistra, vert ~ su
                Glong = ax_v / G0
                Glat  = ay_v / G0
                Gvert = az_lvl / G0
                Gtot  = math.sqrt(Glong*Glong + Glat*Glat + Gvert*Gvert)
            else:
                Glong = Glat = Gvert = Gtot = None

            # Stampa
            line = (
                f"{fmt(ax)} {fmt(ay)} {fmt(az)} | "
                f"{fmt(lax)} {fmt(lay)} {fmt(laz)} | "
                f"{fmt(gx)} {fmt(gy)} {fmt(gz)} | "
                f"{fmt(wx)} {fmt(wy)} {fmt(wz)} | "
                f"{fmt(mx)} {fmt(my)} {fmt(mz)} | "
                f"{fmt(yaw)} {fmt(pitch)} {fmt(roll)} | "
                f"{fmt(Glong)} {fmt(Glat)} {fmt(Gvert)} | {fmt(Gtot)}"
            )
            print(line, flush=True)

            # pacing
            next_t += dt
            sleep_t = next_t - time.monotonic()
            if sleep_t > 0: time.sleep(sleep_t)
            else: next_t = time.monotonic()

    except KeyboardInterrupt:
        print("\nUscita. Bye!")

if __name__ == "__main__":
    main()

#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import math, time, sys
import board, busio

from adafruit_bno08x.i2c import BNO08X_I2C
from adafruit_bno08x import (
    BNO_REPORT_ACCELEROMETER,
    BNO_REPORT_LINEAR_ACCELERATION,
    BNO_REPORT_GYROSCOPE,
    BNO_REPORT_GRAVITY,
    BNO_REPORT_GAME_ROTATION_VECTOR,  # <-- usiamo questo
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
    r = rad(roll_deg)
    cr, sr = math.cos(-r), math.sin(-r)
    x, y, z = v
    return (x, cr*y - sr*z, sr*y + cr*z)

def rot_ry(v, pitch_deg):
    p = rad(pitch_deg)
    cp, sp = math.cos(-p), math.sin(-p)
    x, y, z = v
    return (cp*x + sp*z, y, -sp*x + cp*z)

def rotate_xy(vxy, yaw_deg):
    y = rad(yaw_deg)
    c, s = math.cos(y), math.sin(y)
    x, yv = vxy
    return (c*x - s*yv, s*x + c*yv)

def enable_compat(bno, feature, report_us=100_000):
    # tenta con parametro posizionale, poi senza
    try:
        bno.enable_feature(feature, report_us)
    except TypeError:
        bno.enable_feature(feature)

def read_safe(getter, n):
    """Ritorna tuple di n o (None,)*n ignorando QUALSIASI errore del driver."""
    try:
        vals = getter
        if vals is None:
            return (None,)*n
        return vals
    except Exception as e:
        # Se vuoi fare debug, scommenta:
        # print(f"[IGNORA] {type(e).__name__}: {e}", file=sys.stderr)
        return (None,)*n

# ---------- Main ----------
def main():
    # I2C init (addr default 0x4A; se ADR alto: BNO08X_I2C(i2c, address=0x4B))
    i2c = busio.I2C(board.SCL, board.SDA)
    bno = BNO08X_I2C(i2c)

    # Prova a spegnere verbose interni (se presenti)
    try:
        if getattr(bno, "_debug", False):
            bno._debug = False
    except Exception:
        pass

    # Rate bassi per iniziare (10 Hz)
    rates = {
        BNO_REPORT_ACCELEROMETER:        100_000,
        BNO_REPORT_LINEAR_ACCELERATION:  100_000,
        BNO_REPORT_GYROSCOPE:            100_000,
        BNO_REPORT_GRAVITY:              100_000,
        BNO_REPORT_GAME_ROTATION_VECTOR: 100_000,  # niente magnetometro
    }
    for feat, us in rates.items():
        try:
            enable_compat(bno, feat, us)
        except Exception as e:
            print(f"Impossibile abilitare feature {feat}: {e}", file=sys.stderr)

    time.sleep(0.5)  # warm-up

    # Calibrazione yaw di montaggio con GAME quaternion (2s, kart fermo e dritto)
    yaw_acc = 0.0
    n_yaw = 0
    t0 = time.monotonic()
    while time.monotonic() - t0 < 2.0:
        gi, gj, gk, gr = read_safe(bno.game_quaternion, 4)
        if None not in (gi, gj, gk, gr):
            yaw, pitch, roll = quat_to_euler_deg(gi, gj, gk, gr)
            yaw_acc += yaw
            n_yaw += 1
        time.sleep(0.02)
    mount_yaw_offset = (yaw_acc / n_yaw) if n_yaw else 0.0
    if mount_yaw_offset > 180: mount_yaw_offset -= 360
    if mount_yaw_offset < -180: mount_yaw_offset += 360

    print("BNO085 avviato. Premere CTRL+C per uscire.")
    print(f"(Offset montaggio orizzontale stimato: {mount_yaw_offset:+.1f}°)")
    print("-" * 150)
    header = (
        "ACC[m/s^2] ax ay az | LINACC[m/s^2] lax lay laz | G[m/s^2] gx gy gz | "
        "GYRO[°/s] wx wy wz | EULER(game)[°] yaw pitch roll | "
        "G long lat vert | |G|"
    )
    print(header)
    print("-" * 150)

    target_hz = 20.0   # 20 Hz stampa/lettura (coerente con 10 Hz di report)
    dt = 1.0 / target_hz
    next_t = time.monotonic()

    try:
        while True:
            ax, ay, az     = read_safe(bno.acceleration, 3)
            lax, lay, laz  = read_safe(bno.linear_acceleration, 3)
            gx, gy, gz     = read_safe(bno.gravity, 3)
            wx, wy, wz     = read_safe(bno.gyro, 3)

            gi, gj, gk, gr = read_safe(bno.game_quaternion, 4)  # <-- più robusto
            if None not in (gi, gj, gk, gr):
                yaw, pitch, roll = quat_to_euler_deg(gi, gj, gk, gr)
            else:
                yaw = pitch = roll = None

            # G long/lat/vert dal frame veicolo
            if None not in (lax, lay, laz) and None not in (yaw, pitch, roll):
                ax_lvl, ay_lvl, az_lvl = rot_rx((lax, lay, laz), roll)
                ax_lvl, ay_lvl, az_lvl = rot_ry((ax_lvl, ay_lvl, az_lvl), pitch)
                ax_v, ay_v = rotate_xy((ax_lvl, ay_lvl), -mount_yaw_offset)
                Glong = ax_v / G0
                Glat  = ay_v / G0
                Gvert = az_lvl / G0
                Gtot  = math.sqrt(Glong*Glong + Glat*Glat + Gvert*Gvert)
            else:
                Glong = Glat = Gvert = Gtot = None

            line = (
                f"{fmt(ax)} {fmt(ay)} {fmt(az)} | "
                f"{fmt(lax)} {fmt(lay)} {fmt(laz)} | "
                f"{fmt(gx)} {fmt(gy)} {fmt(gz)} | "
                f"{fmt(wx)} {fmt(wy)} {fmt(wz)} | "
                f"{fmt(yaw)} {fmt(pitch)} {fmt(roll)} | "
                f"{fmt(Glong)} {fmt(Glat)} {fmt(Gvert)} | {fmt(Gtot)}"
            )
            print(line, flush=True)

            next_t += dt
            sleep_t = next_t - time.monotonic()
            if sleep_t > 0: time.sleep(sleep_t)
            else: next_t = time.monotonic()

    except KeyboardInterrupt:
        print("\nUscita. Bye!")

if __name__ == "__main__":
    main()

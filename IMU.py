#!/usr/bin/env python3
import time, sys
import board, busio
from adafruit_bno08x.i2c import BNO08X_I2C
from adafruit_bno08x import BNO_REPORT_ACCELEROMETER

# Nota: indirizzo default 0x4A. Se ADR alto: BNO08X_I2C(i2c, address=0x4B)
i2c = busio.I2C(board.SCL, board.SDA)  # su Pi: bus 1
bno = BNO08X_I2C(i2c)

# Abilita con massima compatibilità:
REPORT_US = 100_000  # 10 Hz
try:
    bno.enable_feature(BNO_REPORT_ACCELEROMETER, REPORT_US)   # vecchie: arg posizionale
except TypeError:
    bno.enable_feature(BNO_REPORT_ACCELEROMETER)              # fallback: senza intervallo

time.sleep(0.3)  # warm-up per il primo pacchetto

print("Leggo accelerometro (Ctrl+C per uscire)")
try:
    while True:
        try:
            ax, ay, az = bno.acceleration  # m/s^2 (con gravità)
            print(f"ACC: {ax:+.3f} {ay:+.3f} {az:+.3f}  m/s^2", flush=True)
        except RuntimeError as e:
            # report non pronto: ignora la lettura di questo giro
            print(f"(no report) {e}", file=sys.stderr)
        except OSError as e:
            # glitch I2C: attendi e riprova
            print(f"(I2C) {e}", file=sys.stderr)
            time.sleep(0.05)
        time.sleep(0.1)  # 10 Hz
except KeyboardInterrupt:
    pass

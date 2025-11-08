#!/usr/bin/env bash
set -euo pipefail

# ────────────────────────────────────────────────────────────────────
#  Setup modem SIMCom A7670E (PPP) + Python GNSS sender su Raspberry Pi
#  - Abilita UART + I2C
#  - Installa pacchetti necessari
#  - Configura PPP (chat + peers)
#  - Installa librerie Python con pip3 --user (senza venv)
#  - Crea servizi systemd (PPP + Python) e li avvia
#  Esegui come root: sudo ./setup_modem_gnss.sh
# ────────────────────────────────────────────────────────────────────

PI_USER="pi"
PI_HOME="/home/${PI_USER}"
APP_DIR="${PI_HOME}/DEVICE_RACESENSE"
PY_FILE="${APP_DIR}/GPIO_MODEMGNSS.py"

# ───── 0) Controlli preliminari ─────
if [[ "$(id -u)" -ne 0 ]]; then
  echo "Esegui come root: sudo $0"
  exit 1
fi

if [[ ! -f "${PY_FILE}" ]]; then
  echo "ATTENZIONE: Non trovo ${PY_FILE}"
  echo "Metti il tuo file nel percorso indicato o aggiorna il percorso nello script."
  exit 1
fi

# ───── 1) Pacchetti di base ─────
echo "[*] Aggiornamento pacchetti…"
apt-get update -y

echo "[*] Installo dipendenze APT…"
apt-get install -y \
  ppp pppconfig screen \
  usb-modeswitch \
  python3 python3-pip python3-serial python3-rpi.gpio \
  minicom \
  raspi-config \
  dos2unix \
  i2c-tools

# ModemManager (se presente) può bloccare le porte seriali del modem
#systemctl disable --now ModemManager 2>/dev/null || true

# ───── 2) Abilita UART e I2C (non interattivo) ─────
#echo "[*] Abilito UART e I2C…"
#raspi-config nonint do_serial 2   # disabilita console seriale, abilita UART
#raspi-config nonint do_i2c 0      # abilita I2C

# ───── 3) Gruppi / permessi utili ─────
#echo "[*] Aggiungo ${PI_USER} ai gruppi dialout, gpio, tty, i2c…"
#usermod -aG dialout,gpio,tty,i2c "${PI_USER}"

# ───── 4) Librerie Python via pip3 (user install, niente upgrade pip!) ─────
echo "[*] Installo librerie Python per l'utente ${PI_USER} con pip3 --user…"
# mi assicuro che pip3 esista (reinstall se necessario)
apt-get install -y python3-pip

# installo nei path utente (~/.local)
sudo -u "${PI_USER}" -H pip3 install --user pynmea2 pyserial --break-system-packages

# ───── 5) File PPP: chatscript e peers ─────
echo "[*] Scrivo chatscript PPP in /etc/chatscripts/ppp0"
install -d -m 0755 /etc/chatscripts
cat > /etc/chatscripts/ppp0 <<'EOF'
ABORT "BUSY"
ABORT "VOICE"
ABORT "NO CARRIER"
ABORT "NO DIALTONE"
ABORT "NO DIAL TONE"
ABORT "NO ANSWER"
ABORT "DELAYED"
ABORT "ERROR"
ABORT "+CGATT: 0"

TIMEOUT 12
"" AT
OK ATH
OK ATE0
OK AT+CGDCONT=1,"IP","m2m.vodafone.it"
OK ATD*99#
TIMEOUT 30
CONNECT ""
EOF
chmod 0644 /etc/chatscripts/ppp0

echo "[*] Scrivo peers PPP in /etc/ppp/peers/ppp0"
install -d -m 0755 /etc/ppp/peers
cat > /etc/ppp/peers/ppp0 <<'EOF'
/dev/ttyS0
115200
connect "/usr/sbin/chat -v -f /etc/chatscripts/ppp0"

noauth
defaultroute
usepeerdns
noipdefault
persist
debug

# MTU/MRU con margine rispetto a 1408
mtu 1400
mru 1400

# Recovery rapido e meno jitter
lcp-echo-failure 3
lcp-echo-interval 20
holdoff 5
maxfail 0

# Evita compressioni inutili
novj
nobsdcomp
nodeflate
nopcomp
noaccomp

logfile /var/log/ppp-ppp0.log
EOF
chmod 0644 /etc/ppp/peers/ppp0

# ───── 6) Service systemd per PPP (ppp0) ─────
echo "[*] Creo servizio systemd per ppp0"
cat > /etc/systemd/system/ppp0.service <<'EOF'
[Unit]
Description=PPP link via peers/ppp0 (SIMCom A7670E)
After=network-pre.target
Wants=network-pre.target

[Service]
Type=simple
ExecStart=/usr/sbin/pppd call ppp0 nodetach
Restart=always
RestartSec=5

[Install]
WantedBy=multi-user.target
EOF
chmod 0644 /etc/systemd/system/ppp0.service

# ───── 7) Service systemd per lo script Python ─────
# Nota: User=pi -> Python carica automaticamente i pacchetti da ~/.local
# Aggiungo PATH con ~/.local/bin per sicurezza.
echo "[*] Creo servizio systemd per lo script GNSS"
cat > /etc/systemd/system/gpio-modemgnss.service <<EOF
[Unit]
Description=RTK/GNSS UDP forwarder (GPIO_MODEMGNSS.py)
After=network-online.target ppp0.service
Wants=network-online.target
Requires=ppp0.service

[Service]
Type=simple
User=${PI_USER}
Group=${PI_USER}
WorkingDirectory=${APP_DIR}
Environment=PYTHONUNBUFFERED=1
Environment=PATH=/usr/local/bin:/usr/bin:/bin:${PI_HOME}/.local/bin
ExecStart=/usr/bin/python3 ${PY_FILE}
Restart=on-failure
RestartSec=5

[Install]
WantedBy=multi-user.target
EOF
chmod 0644 /etc/systemd/system/gpio-modemgnss.service

# ───── 8) Allinea fine-riga e proprietà file utente ─────
echo "[*] Correggo fine-riga e permessi nel progetto"
dos2unix "${PY_FILE}" 2>/dev/null || true
chown -R "${PI_USER}:${PI_USER}" "${APP_DIR}"

# ───── 9) Abilita e avvia servizi ─────
echo "[*] Ricarico systemd, abilito e avvio i servizi…"
systemctl daemon-reload
systemctl enable ppp0.service
systemctl enable gpio-modemgnss.service

# Avvio PPP prima, poi il Python (dipende da ppp0)
systemctl restart ppp0.service
sleep 3
systemctl restart gpio-modemgnss.service

echo
echo "────────────────────────────────────────────────────────────────────"
echo " FATTO!"
echo " - UART e I2C abilitati (se era attiva la console seriale, può servire un riavvio)."
echo " - PPP configurato (peers: /etc/ppp/peers/ppp0, chat: /etc/chatscripts/ppp0)."
echo " - Librerie Python installate per ${PI_USER} in ~/.local"
echo " - Servizi:"
echo "     • ppp0.service            (pppd call ppp0, persist)"
echo "     • gpio-modemgnss.service  (esegue il tuo Python)"
echo
echo " Comandi utili:"
echo "   journalctl -u ppp0 -f"
echo "   journalctl -u gpio-modemgnss -f"
echo "   systemctl status ppp0 gpio-modemgnss"
echo
echo " Se /dev/ttyS0 o la seriale non partono, RIAVVIA il Raspberry."
echo "────────────────────────────────────────────────────────────────────"

#!/usr/bin/env bash
set -euo pipefail

# ────────────────────────────────────────────────────────────────────
#  Setup modem SIMCom A7670E (PPP) + Python GNSS sender su Raspberry Pi
#  - Installa pacchetti necessari
#  - Configura UART, PPP, chat/peers
#  - Installa librerie Python
#  - Crea servizi systemd: ppp0.service + gpio-modemgnss.service
#  - Avvia i servizi
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
# ppp + chat per la connessione dati; python + moduli, GPIO, seriale
apt-get install -y \
  ppp pppconfig screen \
  usb-modeswitch \
  python3 python3-pip python3-venv \
  python3-serial python3-rpi.gpio \
  minicom \
  raspi-config \
  dos2unix

# ModemManager (se presente) talvolta interferisce: meglio disabilitare
systemctl disable --now ModemManager 2>/dev/null || true

# ───── 2) Abilita UART su Raspberry Pi ─────
# (necessario per /dev/ttyS0. Disabilita la serial console e abilita UART)
CONFIG_TXT="/boot/config.txt"
[[ -f /boot/firmware/config.txt ]] && CONFIG_TXT="/boot/firmware/config.txt"

echo "[*] Abilito UART nel file: ${CONFIG_TXT}"
if ! grep -q "^enable_uart=1" "${CONFIG_TXT}"; then
  echo "enable_uart=1" >> "${CONFIG_TXT}"
fi

CMDLINE="/boot/cmdline.txt"
[[ -f /boot/firmware/cmdline.txt ]] && CMDLINE="/boot/firmware/cmdline.txt"
if grep -q "console=serial0,115200" "${CMDLINE}"; then
  echo "[*] Rimuovo console seriale da cmdline"
  sed -i -E 's/\s*console=serial0,115200//g' "${CMDLINE}"
fi

# ───── 3) Gruppi / permessi utili ─────
echo "[*] Aggiungo ${PI_USER} ai gruppi dialout, gpio, tty…"
usermod -aG dialout,gpio,tty "${PI_USER}"

# ───── 4) Librerie Python via pip (extra) ─────
# pynmea2 + pyserial (già da apt, ma mettiamo anche via pip per sicurezza versione)
echo "[*] Installo librerie Python aggiuntive con pip…"
python3 -m pip install --upgrade pip
python3 -m pip install pynmea2 pyserial

# ───── 5) File PPP: chatscript e peers ─────
echo "[*] Scrivo chatscript PPP in /etc/chatscripts/gprs"
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
ExecStart=/usr/bin/python3 ${PY_FILE}
Restart=on-failure
RestartSec=5
Environment=PYTHONUNBUFFERED=1

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

# Avvio PPP prima, poi il Python (lo unit del Python dipende da ppp0)
systemctl restart ppp0.service
sleep 3
systemctl restart gpio-modemgnss.service

echo
echo "────────────────────────────────────────────────────────────────────"
echo " FATTO!"
echo " - UART abilitata (potrebbe essere necessario un riavvio per /boot/*)."
echo " - PPP configurato (peers: /etc/ppp/peers/ppp0, chat: /etc/chatscripts/ppp0)."
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

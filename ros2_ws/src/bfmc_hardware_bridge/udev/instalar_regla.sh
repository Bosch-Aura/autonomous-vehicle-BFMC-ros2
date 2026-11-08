#!/usr/bin/env bash
# Instala la regla udev que da al ESP32 el nombre fijo /dev/esp32 (ver 99-bfmc-esp32.rules).
# Se corre una sola vez por Jetson y pide la contraseña de sudo.
set -euo pipefail

REGLA="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/99-bfmc-esp32.rules"
DESTINO=/etc/udev/rules.d/99-bfmc-esp32.rules

sudo install -m 0644 "$REGLA" "$DESTINO"
sudo udevadm control --reload-rules
sudo udevadm trigger --subsystem-match=tty --action=add
udevadm settle --timeout=5 || true

if [ -e /dev/esp32 ]; then
  echo "Listo: /dev/esp32 -> $(readlink -f /dev/esp32)"
else
  echo "Regla instalada en $DESTINO, pero /dev/esp32 no aparece: ¿está enchufado el ESP32?" >&2
  echo "Comprobá que lsusb muestre 1a86:7523 (QinHeng CH340)." >&2
  exit 1
fi

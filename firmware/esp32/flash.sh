#!/usr/bin/env bash
# Compila y sube un sketch al ESP32 con arduino-cli, y abre el monitor serial.
#
# Uso:
#   ./flash.sh                          sube main/main.ino
#   ./flash.sh test_motor               sube tests_componentes/test_motor
#   ./flash.sh test_motor /dev/ttyUSB1  elige el puerto a mano
#
# El modelo de placa se detecta solo. Para forzarlo:
#   FQBN=esp32:esp32:esp32s3 ./flash.sh

set -euo pipefail

BAUDRATE=115200
CORE="esp32:esp32"
DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

error() {
  echo "❌ $1" >&2
  exit 1
}

# --- 1. Elegir el sketch -----------------------------------------------------

sketch_name="${1:-main}"
if [ "$sketch_name" = "main" ]; then
  sketch_dir="$DIR/main"
else
  sketch_dir="$DIR/tests_componentes/$sketch_name"
fi

if [ ! -f "$sketch_dir/$(basename "$sketch_dir").ino" ]; then
  echo "❌ No existe el sketch '$sketch_name'." >&2
  echo "   Sketches disponibles:" >&2
  echo "     main" >&2
  for d in "$DIR"/tests_componentes/*/; do
    echo "     $(basename "$d")" >&2
  done
  exit 1
fi

# --- 2. Revisar herramientas -------------------------------------------------

if ! command -v arduino-cli > /dev/null; then
  echo "❌ arduino-cli no está instalado. Para instalarlo:" >&2
  echo "   curl -fsSL https://raw.githubusercontent.com/arduino/arduino-cli/master/install.sh | BINDIR=~/.local/bin sh" >&2
  exit 1
fi

if ! arduino-cli core list | grep -q "^$CORE "; then
  echo "📦 Instalando soporte para ESP32 (solo la primera vez, tarda unos minutos)..."
  arduino-cli core update-index
  arduino-cli core install "$CORE"
fi

# --- 3. Encontrar el puerto --------------------------------------------------

port="${2:-}"
if [ -z "$port" ]; then
  ports=(/dev/ttyUSB* /dev/ttyACM*)
  found=()
  for p in "${ports[@]}"; do
    [ -e "$p" ] && found+=("$p")
  done

  if [ "${#found[@]}" -eq 0 ]; then
    error "No encontré ningún ESP32 conectado. Revisá el cable USB (tiene que ser de datos, no solo de carga)."
  elif [ "${#found[@]}" -eq 1 ]; then
    port="${found[0]}"
  else
    echo "Hay más de un puerto. ¿Cuál es el ESP32?"
    select p in "${found[@]}"; do
      [ -n "$p" ] && port="$p" && break
    done
  fi
fi

[ -e "$port" ] || error "El puerto $port no existe."
[ -w "$port" ] || error "No tenés permiso para usar $port. Corré: sudo usermod -aG dialout \$USER (y después cerrá sesión y volvé a entrar)."

echo "🔌 Puerto: $port"

# --- 4. Detectar el modelo de placa ------------------------------------------

detect_fqbn() {
  local data_dir esptool chip
  data_dir="$(arduino-cli config get directories.data)"
  esptool="$(ls -d "$data_dir"/packages/esp32/tools/esptool_py/*/esptool 2> /dev/null | tail -1)"
  [ -x "$esptool" ] || return 1

  chip="$("$esptool" --port "$port" chip-id 2>&1 | grep -oE 'ESP32(-(S2|S3|C2|C3|C5|C6|H2|P4))?' | head -1)" || return 1
  case "$chip" in
    ESP32-S2) echo "$CORE:esp32s2" ;;
    ESP32-S3) echo "$CORE:esp32s3" ;;
    ESP32-C2) echo "$CORE:esp32c2" ;;
    ESP32-C3) echo "$CORE:esp32c3" ;;
    ESP32-C5) echo "$CORE:esp32c5" ;;
    ESP32-C6) echo "$CORE:esp32c6" ;;
    ESP32-H2) echo "$CORE:esp32h2" ;;
    ESP32-P4) echo "$CORE:esp32p4" ;;
    ESP32)    echo "$CORE:esp32" ;;
    *)        return 1 ;;
  esac
}

if [ -z "${FQBN:-}" ]; then
  echo "🔍 Detectando el modelo de ESP32..."
  if FQBN="$(detect_fqbn)"; then
    echo "   Detectado: $FQBN"
  else
    echo "   No pude detectarlo. Elegí el modelo (si no sabés, probá con la opción 1):"
    select FQBN in "$CORE:esp32" "$CORE:esp32s3" "$CORE:esp32c3" "$CORE:esp32s2"; do
      [ -n "$FQBN" ] && break
    done
  fi
fi

# --- 5. Compilar, subir y abrir el monitor -----------------------------------

echo "🔨 Compilando $sketch_name para $FQBN..."
arduino-cli compile --fqbn "$FQBN" "$sketch_dir"

echo "⬆️  Subiendo a $port..."
arduino-cli upload --fqbn "$FQBN" --port "$port" "$sketch_dir"

echo "✅ Listo. Abriendo monitor serial a $BAUDRATE baudios (Ctrl+C para salir)..."
arduino-cli monitor --port "$port" --config "baudrate=$BAUDRATE"

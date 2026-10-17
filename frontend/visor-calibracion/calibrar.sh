#!/usr/bin/env bash
# Prepara y lanza, en una sola terminal de la Jetson, todo lo que necesita el visor de calibración:
# la cámara, la detección de carril, web_video_server y rosbridge.
#
# Uso (desde cualquier carpeta):
#   frontend/visor-calibracion/calibrar.sh              # compila si hace falta y lanza
#   frontend/visor-calibracion/calibrar.sh --compilar   # fuerza recompilar (por ejemplo, tras cambiar de rama)
#
# Ctrl+C detiene todo. Los logs de cada proceso quedan en /tmp/visor-calibracion/.
# Sin `-u`: el setup.bash de ROS usa variables sin definir.
set -eo pipefail
# Control de trabajos: cada proceso lanzado en segundo plano queda en su propio grupo y no ignora
# SIGINT (sin esto, bash se lo hace ignorar y Ctrl+C no los detiene).
set -m

RAIZ="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
ROS_SETUP=/opt/ros/humble/setup.bash
WS="$RAIZ/ros2_ws"
LOGS="${TMPDIR:-/tmp}/visor-calibracion"
ESPERA_TOPICS_S=20

compilar=false
for arg in "$@"; do
  case "$arg" in
    --compilar) compilar=true ;;
    -h|--help) sed -n '2,9p' "$0"; exit 0 ;;
    *) echo "Opción desconocida: $arg" >&2; exit 2 ;;
  esac
done

if [ ! -f "$ROS_SETUP" ]; then
  echo "No encuentro ROS 2 Humble en $ROS_SETUP. Instalalo antes de seguir." >&2
  exit 1
fi
source "$ROS_SETUP"

if $compilar; then
  echo "== Limpiando build, install y log"
  find "$WS/build" "$WS/install" "$WS/log" -mindepth 1 -delete 2> /dev/null || true
fi

if $compilar || [ ! -f "$WS/install/setup.bash" ]; then
  echo "== Compilando el workspace (puede pedir tu contraseña para instalar dependencias)"
  rosdep update || echo "Aviso: 'rosdep update' falló (¿sin internet?). Sigo con lo que haya."
  (cd "$WS" && rosdep install --from-paths src --ignore-src -r -y && colcon build --symlink-install)
fi
source "$WS/install/setup.bash"

faltan=()
for paquete in realsense2_camera web_video_server rosbridge_server; do
  ros2 pkg prefix "$paquete" > /dev/null 2>&1 || faltan+=("ros-humble-${paquete//_/-}")
done
if [ "${#faltan[@]}" -gt 0 ]; then
  echo "Faltan paquetes de ROS: ${faltan[*]}" >&2
  echo "Instalalos con:  sudo apt install ${faltan[*]}" >&2
  exit 1
fi

mkdir -p "$LOGS"
pids=()
nombres=()

lanzar() {  # lanzar <nombre> <comando...>
  local nombre=$1
  shift
  "$@" > "$LOGS/$nombre.log" 2>&1 &
  pids+=("$!")
  nombres+=("$nombre")
  echo "  $nombre (pid $!)  ->  $LOGS/$nombre.log"
}

detener() {
  trap - EXIT
  echo
  echo "== Deteniendo todo"
  for pid in "${pids[@]}"; do kill -INT -- "-$pid" 2> /dev/null || true; done
  wait 2> /dev/null || true
}
trap detener EXIT
trap 'exit 130' INT TERM

echo "== Lanzando"
lanzar camara ros2 launch bfmc_camera camera.launch.py
lanzar carril ros2 launch bfmc_lane_detection lane_detection.launch.py
lanzar video ros2 run web_video_server web_video_server
lanzar rosbridge ros2 launch rosbridge_server rosbridge_websocket_launch.xml

echo "== Esperando los topics (hasta ${ESPERA_TOPICS_S} s cada uno)"
for topic in /camera/color/image_raw /perception/lane; do
  visto=false
  for _ in $(seq 1 "$ESPERA_TOPICS_S"); do
    if ros2 topic list 2> /dev/null | grep -qx "$topic"; then visto=true; break; fi
    sleep 1
  done
  if $visto; then echo "  OK  $topic"; else echo "  NO  $topic  (mirá los logs en $LOGS)"; fi
done

echo "== Calidad de servicio de la imagen (si 'web_video_server' no muestra imagen, mirá esto)"
ros2 topic info -v /camera/color/image_raw 2> /dev/null | grep -E "Node name|Reliability" || echo "  (sin datos: el topic no tiene publicador)"

ip="$(hostname -I 2> /dev/null | awk '{print $1}')"
echo
echo "Listo. En tu computadora abrí frontend/visor-calibracion/index.html y escribí la IP: ${ip:-<la de la Jetson>}"
echo "Los puertos 8080 y 9090 quedan abiertos y sin autenticación: usá una red solo del equipo."
echo "Ctrl+C detiene todo."

wait -n || true
echo
echo "Un proceso terminó. Estado:"
for i in "${!pids[@]}"; do
  if ! kill -0 "${pids[$i]}" 2> /dev/null; then
    echo "--- ${nombres[$i]} se detuvo. Últimas líneas de su log:"
    tail -n 5 "$LOGS/${nombres[$i]}.log"
  fi
done
exit 1

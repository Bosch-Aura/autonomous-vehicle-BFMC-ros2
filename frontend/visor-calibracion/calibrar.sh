#!/usr/bin/env bash
# Prepara y lanza, en una sola terminal de la Jetson, todo lo que necesita el visor de calibración:
# la cámara, la detección de carril, web_video_server y rosbridge.
#
# Uso (desde cualquier carpeta):
#   frontend/visor-calibracion/calibrar.sh              # compila si hace falta y lanza
#   frontend/visor-calibracion/calibrar.sh --compilar   # fuerza recompilar (por ejemplo, tras cambiar de rama)
#   frontend/visor-calibracion/calibrar.sh --auto       # además del visor, arranca TODO el sistema
#                                                       # (control, decisión, puente con el ESP32): el auto SE MUEVE
#   frontend/visor-calibracion/calibrar.sh --auto --puerto /dev/ttyACM0   # otro puerto serial
#   frontend/visor-calibracion/calibrar.sh --lane l_adaptativo              # otro detector de carril
#   frontend/visor-calibracion/calibrar.sh --auto --control pure_pursuit    # otro algoritmo de giro
#
# --lane y --control eligen los módulos al arrancar (ADR-0004); sin ellos corre lo que diga el
# params.yaml de cada paquete. También se eligen desde el visor, antes de Conectar.
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
auto=false
puerto=""
lane=""
control=""
while [ "$#" -gt 0 ]; do
  arg=$1
  shift
  case "$arg" in
    --compilar) compilar=true ;;
    --auto) auto=true ;;
    --puerto)
      [ "$#" -gt 0 ] || { echo "--puerto necesita un valor" >&2; exit 2; }
      puerto=$1
      shift ;;
    --lane)
      [ "$#" -gt 0 ] || { echo "--lane necesita un valor (hsv_ventanas o l_adaptativo)" >&2; exit 2; }
      lane=$1
      shift ;;
    --control)
      [ "$#" -gt 0 ] || { echo "--control necesita un valor (pid o pure_pursuit)" >&2; exit 2; }
      control=$1
      shift ;;
    -h|--help) sed -n '2,19p' "$0"; exit 0 ;;
    *) echo "Opción desconocida: $arg" >&2; exit 2 ;;
  esac
done

if [ -n "$control" ] && ! $auto; then
  echo "--control solo tiene sentido con --auto (sin --auto no se lanza el control)" >&2
  exit 2
fi

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

# Librerías de CUDA que se instalan con pip en el usuario (cuDSS, cuSPARSELt, cuBLAS…): el PyTorch
# de la Jetson las necesita y no están en una ruta que el sistema busque. Sin esto, `import torch`
# falla con "libcudss.so.0: cannot open shared object file". Si no hay ninguna, no cambia nada.
libs_pip=$(python3 -c "import site,glob;print(':'.join(glob.glob(site.getusersitepackages()+'/nvidia/*/lib')))" 2>/dev/null || true)
if [ -n "$libs_pip" ]; then export LD_LIBRARY_PATH="$libs_pip${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"; fi

# Modelo de la detección de señales (opcional: sin él solo no se puede activar esa detección).
# El comando no descarga nada si el modelo ya está y su checksum es correcto, y funciona sin
# internet en ese caso. No se baja al compilar: compilar tiene que andar sin red y sin 52 MB.
echo "== Modelo de señales"
if ! timeout 180 ros2 run bfmc_sign_detection descargar_modelo; then
  echo "Aviso: no se pudo obtener el modelo de señales. Sigo sin él: la detección de señales no" >&2
  echo "       se va a poder activar. Reintentá con:  ros2 run bfmc_sign_detection descargar_modelo" >&2
fi
if ! python3 -c "import ultralytics" > /dev/null 2>&1; then
  echo "Aviso: Ultralytics no se puede cargar (la detección de señales no se va a poder activar)." >&2
  echo "       Mirá el motivo con:  python3 -c 'import ultralytics'" >&2
  echo "       Si falta, instalalo sin reemplazar el PyTorch de la Jetson:  pip install --no-deps ultralytics ultralytics-thop" >&2
fi

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
if $auto; then
  opciones=()
  [ -n "$puerto" ] && opciones+=("serial_port:=$puerto")
  [ -n "$lane" ] && opciones+=("lane_detector:=$lane")
  [ -n "$control" ] && opciones+=("control_algorithm:=$control")
  lanzar sistema ros2 launch bfmc_bringup system.launch.py "${opciones[@]}"
else
  lanzar camara ros2 launch bfmc_camera camera.launch.py
  # Arranca apagado (no carga el modelo ni gasta CPU): se prende desde el visor.
  lanzar senales ros2 launch bfmc_sign_detection sign_detection.launch.py
  if [ -n "$lane" ]; then
    lanzar carril ros2 launch bfmc_lane_detection lane_detection.launch.py detector:="$lane"
  else
    lanzar carril ros2 launch bfmc_lane_detection lane_detection.launch.py
  fi
fi
lanzar video ros2 run web_video_server web_video_server
# Cada llamada a un servicio en su propio hilo y con tope: si un nodo no contesta, esa llamada
# vence sola y no traba las demás de la página (los valores por defecto de Humble, false y 0,
# la dejan esperando para siempre). El tope supera los 20 s que el visor espera la calibración.
lanzar rosbridge ros2 launch rosbridge_server rosbridge_websocket_launch.xml \
  call_services_in_new_thread:=true default_call_service_timeout:=25.0

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

if $auto; then
  echo
  echo "MODO AUTO: el sistema completo está corriendo y el auto va a moverse. Ruedas en el aire la primera vez."
  echo "Para detenerlo: Ctrl+C, o en otra terminal  ros2 service call /hardware/stop std_srvs/srv/SetBool \"{data: true}\""
fi

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

# Visor de calibración

Una página de un solo archivo (`index.html`) para **ver desde una computadora lo que ve la Jetson**: la imagen de la cámara, los datos de `/perception/lane` y el trapecio `warp_src` de `bfmc_lane_detection` para calibrarlo. No es el dashboard del vehículo (ver [`../README.md`](../README.md)): es una herramienta provisoria de calibración.

- **Solo lectura.** La página se suscribe a un topic y pide una imagen. No publica ni llama servicios.
- **Sin dependencias.** No usa librerías ni internet: habla el protocolo de `rosbridge` (JSON por WebSocket) directamente. Funciona en una red sin salida a internet.
- **No es un paquete ROS.** `colcon` no la compila ni la testea.

## Qué muestra

| Zona | Qué es |
|---|---|
| Imagen de la cámara | El stream de `web_video_server`, con el trapecio de `warp_src` dibujado encima |
| Carril | `detected`, `lateral_error_m`, `heading_error_rad`, `confidence` e `intersection_detected` de `/perception/lane`, y cuánto hace que llegó el último dato (rojo si pasó más de 1 s) |
| `warp_src` | Los ocho valores del trapecio, en el formato de `params.yaml`, listos para copiar |

## Cómo usarlo

### 1. En la Jetson: con el script

Una sola terminal, parado en cualquier carpeta del repositorio:

```bash
frontend/visor-calibracion/calibrar.sh
```

El script:

1. Carga ROS 2 Humble y compila el workspace si todavía no está compilado (`--compilar` fuerza una compilación limpia, por ejemplo tras cambiar de rama).
2. Comprueba que estén instalados `realsense2_camera`, `web_video_server` y `rosbridge_server`. Si falta alguno, dice qué `apt install` correr y termina.
3. Lanza la cámara, `lane_detection`, `web_video_server` y `rosbridge`, cada uno con su log en `/tmp/visor-calibracion/`.
4. Espera a que aparezcan los topics, muestra la política de calidad de servicio de la imagen y la IP de la Jetson.
5. **Ctrl+C detiene todo.** Si un proceso se cae, avisa cuál y muestra las últimas líneas de su log.

Necesita `sudo apt install ros-humble-web-video-server ros-humble-rosbridge-server` la primera vez.

<details>
<summary>A mano (lo mismo que hace el script)</summary>

En cuatro terminales, cada una con ROS y el workspace cargados:

```bash
source /opt/ros/humble/setup.bash
source ros2_ws/install/setup.bash
```

| Terminal | Comando |
|---|---|
| 1 | `ros2 launch bfmc_camera camera.launch.py` |
| 2 | `ros2 launch bfmc_lane_detection lane_detection.launch.py` |
| 3 | `ros2 run web_video_server web_video_server` |
| 4 | `ros2 launch rosbridge_server rosbridge_websocket_launch.xml` |

La terminal 4 también necesita el workspace cargado: `rosbridge` tiene que conocer el tipo `bfmc_interfaces/msg/LaneState` para pasarlo a JSON.

</details>

Por defecto, `web_video_server` usa el puerto 8080 y `rosbridge`, el 9090.

### 2. En tu computadora

Abrí `index.html` en el navegador (doble clic) y escribí la IP de la Jetson. Si el navegador bloquea algo al abrirlo como archivo, servilo desde esta carpeta:

```bash
python -m http.server 8000
```

y abrí `http://localhost:8000`. Después apretá **Conectar**.

### 3. Calibrar `warp_src`

1. Con el auto apoyado sobre la pista, arrastrá los cuatro puntos hasta que el trapecio calce con el carril, como lo debe ver el detector.
2. Apretá **Copiar** y pegá la línea en `ros2_ws/src/bfmc_lane_detection/config/params.yaml`.
3. Relanzá `lane_detection`: el nodo lee sus parámetros al arrancar.
4. Volvé a mirar `confidence` y `detected`.

Los valores iniciales de los puntos son los del `params.yaml` actual. **Valores del params.yaml** los restablece.

## Seguridad: leer antes de abrir los puertos

`rosbridge` **no autentica** y, tal como se lanza arriba, permite a cualquier cliente de la red publicar en cualquier topic (incluido `/cmd/drive`) y llamar servicios. `web_video_server` escucha en todas las interfaces (`0.0.0.0`) y tampoco autentica. Esta página no publica nada, pero los puertos quedan abiertos para quien los use.

- Usá una **red solo del equipo**, con contraseña (un router o un hotspot propio), no una Wi-Fi compartida.
- Limitá con **firewall** los puertos 8080 y 9090 de la Jetson a las IP de las computadoras del equipo.
- **Apagá todo** (Ctrl+C en el script) cuando no estés calibrando.
- Con el auto en movimiento, la seguridad sigue en el watchdog del ESP32 y en `bfmc_safety`, que no dependen de esta página.

## Qué está verificado y qué no

**Verificado en un navegador, contra un servidor falso** que imita a `web_video_server` y a `rosbridge`:

- La página muestra la imagen y actualiza los datos cuando llegan mensajes.
- Los valores iniciales de `warp_src` coinciden con los del `params.yaml`.
- El arrastre mueve solo el punto elegido y limita los valores a 0–1.
- Si cae la conexión, muestra "reintentando" y el último dato se marca en rojo.
- La única orden que envía la página es la suscripción (`subscribe`).

**Verificado en un contenedor de ROS 2 Humble (amd64), con una cámara falsa** que publica una imagen sintética en `/camera/color/image_raw`, junto con `lane_detection`, `web_video_server` 3.1.0 y `rosbridge_server` 2.0.8 de los repositorios de Humble:

- `calibrar.sh` compila el workspace, avisa de los paquetes que faltan, lanza los cuatro procesos y, con Ctrl+C, los detiene sin dejar ninguno vivo.
- `rosbridge` acepta la suscripción a `/perception/lane` con el tipo `bfmc_interfaces/msg/LaneState` y entrega `detected`, `lateral_error_m`, `heading_error_rad`, `confidence` e `intersection_detected`.
- `web_video_server` entrega la imagen en `/stream` cuando la cámara publica con calidad de servicio confiable **y** con "mejor esfuerzo". Para eso la página pide la imagen con `qos_profile=sensor_data`: sin ese parámetro, una cámara en "mejor esfuerzo" no llega al navegador.

**Sin verificar** (necesita la Jetson y la cámara real):

- Que los paquetes `ros-humble-web-video-server`, `ros-humble-rosbridge-server` y `ros-humble-realsense2-camera` existan para la arquitectura de la Jetson (arm64).
- Que el driver de la RealSense publique la imagen en `/camera/color/image_raw` con `bfmc_camera`, y con qué calidad de servicio.
- Que `detected` pase a verdadero con la pista real, y que el trapecio calibrado dé una buena detección.
- Que la red y el firewall permitan la conexión.

## Qué no hace (todavía)

- No muestra las líneas detectadas ni la vista cenital: `lane_detection` no publica una imagen anotada.
- No controla nada ni autentica.
- No muestra `sign_detection`, `/safety/status` ni `/vehicle/state`.

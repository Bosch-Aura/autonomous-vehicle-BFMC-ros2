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

### 1. En la Jetson

Con la cámara y `lane_detection` andando (por ejemplo, con `ros2 launch bfmc_bringup system.launch.py`), instalá y lanzá dos servidores:

```bash
sudo apt install ros-humble-web-video-server ros-humble-rosbridge-server

ros2 run web_video_server web_video_server
ros2 launch rosbridge_server rosbridge_websocket_launch.xml
```

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

`rosbridge` **no autentica** y, tal como se lanza arriba, permite a cualquier cliente de la red publicar en cualquier topic (incluido `/cmd/drive`) y llamar servicios. Esta página no lo hace, pero el puerto queda abierto para quien lo use.

- Usá una **red solo del equipo**, con contraseña (un router o un hotspot propio), no una Wi-Fi compartida.
- Limitá con **firewall** los puertos 8080 y 9090 de la Jetson a las IP de las computadoras del equipo.
- **Apagá `rosbridge` y `web_video_server`** cuando no estés calibrando.
- Con el auto en movimiento, la seguridad sigue en el watchdog del ESP32 y en `bfmc_safety`, que no dependen de esta página.

## Qué está verificado y qué no

**Verificado** (contra un servidor falso que imita a `web_video_server` y a `rosbridge`, en un navegador):

- La página muestra la imagen y actualiza los datos cuando llegan mensajes.
- Los valores iniciales de `warp_src` coinciden con los del `params.yaml`.
- El arrastre mueve solo el punto elegido y limita los valores a 0–1.
- Si cae la conexión, muestra "reintentando" y el último dato se marca en rojo.
- La única orden que envía la página es la suscripción (`subscribe`).

**Sin verificar** (necesita la Jetson y la cámara):

- Que `rosbridge` acepte la suscripción con el tipo `bfmc_interfaces/msg/LaneState` y que `bfmc_interfaces` esté instalado donde corre.
- Que el topic de la imagen, `/camera/color/image_raw`, sea el que publica el driver con `bfmc_camera` (si no, cambialo en la página).
- Que `web_video_server` y `rosbridge` existan como paquetes de Humble para la arquitectura de la Jetson, y sus puertos y direcciones de escucha por defecto.
- Que la red y el firewall permitan la conexión.
- Que el trapecio calibrado dé una buena detección con la pista real.

## Qué no hace (todavía)

- No muestra las líneas detectadas ni la vista cenital: `lane_detection` no publica una imagen anotada.
- No controla nada ni autentica.
- No muestra `sign_detection`, `/safety/status` ni `/vehicle/state`.

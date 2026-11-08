# Visor de calibración

Una página de un solo archivo (`index.html`) para **calibrar `bfmc_lane_detection` desde una computadora, viendo lo que ve la Jetson**. Muestra la imagen de la cámara, lo que el detector toma por línea, la vista desde arriba con las ventanas y las curvas, y los datos de `/perception/lane`. Deja **calibrar la vista desde arriba** con un botón, **cambiar los parámetros del detector en vivo** (el rectángulo del piso que se ve, el ancho del carril, el umbral de color y la región de interés), **guardarlos en la Jetson** y **manejar el auto**: velocidad, Iniciar y Frenar. No es el dashboard del vehículo (ver [`../README.md`](../README.md)): es una herramienta provisoria de calibración y de mando.

- **Lee, calibra y manda.** Se suscribe a `/perception/lane`, `/hardware/status` y `/vehicle/state`, pide imágenes y llama a servicios: `get_parameters`, `set_parameters`, `set_parameters_atomically`, `calibrate_ground` y `save_params` del detector; `get_parameters` y `set_parameters` del control; y `/hardware/stop`, `/hardware/arm` y `/hardware/obstacle_reset` del puente. No publica ningún topic.

**Distribución (A2-157).** Encabezado de una línea (título, IP, Conectar, Desconectar y el estado de la conexión; puertos, topics y nodo plegados en "Conexión avanzada"). A la izquierda, una grilla de 2 × 2: Cámara, Lo que ve el detector, Vista desde arriba y Calibración. A la derecha, un panel de 340 px con el Mando del auto y el Carril. Desde 1000 × 600 px la página entra en la pantalla sin scroll: las imágenes se achican para entrar en su celda y mantienen la proporción (container queries). En pantallas chicas todo se apila en una columna.
- **Sin dependencias.** No usa librerías ni internet: habla el protocolo de `rosbridge` (JSON por WebSocket) directamente. Funciona en una red sin salida a internet.
- **No es un paquete ROS.** `colcon` no la compila ni la testea.

## Qué muestra

| Zona | Qué es |
|---|---|
| **Cámara** | La imagen de la cámara, sin nada encima. |
| **Lo que ve el detector** | La imagen con lo que el detector toma por **blanco pintado de rojo**, el trapecio que usa (amarillo, lo calcula el detector), el límite de la región de interés (cian), las líneas que encontró (verde y azul), el desvío medido (magenta) y, con el Kalman activo, el filtrado (blanco). Arriba: el modo, el chequeo que falló y cómo se buscó cada línea. |
| **Vista desde arriba** | El piso "aplanado": la máscara de blancos, las ventanas que siguen cada línea, las curvas, el eje del auto (amarillo) y la fila donde mide (naranja). El borde de abajo de esta vista **no** es el auto: está `calib_forward_near_m` delante de la cámara. |
| **Calibración** | **Calibrar**, **Deshacer** y **Guardar**; el resultado de la calibración; tres deslizadores en metros con la parte del piso que se ve (desde, hasta y medio ancho); las medidas con cinta (ancho del carril, distancia entre ejes y de la cámara al eje trasero), cada una con cómo medirla; el color y la región; y, plegado, el texto para copiar a los `params.yaml` del repo. |
| **Mando del auto** | Estado del auto, del enlace con el ESP32, del ultrasónico y de la decisión. **Velocidad en cm/s** (0 a 50), **Iniciar**, **Frenar**, **Rearmar tras una falla** y **Destrabar obstáculo**. La explicación larga está plegada en "Cómo funciona". |
| **Carril** | De `/perception/lane`: el **modo** (dos líneas, una línea, predicho o PERDIDO), si cada línea es válida, desvío, rumbo, curvatura, confianza, intersección y cuánto hace que llegó el último dato. Con el carril perdido, desvío y rumbo se muestran como `--`. |

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

Servila desde esta carpeta (es la forma con la que se probó):

```bash
python -m http.server 8000
```

Abrí `http://localhost:8000/index.html`, escribí la IP de la Jetson y apretá **Conectar**. Al conectar, la página **lee del detector los valores que tiene en ese momento** (trapecio, color y región) y empieza desde ahí.

Si la imagen se queda en "conectando…" y el panel del carril sí funciona, el pedido de la imagen no está recibiendo cuadros: abrí `http://<IP>:8080` y mirá si el topic figura en la lista, y probá `http://<IP>:8080/stream_viewer?topic=/camera/color/image_raw`.

### 3. Calibrar

Con el auto **quieto** sobre la pista, la cámara montada y el piso delante del auto **despejado**:

1. **Calibrar.** Mide con la profundidad la altura, la inclinación y el giro de la cámara (A2-151). El trapecio y la escala de la vista desde arriba salen de ahí: **no se arrastran a mano**, así la escala nunca queda desatada del trapecio. Si el auto está en marcha, la página no llama al servicio y avisa (el detector también lo rechaza). Muestra lo que midió, o el motivo si no pudo (por ejemplo, algo delante de la cámara). **Deshacer** vuelve a la calibración anterior.
2. **Qué parte del piso ver.** Con los tres deslizadores en metros (desde, hasta y medio ancho, medidos desde el pie de la cámara). El detector vuelve a proyectar el trapecio al instante con la pose medida; si se sale de la imagen, lo rechaza y avisa.
3. **Medidas con cinta.** El **ancho del carril** (centro a centro de las líneas) va al detector: no cambia la escala, la usan los chequeos y el modo "una línea". La **distancia entre ejes** y la de la **cámara al eje trasero** van al control (Pure Pursuit).
4. **Color.** Mirá qué pinta de rojo en "Lo que ve el detector". Si toma por blanco cosas que no son línea, subí el brillo mínimo o bajá la saturación máxima.
5. **Región.** "Descartar de arriba" ignora el fondo (línea cian). Tiene que quedar por encima del trapecio.
6. **Guardar.** Deja en la Jetson (`~/.config/bfmc/lane_detection.yaml`) la calibración, la parte del piso, el ancho del carril, el color y la región; se cargan al arrancar. La distancia entre ejes y la de la cámara al eje trasero son del control y **no** las guarda: copialas al `params.yaml` del repo ("Para dejarlo fijo en el repo"), porque son medidas del auto, iguales para todo el equipo.

**Aplicar en vivo** se puede apagar: la página no manda nada y los cambios quedan solo en el navegador. **Valores del detector** vuelve a los que se leyeron al conectar.

### Servicios que usa la calibración

| Servicio | Tipo | Qué hace | Espera |
|---|---|---|---|
| `<nodo del detector>/calibrate_ground` (por defecto `/lane_detection/calibrate_ground`) | `std_srvs/srv/Trigger` | Mide la pose de la cámara con la profundidad y la aplica. `success` y en `message` lo medido (`altura 0.205 m, pitch 20.1°, …`) o el motivo | 20 s |
| `<nodo>/save_params` | `std_srvs/srv/Trigger` | Guarda en la Jetson; `message` dice dónde | 3 s |
| `<nodo>/set_parameters_atomically` | `rcl_interfaces/srv/SetParametersAtomically` | Deshacer: restaura la pose anterior, todo junto | 3 s |

Si el detector no tiene esos servicios (una versión anterior a A2-151), la página avisa y no cambia nada.

## Seguridad: leer antes de abrir los puertos

`rosbridge` **no autentica** y permite a cualquier cliente de la red publicar en cualquier topic (incluido `/cmd/drive`) y llamar servicios. `web_video_server` escucha en todas las interfaces (`0.0.0.0`) y tampoco autentica. Esta página cambia parámetros del detector y la velocidad de crucero, e inicia y frena el auto, pero **cualquiera con acceso a los puertos puede hacer lo mismo y mucho más**.

- Usá una **red solo del equipo**, con contraseña (un router o un hotspot propio), no una Wi-Fi compartida.
- Limitá con **firewall** los puertos 8080 y 9090 de la Jetson a las IP de las computadoras del equipo.
- **Apagá todo** (Ctrl+C en el script) cuando no estés calibrando.
- Con el auto en movimiento, la seguridad sigue en el watchdog del ESP32 y en su frenado por obstáculo (ver [`docs/adr/0003-el-esp32-es-la-autoridad-ante-obstaculos.md`](../../docs/adr/0003-el-esp32-es-la-autoridad-ante-obstaculos.md)), que no dependen de esta página.

## Qué está verificado y qué no

**Verificado en un contenedor de ROS 2 Humble (amd64)** con el detector, `web_video_server` 3.1.0 y `rosbridge_server` 2.0.8 reales y una cámara falsa (un carril proyectado con la geometría del YAML y un cartel blanco afuera del trapecio), con la página abierta en un navegador:

- Las tres imágenes llegan: la cámara (640 x 480), la del detector (640 x 480) y la vista desde arriba (400 x 400).
- La página lee del nodo el trapecio, el color y la región al conectar, y los muestra.
- Al mover un punto del trapecio, el nodo cambia: `ros2 param get /lane_detection warp_src` devuelve el valor nuevo, el log dice "detector actualizado", y las mediciones y las imágenes cambian. Los deslizadores cambian `white_hsv_min`, `white_hsv_max` y `roi_top_fraction` en el nodo.
- Un trapecio cruzado no se envía; el nodo conserva el último válido; y los valores puestos antes se recuperan al recargar la página.
- En la imagen del detector, el cartel blanco aparece en rojo (lo toma por blanco) pero queda afuera del trapecio y no cuenta.

**Verificado en la Jetson real** (Orin, Ubuntu 22.04.5, aarch64, RealSense), antes de esta función: los paquetes existen en arm64, el script compila y lanza todo, la cámara publica en `RELIABLE` a 640 x 480, y la página muestra la imagen y los datos del carril.

**Verificado (A2-157), con Chrome sin interfaz y la página sin conectar:** a 1000 × 600, 1366 × 768, 1440 × 900 y 1920 × 1080 la página no scrollea y el panel derecho entra entero (medido con `scrollHeight`); las imágenes mantienen 4:3 y 1:1 (por ejemplo, 393 × 295 y 277 × 277 a 1366 × 768); la tarjeta Calibración scrollea por dentro (tiene muchos controles). A 500 px de ancho no hay scroll horizontal (Chrome sin interfaz no permite menos de 500).

**Sin verificar (A2-157):** el ancho de un celular real (~400 px), la consola sin errores con la página conectada, y el flujo Calibrar / Deshacer / Guardar con el detector real (o un rosbridge simulado). Lo verificado más abajo es de la versión anterior (con el trapecio arrastrable).

**Sin verificar:**

- Las imágenes de depuración y la calibración en vivo **en la Jetson real**, con la cámara real: el costo en CPU de dibujar las imágenes y que 10 imágenes por segundo no atrasen al detector.
- Cuántos cuadros por segundo entrega la cámara.
- Que con la pista real el trapecio y el color calibrados den buena detección.
- Abrir `index.html` con doble clic (como archivo): la prueba se hizo servida con `python -m http.server`.
- La orientación de la cámara montada en el auto.

## Qué no hace (todavía)

- No guarda la distancia entre ejes ni la de la cámara al eje trasero: van al `params.yaml` del repo con "Copiar".
- No autentica: **cualquiera con acceso al puerto 9090 puede mover o frenar el auto**.
- No mide la velocidad real: el deslizador fija la velocidad que el control pide, que el puente traduce a un duty sin medirla (el auto no tiene encoder).
- No muestra `sign_detection`.

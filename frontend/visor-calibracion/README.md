# Visor de calibración

Una página de un solo archivo (`index.html`) para **calibrar `bfmc_lane_detection` desde una computadora, viendo lo que ve la Jetson**. Muestra la imagen de la cámara, lo que el detector toma por línea, la vista desde arriba con las ventanas y las curvas, y los datos de `/perception/lane`. Deja **cambiar los parámetros del detector en vivo** (el trapecio `warp_src`, el umbral de color y la región de interés) y **manejar el auto**: velocidad, Iniciar y Frenar. No es el dashboard del vehículo (ver [`../README.md`](../README.md)): es una herramienta provisoria de calibración y de mando.

- **Lee, calibra y manda.** Se suscribe a `/perception/lane`, `/hardware/status` y `/vehicle/state`, pide imágenes y llama a servicios: `get_parameters` y `set_parameters` del detector y del control, y `/hardware/stop` y `/hardware/arm` del puente. No publica ningún topic.
- **Sin dependencias.** No usa librerías ni internet: habla el protocolo de `rosbridge` (JSON por WebSocket) directamente. Funciona en una red sin salida a internet.
- **No es un paquete ROS.** `colcon` no la compila ni la testea.

## Qué muestra

| Zona | Qué es |
|---|---|
| **Cámara y trapecio** | La imagen de la cámara con el trapecio de `warp_src` encima. Se arrastran los cuatro puntos y, con "Aplicar en vivo", el detector usa el trapecio mientras se mueve. |
| **Lo que ve el detector** | La imagen de la cámara con lo que el detector toma por **blanco pintado de rojo**, el trapecio que usa (amarillo), el límite de la región de interés (cian), las líneas que encontró (verde la izquierda, azul la derecha) y el error lateral (magenta). Sirve para ver si está tomando por línea algo que no lo es. |
| **Vista desde arriba** | Lo que el detector ve después de "aplanar" el piso: la máscara de blancos, las **ventanas deslizantes** que siguen cada línea, las curvas ajustadas, el auto (amarillo) y la fila donde mide (naranja). |
| **Carril** | `detected`, `lateral_error_m`, `heading_error_rad`, `confidence` e `intersection_detected` de `/perception/lane`, y cuánto hace que llegó el último dato (rojo si pasó más de 1 s). |
| **Mando del auto** | Estado del auto (detenido, en marcha, detenido por un obstáculo a N cm, falla), del enlace con el ESP32 y de la decisión. Un deslizador de **velocidad en cm/s** (0 a 50), **Iniciar**, **Frenar** y **Rearmar después de una falla**. El auto arranca detenido: Iniciar manda primero la velocidad del deslizador y después suelta el freno; Frenar lo detiene hasta el próximo Iniciar. Si el pedido no llega, avisa en rojo. |
| **Ajustes en vivo** | Tres deslizadores (brillo mínimo y saturación máxima del blanco, y cuánto de arriba se descarta) y el texto con las cuatro líneas del `params.yaml`, listas para copiar. |

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

Con el auto apoyado sobre la pista y la cámara derecha:

1. **Trapecio.** Arrastrá los cuatro puntos hasta que el trapecio calce con el carril. El detector lo usa al instante: en "Lo que ve el detector" el trapecio amarillo se mueve, y en la vista desde arriba las dos líneas tienen que quedar **paralelas y verticales**. Si el trapecio es inválido (los puntos se cruzan o quedan alineados), se pone en rojo, no se envía y el detector sigue con el último válido.
2. **Color.** Mirá qué pinta de rojo en "Lo que ve el detector". Si toma por blanco cosas que no son línea (una pared, una hoja), subí el brillo mínimo o bajá la saturación máxima hasta que solo queden las líneas.
3. **Región.** Subí "descartar de arriba" para ignorar el fondo; el límite es la línea cian.
4. **Escala.** Cuando cambia el trapecio también cambian `lane_width_px` y `forward_range_m` (cuántos píxeles de la vista desde arriba son el ancho del carril y la distancia hacia adelante). Se miden en la vista desde arriba y se escriben en el `params.yaml`; la página todavía no los cambia.
5. **Dejarlo fijo.** Lo que se cambia en vivo vale **mientras el nodo siga andando**: no se guarda. Apretá **Copiar**, pegá las cuatro líneas en `ros2_ws/src/bfmc_lane_detection/config/params.yaml` (ya vienen con la sangría que usa el archivo) y reiniciá el script.

**Aplicar en vivo** se puede apagar: entonces la página no manda nada y los cambios quedan solo en el navegador. **Valores del detector** vuelve a los que se leyeron al conectar.

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

**Sin verificar:**

- Las imágenes de depuración y la calibración en vivo **en la Jetson real**, con la cámara real: el costo en CPU de dibujar las imágenes y que 10 imágenes por segundo no atrasen al detector.
- Cuántos cuadros por segundo entrega la cámara.
- Que con la pista real el trapecio y el color calibrados den buena detección.
- Abrir `index.html` con doble clic (como archivo): la prueba se hizo servida con `python -m http.server`.
- La orientación de la cámara montada en el auto.

## Qué no hace (todavía)

- No cambia `lane_width_px`, `forward_range_m` ni el resto de los parámetros: solo el trapecio, el brillo, la saturación y la región.
- No guarda los cambios en el `params.yaml`: hay que copiarlos.
- No autentica: **cualquiera con acceso al puerto 9090 puede mover o frenar el auto**.
- No mide la velocidad real: el deslizador fija la velocidad que el control pide, que el puente traduce a un duty sin medirla (el auto no tiene encoder).
- No muestra `sign_detection`.

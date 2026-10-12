# bfmc_sign_detection

Detecta señales de tránsito con YOLO y estima a qué distancia están. Reutiliza el modelo entrenado del repositorio anterior (AURA-2026). `bfmc_decision` consume su resultado.

## Responsabilidad

Detectar señales y medir su distancia. **No ejecuta maniobras**: no decide qué hacer al ver una señal.

## Topics

| Dirección | Parámetro | Topic | Tipo |
|---|---|---|---|
| Se suscribe | `color_topic` | `/camera/color/image_raw` | `sensor_msgs/Image` (`bgr8` o `rgb8`) |
| Se suscribe | `depth_topic` | `/camera/aligned_depth_to_color/image_raw` | `sensor_msgs/Image` (`16UC1` en milímetros o `32FC1` en metros) |
| Publica | `signs_topic` | `/perception/signs` | `bfmc_interfaces/SignDetectionArray` |

Publica **un mensaje por cada imagen de color, también vacío** cuando no hay señales: así quien lo consume sabe que el detector sigue vivo. El encabezado es el de la imagen de color.

**La profundidad tiene que estar alineada al color**: la distancia se mide en el píxel de la imagen de color donde se detectó la señal. `bfmc_camera` publica esa profundidad. Se usa la última profundidad recibida si no es más vieja que `max_depth_age_s`; si no, la distancia sale como `NaN`.

## Cómo funciona

Para cada detección del modelo:

1. Se descarta si su confianza es menor que `min_confidence`.
2. Se traduce la clase del modelo al tipo de señal del sistema con `class_map`. **Una clase que no está en el mapa se descarta.**
3. La distancia es la **mediana** de los valores de profundidad válidos en una ventana centrada en la caja (`depth_window_fraction`). Un valor es válido si es finito y está entre `min_depth_m` y `max_depth_m`. Con menos de `min_depth_pixels` valores válidos, la distancia es `NaN`.

Las señales se entregan de mayor a menor confianza, con la caja recortada a la imagen.

## El modelo

Es el `best.pt` del repositorio anterior (AURA-2026, commit del 2025-11-21): **YOLOv8m con 13 clases**.

| Clase del modelo | Etiqueta del sistema |
|---|---|
| `stop` | `stop` |
| `priority` | `priority` |
| `roundabout` | `roundabout` |
| `crosswalk` | `crosswalk` |
| `parking` | `parking` |
| `onewayroad` | `one_way` |
| `no_entry` | `no_entry` |
| `highway_entry` / `highway_exit` | `highway_entry` / `highway_exit` |
| `trafficlight` | `traffic_light` |
| `lightred` / `lightyellow` / `lightgreen` | `light_red` / `light_yellow` / `light_green` |

`bfmc_decision` usa hoy `stop`, `priority` y `roundabout`. El mapa vive en `config/params.yaml`: reentrenar o cambiar el modelo no obliga a tocar el código.

**El modelo no está en el repositorio:** pesa 52 MB y quedaría para siempre en el historial de git. Está publicado como asset del release [`modelo-senales-v1`](https://github.com/Bosch-Aura/autonomous-vehicle-BFMC-ros2/releases/tag/modelo-senales-v1), con su checksum SHA-256 en [`config/model.yaml`](config/model.yaml).

```bash
ros2 run bfmc_sign_detection descargar_modelo
```

Lo baja a `model_path` (por defecto `~/.cache/bfmc/senales/best.pt`) **verificando el checksum**: baja a un archivo temporal que solo reemplaza al definitivo si coincide, así un archivo a medias o corrupto nunca queda en su lugar. Si ya está y es correcto, no descarga nada.

## Instalar Ultralytics

El nodo usa [Ultralytics](https://docs.ultralytics.com/) para correr el modelo. No viene con ROS ni se instala con `rosdep`; **se importa recién al arrancar el nodo**, así el paquete y sus tests funcionan sin él.

```bash
pip install ultralytics
```

En la Jetson, PyTorch lo trae JetPack: conviene instalar Ultralytics sin reemplazarlo. Si falta Ultralytics o el modelo, el nodo falla al arrancar con un mensaje que dice cómo resolverlo.

## Parámetros

Viven en [`config/params.yaml`](config/params.yaml). El nodo no tiene valores por defecto en el código: si falta uno, falla al arrancar. Los marcados `[asumido]` son una primera propuesta.

| Parámetro | Unidad | Descripción |
|---|---|---|
| `color_topic`, `depth_topic`, `signs_topic` | — | Topics |
| `model_path` | — | Dónde está el modelo |
| `device` | — | `cuda`, `cpu` o vacío para que Ultralytics elija |
| `yolo_confidence` | 0–1 | Umbral con el que el modelo propone detecciones |
| `class_map` | — | Lista de `clase:etiqueta` |
| `min_confidence` | 0–1 | Umbral con el que una detección se publica |
| `depth_window_fraction` | (0, 1] | Fracción de la caja que se usa para medir |
| `min_depth_m`, `max_depth_m`, `min_depth_pixels` | m, m, px | Qué valores de profundidad son válidos |
| `depth_unit_m` | m | Valor de un paso de la profundidad `16UC1` (0,001 = milímetros) |
| `max_depth_age_s` | s | Antigüedad máxima de la profundidad |

## Cómo probarlo

```bash
docker compose run --rm test
```

Los tests no necesitan el modelo, YOLO ni la cámara:

- La lógica se prueba con un **detector falso**: confianza, mapa de clases, cajas, distancia con profundidad (mediana, valores inválidos, otra resolución).
- El adaptador de YOLO se prueba con un **módulo `ultralytics` falso**.
- La descarga se prueba con un **abridor que imita a GitHub**: checksum correcto, checksum distinto, red caída a mitad de camino, archivo existente.
- La configuración: el mapa cubre las 13 clases del modelo y las etiquetas que usa `bfmc_decision`.
- El nodo, con un detector falso inyectado.

**Que el modelo detecte bien las señales de la pista solo se comprueba con imágenes reales.** Que corra a una velocidad útil, solo en la Jetson.

## Estructura

- `sign_detection_logic.py`: la lógica (filtros, mapa de clases y distancia), sin ROS ni YOLO.
- `yolo_detector.py`: el adaptador de Ultralytics, con importación diferida.
- `model_download.py`: la descarga con verificación de checksum (`descargar_modelo`).
- `image_utils.py`: convierte mensajes de color y de profundidad a arreglos de numpy, sin `cv_bridge`.
- `sign_detection_node.py`: conecta todo con ROS.
- `config/params.yaml`, `config/model.yaml`, `launch/sign_detection.launch.py` y `test/`.

# bfmc_lane_detection

Convierte la imagen de la cámara en el **estado del carril**: cuánto está desviado el auto, hacia dónde apunta el carril, qué tan seguro es el resultado y si hay una intersección adelante. `bfmc_control` y `bfmc_decision` lo consumen.

## Responsabilidad

Medir el carril en cada imagen. **No calcula comandos**: no decide cuánto girar ni si frenar.

## Topics

| Dirección | Parámetro | Topic | Tipo |
|---|---|---|---|
| Se suscribe | `image_topic` | `/camera/color/image_raw` | `sensor_msgs/Image` (`bgr8` o `rgb8`; *best effort*, cola de 1) |
| Publica | `lane_topic` | `/perception/lane` | `bfmc_interfaces/LaneState` |
| Publica | `debug_overlay_topic` | `/perception/lane_debug/overlay` | `sensor_msgs/Image` (`bgr8`): la cámara con lo detectado encima |
| Publica | `debug_birdview_topic` | `/perception/lane_debug/birdview` | `sensor_msgs/Image` (`bgr8`): la vista desde arriba con las ventanas y las curvas |

El encabezado del resultado es el de la imagen, así quien lo consume sabe de qué instante es la medición. Una imagen con una codificación no soportada se descarta con una advertencia.

## Cómo funciona

1. **Umbral de color:** se aíslan las marcas **blancas** (HSV).
2. **Región de interés:** se descarta la parte alta de la imagen.
3. **Vista cenital:** una transformación de perspectiva "aplana" el piso, así las líneas del carril quedan paralelas y las distancias son uniformes.
4. **Ajuste:** se buscan las líneas izquierda y derecha con ventanas deslizantes y se les ajusta un polinomio de segundo grado.
5. **Medición** a una fila de referencia (`reference_row_fraction`).

| Campo de `LaneState` | Significado |
|---|---|
| `lateral_error_m` | Positivo si el centro del carril queda a la **izquierda** del auto |
| `heading_error_rad` | Positivo si el carril sigue hacia la **izquierda** |
| `confidence` | De 0 a 1. Con las dos líneas visibles es mayor que con una; crece con los píxeles de línea hasta `full_confidence_pixels` |
| `detected` | `false` si no se encontró ninguna línea; los demás campos no valen |
| `intersection_detected` | Hay una franja horizontal blanca que cruza la vista cercana al auto |

**Una sola línea visible:** el centro se estima sumando o restando medio ancho de carril. **Dos líneas demasiado juntas o separadas** (fuera de 0,5 a 1,5 veces el ancho esperado) se toman como ruido: se usa la que tiene más píxeles.

## Imágenes de depuración y cambios en vivo

**Qué ve el detector.** Si hay alguien suscripto a las dos imágenes de depuración, el nodo las dibuja y las publica (a lo sumo `debug_max_fps` por segundo). Si nadie las pide, no gasta CPU en dibujarlas. Se apagan del todo con `publish_debug: false`.

- `/perception/lane_debug/overlay`: la imagen de la cámara con lo que el detector toma por **blanco pintado de rojo**, el trapecio de `warp_src` (amarillo), el límite de la región de interés (cian), las curvas encontradas (verde la izquierda, azul la derecha) y el error lateral (magenta). Sirve para ver si toma por línea algo que no lo es.
- `/perception/lane_debug/birdview`: la vista desde arriba con la máscara de blancos, las ventanas deslizantes de cada línea (más gruesas las que encontraron píxeles suficientes), las curvas, el auto (amarillo) y la fila donde se mide (naranja).

Se ven en el navegador con `web_video_server` (por ejemplo, `http://<IP>:8080/stream?type=mjpeg&topic=/perception/lane_debug/overlay`, con las barras del topic sin codificar) o con el visor de calibración (`frontend/visor-calibracion/`). Las imágenes se calculan a partir del mismo análisis que el resultado: **no cambian lo que se publica en `/perception/lane`**.

**Cambiar parámetros con el nodo andando.** Todos los parámetros de la visión se pueden cambiar sin reiniciar, por ejemplo:

```bash
ros2 param set /lane_detection warp_src "[0.30, 0.55, 0.70, 0.55, 1.0, 0.95, 0.0, 1.0]"
ros2 param set /lane_detection roi_top_fraction 0.45
```

El cambio **se valida antes de aplicarse**: un valor inválido (un trapecio cruzado o aplastado, un umbral fuera de rango) se rechaza con el motivo y el detector sigue como estaba. No se guarda en el YAML: al reiniciar el nodo vuelven los valores del archivo. Los topics y los nombres de las imágenes de depuración solo se eligen al arrancar. El nodo corre en un solo hilo, así que un cambio nunca se cruza con una imagen a medio procesar.

## Parámetros

Viven en [`config/params.yaml`](config/params.yaml). El nodo no tiene valores por defecto en el código: si falta uno, falla al arrancar.

**Todos los valores de visión y de geometría son una primera propuesta, sin calibrar.** Dependen de la cámara, de su montaje y de la iluminación. Con la configuración actual el pipeline es correcto en geometría (los tests lo verifican con carriles sintéticos), pero **no se probó con imágenes reales de la pista**.

| Grupo | Parámetros |
|---|---|
| Topics | `image_topic`, `lane_topic` |
| Color | `white_hsv_min`, `white_hsv_max`, `morph_kernel_px` |
| Imagen | `roi_top_fraction`, `warp_src`, `warp_width`, `warp_height` |
| Escala | `lane_width_m`, `lane_width_px`, `forward_range_m` |
| Líneas | `n_windows`, `window_margin_px`, `min_pixels` |
| Medición | `reference_row_fraction`, `full_confidence_pixels` |
| Intersección | `intersection_band_top_fraction`, `intersection_row_fraction`, `intersection_min_rows` |
| Depuración | `publish_debug`, `debug_max_fps`, `debug_overlay_topic`, `debug_birdview_topic` |

## Cómo calibrarlo con la cámara real

La forma más cómoda es el **visor de calibración** (`frontend/visor-calibracion/`): se arrastra el trapecio sobre la imagen de la cámara y el detector lo usa al instante, y se ve en la imagen de depuración qué toma por línea. Lo que sigue es el procedimiento a mano.

1. **Guardá un cuadro** de la cámara con el auto en el carril, bien centrado y derecho.
2. **`warp_src`:** en ese cuadro ubicá cuatro puntos del piso que formen un rectángulo en la realidad (por ejemplo, dos tramos de línea del carril, uno cerca y otro lejos). Anotá sus posiciones como fracción del ancho y del alto, en el orden arriba-izquierda, arriba-derecha, abajo-derecha, abajo-izquierda.
3. **`lane_width_px`:** medí en la vista cenital resultante el ancho en píxeles entre las dos líneas, y poné el ancho real en `lane_width_m`.
4. **`forward_range_m`:** la distancia real hacia adelante que cubre la vista cenital.
5. **Color:** si las líneas no se aíslan, ajustá `white_hsv_min` y `white_hsv_max` con un cuadro de la pista.
6. **Verificá el signo:** con el auto corrido a la izquierda del carril, `lateral_error_m` debe dar **negativo**.

## Cómo probarlo

```bash
docker compose run --rm test
```

Los tests no necesitan la cámara ni la pista:

- **Carriles sintéticos de posición conocida** con una vista cenital identidad: miden desvío y rumbo con el signo correcto, una sola línea, confianza, ausencia de carril, ruido, líneas no blancas y la intersección.
- **Con la geometría real del YAML:** se dibuja un carril en vista cenital, se lo proyecta a la imagen de la cámara con la inversa de la misma transformación y se comprueba que el pipeline recupere el desvío y el rumbo. Si alguien rompe la geometría del YAML, estos tests fallan.
- **El nodo**, con imágenes publicadas por un nodo auxiliar.

Que la configuración sea la correcta para la cámara y la iluminación reales solo se sabe con imágenes de la pista.

## Estructura

- `lane_detection_logic.py`: el pipeline, sin ROS. Cada imagen se procesa sola. `detect` devuelve el resultado; `analizar` devuelve además lo que el detector vio (máscara, ventanas, curvas).
- `lane_debug.py`: dibuja las dos imágenes de depuración a partir de ese análisis, sin ROS.
- `image_utils.py`: convierte un mensaje de imagen a un arreglo BGR, sin `cv_bridge`.
- `lane_detection_node.py`: conecta la imagen con la lógica.
- `config/params.yaml`, `launch/lane_detection.launch.py` y `test/`.

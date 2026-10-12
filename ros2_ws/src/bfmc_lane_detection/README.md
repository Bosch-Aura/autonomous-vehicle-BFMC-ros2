# bfmc_lane_detection

Convierte la imagen de la cámara en el **estado del carril**: cuánto está desviado el auto, hacia dónde apunta el carril, qué tan seguro es el resultado y si hay una intersección adelante. `bfmc_control` y `bfmc_decision` lo consumen.

## Responsabilidad

Medir el carril en cada imagen. **No calcula comandos**: no decide cuánto girar ni si frenar.

## Topics

| Dirección | Parámetro | Topic | Tipo |
|---|---|---|---|
| Se suscribe | `image_topic` | `/camera/color/image_raw` | `sensor_msgs/Image` (`bgr8` o `rgb8`; *best effort*, cola de 1) |
| Publica | `lane_topic` | `/perception/lane` | `bfmc_interfaces/LaneState` |

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

## Cómo calibrarlo con la cámara real

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

- `lane_detection_logic.py`: el pipeline, sin ROS. Cada imagen se procesa sola.
- `image_utils.py`: convierte un mensaje de imagen a un arreglo BGR, sin `cv_bridge`.
- `lane_detection_node.py`: conecta la imagen con la lógica.
- `config/params.yaml`, `launch/lane_detection.launch.py` y `test/`.

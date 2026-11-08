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
| Servicio | `calibrate_service` | `/lane_detection/calibrate_ground` | `std_srvs/Trigger`: calibra la vista cenital con la profundidad (ver "Cómo calibrarlo") |
| Servicio | `reset_service` | `/lane_detection/reset_params` | `std_srvs/Trigger`: vuelve la parte del piso, el color, la región y el ancho del carril a los valores del `params.yaml` del paquete, en vivo (ver "Predeterminados") |
| Se suscribe (solo al calibrar) | `depth_topic` | `/camera/aligned_depth_to_color/image_raw` | `sensor_msgs/Image` (`16UC1` en mm o `32FC1` en m; *best effort*) |
| Se suscribe (solo al calibrar) | `depth_info_topic` | `/camera/aligned_depth_to_color/camera_info` | `sensor_msgs/CameraInfo`: los intrínsecos de esa profundidad |

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
| `curvature` | [1/m] del centro del carril; positiva si dobla hacia la **izquierda** |
| `reference_distance_m` | Dónde se midieron desvío, rumbo y curvatura: distancia desde el pie de la cámara (`calib_forward_near_m + reference_row_fraction × largo de la vista`). No es el auto |
| `mode` | `MODE_TWO_LINES`, `MODE_ONE_LINE`, `MODE_PREDICTED` o `MODE_LOST` (ver "Chequeos y fallback") |
| `left_valid`, `right_valid` | Si cada línea pasó sus chequeos en esta imagen |
| `confidence` | De 0 a 1. Con las dos líneas es mayor que con una; crece con los píxeles de línea hasta `full_confidence_pixels`; en `MODE_PREDICTED` baja hasta 0 |
| `detected` | `false` **solo** en `MODE_LOST`; entonces el desvío y el rumbo son NaN y la confianza 0 |
| `intersection_detected` | Hay una franja horizontal blanca que cruza la vista cercana al auto |

## Chequeos y fallback (A2-152)

**Chequeos de cada imagen** (`lane_detection_logic.py`, sin memoria). Una línea se descarta si:

- tiene menos de `min_pixels` píxeles o menos de `min_windows` ventanas con píxeles;
- el polinomio ajusta mal: error medio mayor a `max_residual_m`;
- es más curva que la curva más cerrada de la pista (`min_curve_radius_m`).

Si las dos pasan pero no están a `lane_width_m` ± `lane_width_tolerance_m` entre sí en todas las alturas de `width_check_fractions`, se descarta la que **ajusta peor** (mayor residuo; a igual residuo, la de menos píxeles): una mancha de reflejo puede tener muchos píxeles, pero no forma de línea. El ancho se mide **perpendicular** a las líneas: en una curva cruzan la fila inclinadas, y en horizontal un carril de 0,37 m inclinado 30° mide 0,43 m. No hay un chequeo de paralelismo aparte: el ancho en varias alturas ya lo acota.

**Fallback escalonado** (`lane_tracker.py`, con memoria y el tiempo de la imagen):

| Modo | Cuándo | Qué se publica |
|---|---|---|
| `MODE_TWO_LINES` | Las dos líneas válidas | Centro = promedio de las dos |
| `MODE_ONE_LINE` | Una válida | Centro = esa línea ± medio `lane_width_m` |
| `MODE_PREDICTED` | Ninguna, hace menos de `lost_timeout_s` | El último estado válido; la confianza baja en línea recta hasta 0 |
| `MODE_LOST` | Ninguna por más de `lost_timeout_s` | `detected = false`, NaN, confianza 0: `bfmc_decision` frena |

Es el patrón común de degradación gradual: seguir con una estimación por un tiempo acotado (el error de seguir a ciegas crece rápido) y después una parada controlada. Cuando vuelven las líneas se recupera solo. La memoria está aparte del detector: cambiar un parámetro en vivo no la borra.

**Búsqueda alrededor del polinomio previo (A2-153).** `LaneTracker` guarda el polinomio de cada línea válida y se lo pasa al detector en el cuadro siguiente. Con previo, el detector toma solo los píxeles a menos de `search_margin_m` de él (`prior_search`): un reflejo o una mancha lejos de la línea no la confunde, y es más barato que las ventanas. Vuelve a las ventanas deslizantes en toda la mitad de la imagen (`sliding_windows`), **por línea**:

- si no hay polinomio previo (al arrancar);
- si lo encontrado cerca no pasa los chequeos de la línea (en el mismo cuadro, sin perderlo);
- si la línea lleva más de `lost_timeout_s` sin ser válida, o si el chequeo de ancho la descartó (lo encontrado no era la línea: así no se queda "enganchada" a otra);
- si se rehace el detector por un cambio de parámetros (el polinomio está en píxeles de la vista vieja).

**Filtro de Kalman (A2-154, `kalman_enabled`).** Con el filtro activo, lo publicado es el estado filtrado `[desvío, rumbo, curvatura]` en la fila de medición (`lane_kalman.py`). Predice con el avance del auto (`d' = d + v·dt·ψ`, `ψ' = ψ + v·dt·κ`) y corrige con cada medición, con más peso con dos líneas (`kalman_r_two_lines`) que con una (`kalman_r_one_line`). En `MODE_PREDICTED` y `MODE_LOST` solo predice, y la confianza baja con la incertidumbre del desvío (`kalman_max_std_m`). Una medición fuera del 99 % (`kalman_gate_chi2`) se descarta: un reflejo o un salto no llega a lo publicado.

- **Velocidad:** la comandada (`/cmd/drive`); no hay odometría. Sin comandos, 0: el estado se mantiene y su incertidumbre crece.
- **Se reinicia** al salir de `MODE_LOST` con dos líneas, al rehacer el detector y si descarta mediciones seguidas por más de `lost_timeout_s` (la realidad cambió de golpe: sin reiniciar quedaría aferrado a su estimación vieja).
- **No se resta el giro propio del auto** (necesitaría la distancia entre ejes): lo absorbe Q. Si en la pista se atrasa en las curvas, es la primera mejora a probar.
- **Está apagado por defecto** hasta medir R y Q en la pista. Con el filtro activo, conviene `filter_alpha: 1.0` en `bfmc_control`, para no filtrar dos veces.
- **Cómo medir R:** con el auto quieto en una recta, la varianza de lo medido durante unos segundos (repetir en una curva). **Q:** arrancar chico y subir hasta que el filtrado siga las curvas sin atrasarse en el visor.

En la imagen de depuración se ven el modo, qué chequeo falló, el método de búsqueda de cada línea y, con el filtro, el estado filtrado junto al medido (y su centro, marcado en blanco). Para analizar una prueba en pista: `ros2 bag record /perception/lane`.

**Tests con capturas reales:** guardá cuadros de la pista como PNG en `test/capturas/`, con el modo esperado al principio del nombre (`dos_lineas__recta_01.png`, `una_linea__reflejo_02.png`, `perdido__tapada_01.png`). `test_capturas.py` los corre todos; sin capturas, se saltea.

## Imágenes de depuración y cambios en vivo

**Qué ve el detector.** Si hay alguien suscripto a las dos imágenes de depuración, el nodo las dibuja y las publica (a lo sumo `debug_max_fps` por segundo). Si nadie las pide, no gasta CPU en dibujarlas. Se apagan del todo con `publish_debug: false`.

- `/perception/lane_debug/overlay`: la imagen de la cámara con lo que el detector toma por **blanco pintado de rojo**, el trapecio de la vista cenital (amarillo), el límite de la región de interés (cian), las curvas encontradas (verde la izquierda, azul la derecha) y el error lateral (magenta). Sirve para ver si toma por línea algo que no lo es.
- `/perception/lane_debug/birdview`: la vista desde arriba con la máscara de blancos, las ventanas deslizantes de cada línea (más gruesas las que encontraron píxeles suficientes), las curvas, el auto (amarillo) y la fila donde se mide (naranja).

Se ven en el navegador con `web_video_server` (por ejemplo, `http://<IP>:8080/stream?type=mjpeg&topic=/perception/lane_debug/overlay`, con las barras del topic sin codificar) o con el visor de calibración (`frontend/visor-calibracion/`). Las imágenes se calculan a partir del mismo análisis que el resultado: **no cambian lo que se publica en `/perception/lane`**.

**Cambiar parámetros con el nodo andando.** Todos los parámetros de la visión se pueden cambiar sin reiniciar, por ejemplo:

```bash
ros2 param set /lane_detection calib_forward_near_m 0.50
ros2 param set /lane_detection roi_top_fraction 0.25
```

El cambio **se valida antes de aplicarse**: un valor inválido (un rectángulo cuyo borde lejano no entra en la imagen, un umbral fuera de rango) se rechaza con el motivo y el detector sigue como estaba. **Con el auto andando** (ESP32 en `RUNNING` según `/hardware/status`) se rechazan los cambios de geometría y la calibración: cambiarían la escala en plena marcha. El ancho del carril sí se puede cambiar. Sin el puente lanzado no hay estado y el auto no se puede mover, así que se permiten. Lo cambiado vale hasta reiniciar; para que quede, **Guardar** (ver abajo). Los topics, el nombre del servicio y los de las imágenes de depuración solo se eligen al arrancar. Un cambio arma un detector nuevo y lo reemplaza de una vez: la imagen que se está procesando termina con el detector con el que empezó, nunca con una mezcla.

**Hilos.** El nodo corre con un `MultiThreadedExecutor`: el servicio de calibración espera los cuadros de profundidad, y con un solo hilo esa espera trabaría al propio callback que los recibe. Las imágenes de color, la profundidad y la calibración tienen cada una su grupo; los servicios de parámetros (`get_parameters`, `set_parameters`...) y los topics chicos quedan en el grupo por defecto. Así una imagen lenta no deja sin respuesta a los parámetros: si compartieran grupo, siempre habría una imagen nueva esperando y el executor la elegiría antes, y el visor diría "el nodo no respondió". Como un cambio de parámetros puede llegar mientras se procesa una imagen, el tracker y el cambio de detector van con un candado; si el detector cambió durante una imagen, al terminarla se olvidan las líneas previas (están en píxeles de la vista vieja).

## Parámetros

Viven en [`config/params.yaml`](config/params.yaml). El nodo no tiene valores por defecto en el código: si falta uno, falla al arrancar.

**Todos los valores de visión y de geometría son una primera propuesta, sin calibrar.** Dependen de la cámara, de su montaje y de la iluminación. Con la configuración actual el pipeline es correcto en geometría (los tests lo verifican con carriles sintéticos), pero **no se probó con imágenes reales de la pista**.

| Grupo | Parámetros |
|---|---|
| Topics | `image_topic`, `lane_topic` |
| Color | `white_hsv_min`, `white_hsv_max`, `morph_kernel_px` |
| Imagen | `roi_top_fraction`, `warp_width`, `warp_height` |
| Pista | `lane_width_m` (no cambia la escala) |
| Geometría | `calib_height_m`, `calib_pitch_deg`, `calib_roll_deg`, `camera_intrinsics`, `calib_forward_near_m`, `calib_forward_far_m`, `calib_half_width_m` |
| Guardar | `save_service`, `persist_file` |
| Predeterminados | `reset_service` |
| Estado del auto | `hardware_status_topic` |
| Líneas | `n_windows`, `window_margin_px`, `min_pixels` |
| Chequeos (A2-152) | `min_windows`, `max_residual_m`, `min_curve_radius_m`, `lane_width_tolerance_m`, `width_check_fractions` |
| Fallback y búsqueda (A2-152, A2-153) | `lost_timeout_s`, `search_margin_m` |
| Medición | `reference_row_fraction`, `full_confidence_pixels` |
| Intersección | `intersection_band_top_fraction`, `intersection_row_fraction`, `intersection_min_rows` |
| Depuración | `publish_debug`, `debug_max_fps`, `debug_overlay_topic`, `debug_birdview_topic` |
| Calibración | `calibrate_service`, `depth_topic`, `depth_info_topic`, `calib_frames`, `calib_timeout_s` |
| Puntos del piso | `calib_depth_min_m`, `calib_depth_max_m`, `calib_subsample`, `calib_min_points` |
| Plano (RANSAC) | `calib_ransac_iters`, `calib_ransac_seed`, `calib_inlier_m`, `calib_min_inlier_fraction`, `calib_height_range_m`, `calib_pitch_range_deg` |
| Chequeo al arrancar | `calib_check_on_start`, `calib_check_frames`, `calib_check_height_tol_m`, `calib_check_pitch_tol_deg` |

## Cómo calibrarlo con la cámara real

El **visor de calibración** (`frontend/visor-calibracion/`) muestra en vivo qué toma el detector por línea.

### Geometría: calibración con la profundidad

**El trapecio y la escala no son parámetros: se calculan.** Salen de tres datos independientes:

| Dato | Qué describe | De dónde sale |
|---|---|---|
| Pose de la cámara (`calib_height_m`, `calib_pitch_deg`, `calib_roll_deg`) | dónde está la cámara | la calibración con la profundidad |
| `camera_intrinsics` | la lente | el `camera_info`, lo copia la calibración |
| Rectángulo del piso (`calib_forward_near_m`, `calib_forward_far_m`, `calib_half_width_m`) | qué parte del piso mostrar | se elige (en metros) |

El trapecio es ese rectángulo visto por la cámara, y la escala es la del rectángulo (0,50 m en 400 px: 1 px = 1,25 mm). Como no hay un número de escala aparte, **no se puede desincronizar del trapecio**. Cambiar el rectángulo vuelve a proyectar el trapecio al instante, sin medir de nuevo: es lo que reemplaza a arrastrar el trapecio a mano.

`lane_width_m` es un dato **de la pista**, no de la cámara: no cambia la escala. Lo usan el chequeo de separación entre líneas y el modo "una sola línea" (el carril en píxeles se calcula como `lane_width_m` / escala). Corregirlo nunca exige recalibrar.

La calibración mide la pose:

1. Con la profundidad alineada al color (promedio de `calib_frames` cuadros) arma los puntos 3D de la mitad de abajo de la imagen, entre `calib_depth_min_m` y `calib_depth_max_m`.
2. Encuentra el plano del piso con RANSAC (semilla fija) y lo reajusta con los inliers. De ahí salen la **altura** de la cámara, su **pitch** y su **roll**.
3. Aplica la pose y los intrínsecos medidos: el nodo vuelve a proyectar el rectángulo y arma el detector.

**Cómo se hace.** Una vez, con el auto **quieto**, la cámara montada y la zona delante del auto **despejada** (sin objetos ni paredes a menos de 1,5 m). Se repite solo si la cámara se mueve.

```bash
ros2 service call /lane_detection/calibrate_ground std_srvs/srv/Trigger
```

O con el botón **Calibrar** del visor (A2-157). Tarda menos de un segundo después de recibir los cuadros (espera hasta `calib_timeout_s`).

- **Si sale bien:** el detector la usa al instante y la respuesta dice lo que midió (`altura 0.205 m, pitch 20.1°, roll 0.4°, inliers 93%, 1 px = 1.25 mm`). Para que quede al reiniciar, **Guardar**.
- **Si falla, no cambia nada** y la respuesta dice por qué y qué hacer: no llegó la profundidad (¿`align_depth` activado?), hay algo delante de la cámara (pocos inliers), la altura o la inclinación no tienen sentido (una pared tomada por piso), o el borde lejano del rectángulo no entra en la imagen (achicá `calib_half_width_m` o bajá `calib_forward_far_m`).
- Si el trapecio empieza más arriba que `roi_top_fraction`, la respuesta lo avisa: esa parte de la vista quedaría tapada.

**Chequeo al arrancar.** Con `calib_check_on_start`, el nodo mide la cámara con `calib_check_frames` cuadros y, si la altura o el pitch difieren de `calib_height_m` y `calib_pitch_deg` más que la tolerancia (1 cm o 1°), loguea un WARN: la cámara se movió y hay que recalibrar. No corrige nada solo y no frena el arranque.

**Lo que la calibración asume y no puede medir:**

- **Que la cámara mira hacia adelante del auto y está centrada en él.** La profundidad no ve el giro alrededor del eje vertical (yaw) ni un corrimiento lateral de la cámara. Si la cámara está girada 2°, a 0,5 m el desvío sale corrido unos 2 cm.
- **El borde de abajo de la vista cenital está a `calib_forward_near_m` (0,45 m) del pie de la cámara, no pegado al auto.** La fila de medición (`reference_row_fraction: 0.25`) queda entonces a 0,45 + 0,25 × 0,5 = **0,575 m** delante de la cámara: `lateral_error_m` es el desvío del carril a esa distancia, no a la altura del auto. En la vista desde arriba, la línea amarilla es el **eje** del auto, no su posición.
- **`lane_width_m` es un valor a medir con cinta:** de centro a centro de las dos líneas, en una recta, en 3 lugares, y promediar.

**Seguridad.** El servicio no pide autenticación: cualquiera en la red de ROS (incluido el visor por rosbridge) lo puede llamar. Con el ESP32 en `RUNNING` el nodo la rechaza, y las validaciones rechazan una calibración absurda (pared, obstáculo, altura o inclinación imposibles). Lo que queda: sin el puente lanzado el nodo no sabe si el auto se mueve (aunque sin puente no se mueve), y la red sigue abierta. Calibrá con el auto quieto y en una red solo del equipo.

### Guardar: que quede al reiniciar

```bash
ros2 service call /lane_detection/save_params std_srvs/srv/Trigger
```

O el botón **Guardar** del visor. Escribe en `persist_file` (`~/.config/bfmc/lane_detection.yaml`) **solo** estas medidas y ajustes: `lane_width_m`, la geometría (pose, intrínsecos y rectángulo) y el color y la región (`white_hsv_min`, `white_hsv_max`, `roi_top_fraction`). Los launch (`lane_detection.launch.py` y `bfmc_bringup/system.launch.py`) cargan ese archivo **después** del `params.yaml`, así le gana.

**Predeterminados.** `ros2 service call /lane_detection/reset_params std_srvs/srv/Trigger`, o el botón del visor, vuelve en vivo `white_hsv_min`, `white_hsv_max`, `roi_top_fraction`, el rectángulo (`calib_forward_near_m`, `calib_forward_far_m`, `calib_half_width_m`) y `lane_width_m` a los valores del `params.yaml` del paquete, todos juntos y con la misma validación (con el auto andando se rechaza). **No** toca la pose ni los intrínsecos, que son medidas de la cámara real (para eso, Calibrar), ni el archivo guardado: para que quede al reiniciar, Guardar.

- **Por qué fuera del repo:** con `--symlink-install`, el `params.yaml` instalado es un enlace al del repo. Escribirlo dejaría cambios sin commitear en la Jetson y conflictos en cada `git pull`.
- **Escritura atómica:** se escribe un temporal y se lo renombra. Un corte de luz deja el archivo anterior entero, nunca uno a medias.
- **Para volver a los valores del repo:** borrá `~/.config/bfmc/lane_detection.yaml`.
- **Cuidado:** si se cambia el `params.yaml` del repo (por ejemplo, otro `lane_width_m` medido por el equipo), lo guardado en la Jetson le sigue ganando hasta que se borre o se vuelva a guardar.

**Cerca del auto la cámara ve poco de ancho.** Con la cámara nominal (0,20 m, 20°), las esquinas cercanas del rectángulo de ±0,25 m a 0,45 m caen a 1 % del borde de la imagen. Si la cámara real está más baja o menos inclinada, o si se baja `calib_forward_near_m`, las esquinas cercanas quedan fuera de la imagen: el trapecio va igual donde dice el parámetro y esa franja de la vista cenital sale negra (sin datos); el nodo lo avisa en el log y en la respuesta de la calibración. Con la cámara medida en el auto (0,268 m, 23,6°) y ±0,25 m, las esquinas cercanas entran recién desde 0,44 m. El borde lejano sí tiene que entrar: si no, se rechaza.

### Color y verificación

1. **Color:** si las líneas no se aíslan, ajustá `white_hsv_min` y `white_hsv_max` con un cuadro de la pista.
2. **Verificá el signo:** con el auto corrido a la izquierda del carril, `lateral_error_m` debe dar **negativo**.
3. **Verificá la escala:** con el auto corrido una distancia conocida (por ejemplo 5 cm) del centro del carril, `lateral_error_m` debe dar esa distancia.

## Cómo probarlo

```bash
docker compose run --rm test
```

Los tests no necesitan la cámara ni la pista:

- **Carriles sintéticos de posición conocida** con una vista cenital identidad: miden desvío y rumbo con el signo correcto, una sola línea, confianza, ausencia de carril, ruido, líneas no blancas y la intersección.
- **Con la geometría real del YAML:** se dibuja un carril en vista cenital, se lo proyecta a la imagen de la cámara con la inversa de la misma transformación y se comprueba que el pipeline recupere el desvío y el rumbo. Si alguien rompe la geometría del YAML, estos tests fallan.
- **La geometría:** cambiar el ancho del carril no cambia la escala ni la medición; cambiar el rectángulo reproyecta el trapecio y un punto del piso sigue cayendo donde corresponde; pitch y roll reconstruyen el mismo plano.
- **La calibración**, con la profundidad sintética de un piso de altura e inclinación conocidas (`test/synthetic_depth.py`), con 3 mm de ruido y un 20 % de puntos de un obstáculo: recupera la altura con error menor a 5 mm y el pitch con error menor a 0,5°; un punto del piso conocido cae en la vista cenital donde predice la escala; el detector calibrado mide el desvío real; y una pared, un piso tapado o un rectángulo que no entra se rechazan con su motivo.
- **El nodo**, con imágenes publicadas por un nodo auxiliar; el servicio de calibración con profundidad publicada (aplica la pose; con el piso tapado no cambia nada); el bloqueo con el auto andando; y Guardar (solo la lista fija, fuera del repo, escritura atómica).

Que la configuración sea la correcta para la cámara y la iluminación reales solo se sabe con imágenes de la pista.

## Estructura

- `lane_detection_logic.py`: el pipeline, sin ROS. Cada imagen se procesa sola. `detect` devuelve el resultado; `analizar` devuelve además lo que el detector vio (máscara, ventanas, curvas).
- `lane_debug.py`: dibuja las dos imágenes de depuración a partir de ese análisis, sin ROS.
- `image_utils.py`: convierte un mensaje de imagen a un arreglo BGR, y una profundidad a metros, sin `cv_bridge`.
- `ground_calibration.py`: mide la pose de la cámara con la profundidad (plano del piso con RANSAC) y calcula el trapecio y la escala con la pose, los intrínsecos y el rectángulo, sin ROS.
- `param_store.py`: guarda y lee los parámetros guardados fuera del repo, sin ROS.
- `lane_detection_node.py`: conecta la imagen con la lógica.
- `config/params.yaml`, `launch/lane_detection.launch.py` y `test/`.

# Tickets: lo implementado y lo pendiente

Registro de cómo quedó cada ticket en el código, para actualizar su descripción en Jira.
Cada ticket dice qué se hizo, en qué se apartó del ticket original y por qué, y qué
falta. Se actualiza con cada ticket nuevo.

Rama: `prueba-jetson`.

---

## A2-151: calibración de la vista cenital con la profundidad

Commit `705f806`.

### Descripción actualizada (para Jira)

La escala de la vista cenital ya no se carga a mano. Antes, `warp_src` (el trapecio) y
la escala (`lane_width_px`, `forward_range_m`) eran parámetros independientes: si se
movía el trapecio, los metros salían con la escala vieja. Ahora **`warp_src` y la escala
no son parámetros: se calculan** con tres datos independientes:

| Dato | De dónde sale |
|---|---|
| Pose de la cámara: `calib_height_m`, `calib_pitch_deg`, `calib_roll_deg` | La calibración con la profundidad |
| `camera_intrinsics` (fx, fy, cx, cy, ancho, alto) | El `camera_info`; lo copia la calibración |
| Rectángulo del piso: `calib_forward_near_m`, `calib_forward_far_m`, `calib_half_width_m` | Se elige, en metros |

- **Servicio `/lane_detection/calibrate_ground`** (`std_srvs/Trigger`): se suscribe a la
  profundidad alineada solo mientras calibra, promedia `calib_frames` cuadros, encuentra
  el piso con RANSAC (semilla fija) y aplica la pose medida. Responde, por ejemplo,
  `altura 0.205 m, pitch 20.1°, roll 0.4°, inliers 93%, 1 px = 1.25 mm`.
- **Rechaza con el motivo y qué hacer:** sin profundidad, obstáculo delante, altura o
  inclinación sin sentido (una pared también es un plano), rectángulo fuera de la imagen.
- **Con el ESP32 en `RUNNING` rechaza** la calibración y los cambios de geometría.
- **Servicio `/lane_detection/save_params`:** guarda las medidas y la geometría en
  `~/.config/bfmc/lane_detection.yaml`, fuera del repo, con escritura atómica. Los launch
  lo cargan después del `params.yaml`.
- **Chequeo al arrancar:** si la altura o el pitch difieren de la última calibración más
  de 1 cm o 1°, avisa que la cámara se movió.
- El nodo pasa a `MultiThreadedExecutor`: con un solo hilo, el servicio que espera la
  profundidad trabaría al callback que la recibe.

### Diferencias con el ticket original, y por qué

| Ticket original | Cómo quedó | Por qué |
|---|---|---|
| El servicio escribe `warp_src`, `lane_width_px` y `forward_range_m` | Esos tres dejaron de ser parámetros; se calculan | Así no se pueden desincronizar: no hay ningún número de escala aparte |
| `lane_width_m` interviene en la escala | La escala sale solo de la calibración; `lane_width_m` es un dato de la pista | Con la calibración, `lane_width_m` se cancela en la cuenta. Corregirlo no exige recalibrar |
| Arrastrar el trapecio en el visor | Se elige el rectángulo en metros y el trapecio se reproyecta con la pose medida | Arrastrar el trapecio a mano desataba la escala |
| Roll = `asin(n_x)` | Roll = `atan2(n_x, -n_y)` | Con la fórmula original, guardar pitch y roll no reconstruía el mismo plano (2,82° en vez de 3°) |
| No escribe el `params.yaml`; se copia desde el visor | Botón/servicio Guardar a un archivo fuera del repo | Escribir el `params.yaml` instalado (enlace al del repo) ensucia el repo en la Jetson |
| Validaciones: inliers, altura, rectángulo | + rango de inclinación (`calib_pitch_range_deg`) | Una pared pasaba el chequeo de altura |
| — | Rechazo con el auto andando | Calibrar en marcha cambiaría la escala en plena conducción |

### Pendiente

- [ ] **Medir con cinta** el ancho real del carril (centro a centro) y cargarlo en `lane_width_m`.
- [ ] **Medir la distancia entre ejes** del auto (para A2-155).
- [ ] **Calibrar en la pista** y comprobar el error con cinta cada 10 cm: menor a 1 cm a 0,5 m y a 2 cm a 1 m. Adjuntar captura del visor al ticket.
- [ ] Verificar en la Jetson que la cámara real entra en el rectángulo por defecto (±0,25 m a 0,45 m): con la cámara nominal, las esquinas cercanas caen al 1 % del borde.
- [ ] Botón Calibrar y Guardar en el visor (A2-157). Hasta entonces, arrastrar el trapecio en el visor da error (el parámetro `warp_src` ya no existe).
- [ ] Correr la suite completa (`docker compose run --rm test`). Se corrieron `bfmc_lane_detection` y `bfmc_bringup` por separado; los tests de arranque del sistema fallaron solo por falta de dependencias en el contenedor `dev`.

---

## A2-152: chequeos de cada medición y fallback escalonado

Commit `32a97f2`.

### Descripción actualizada (para Jira)

Cada medición del carril se verifica antes de creerle y, si falla, el estado baja de
escalón en vez de seguir confiado.

**Chequeos por imagen** (`lane_detection_logic.py`, sin memoria). Una línea se descarta si:

- tiene menos de `min_pixels` píxeles o menos de `min_windows` ventanas con píxeles;
- el polinomio ajusta mal: error medio mayor a `max_residual_m`;
- es más curva que la curva más cerrada de la pista (`min_curve_radius_m`).

Si las dos pasan pero no están a `lane_width_m` ± `lane_width_tolerance_m` en todas las
alturas de `width_check_fractions`, se descarta la que **ajusta peor** (mayor residuo; a
igual residuo, la de menos píxeles). El ancho se mide **perpendicular** a las líneas.

**Fallback escalonado** (`lane_tracker.py`, con memoria y el tiempo de la imagen):

| Modo | Cuándo |
|---|---|
| `MODE_TWO_LINES` | Las dos líneas válidas |
| `MODE_ONE_LINE` | Una válida |
| `MODE_PREDICTED` | Ninguna, hace menos de `lost_timeout_s` (0,3 s): se repite el último estado y la confianza baja hasta 0 |
| `MODE_LOST` | Ninguna por más de `lost_timeout_s`: `detected = false`, NaN, confianza 0 |

- **`LaneState`:** `mode`, `left_valid`, `right_valid`. `detected` es `false` solo en `MODE_LOST`.
- **`bfmc_decision`:** con el carril perdido y sin maniobra en curso, el auto frena. Cuando vuelve el carril, sigue solo.
- **Depuración:** el modo y el chequeo que falló aparecen en la imagen.
- **Tests con capturas reales:** los PNG en `test/capturas/`, con el modo esperado en el nombre (`dos_lineas__…`, `una_linea__…`, `perdido__…`).

### Diferencias con el ticket original, y por qué

| Ticket original | Cómo quedó | Por qué |
|---|---|---|
| Ancho de carril en 2 o 3 alturas | Medido **perpendicular** a las líneas | En horizontal, un carril de 0,37 m inclinado 30° mide 0,43 m: se rechazarían las curvas cerradas |
| Chequeo de paralelismo | No se agregó | El ancho en varias alturas ya lo acota; sería un umbral redundante |
| Desempate: "más píxeles y menor residuo" | Primero menor residuo, después más píxeles | El ticket no dice qué pasa si cada línea gana en una cosa; un reflejo tiene muchos píxeles pero ajusta mal |
| Polinomios guardados en el detector | Memoria en una clase aparte (`LaneTracker`) | El nodo rehace el detector en cada cambio de parámetro: la memoria se borraba |
| Bajar la confianza en PREDICTED | Baja en línea recta hasta 0 al cumplirse el plazo | Una fórmula sin parámetro nuevo, sin saltos |
| `lost_timeout_ms` | `lost_timeout_s` | Unidades SI, como el resto del repo |
| Watchdog del tópico en `bfmc_decision` | No se agregó | Ya existe: `lane_cmd_timeout_s` frena si deja de llegar el carril |
| Con LOST, maniobra si el mapa indica intersección | No se agregó | No hay ningún mapa en el repo |
| Log CSV | No se agregó | `ros2 bag record /perception/lane` graba lo mismo sin código |
| Frenar con LOST (contradecía A2-142: "sin carril, seguir") | Corte breve: sigue; más de 0,3 s: frena; vuelve el carril: sigue solo | Es el patrón común de degradación gradual (UN R157, *minimal risk manoeuvre*): estimación acotada y después parada controlada. En BFMC, salirse cuesta 2,0 puntos y reposicionar 2,5; detenerse no se penaliza |

### Pendiente

- [ ] **Correr los tests** de `bfmc_interfaces`, `bfmc_lane_detection` y `bfmc_decision` (no se corrieron). Los tests del reflejo, el desempate y la curvatura usan dibujos con umbrales calculados a mano: si fallan, revisar primero el dibujo.
- [ ] **Avisar a quien mantiene `bfmc_control` y `bfmc_decision`** del cambio de interfaz de `LaneState`, y a quien decidió A2-142 (AnaB2) del cambio de comportamiento.
- [ ] **Medir el radio de la curva más cerrada** de la pista y cargarlo en `min_curve_radius_m` (hoy 0,4 m sin medir).
- [ ] **Afinar los umbrales en la pista** con el protocolo de zonas (recta, curva cerrada, intersección, rotonda, punteadas).
- [ ] **Guardar capturas PNG** de casos reales (bueno, reflejo, una línea, sin líneas) en `test/capturas/`.
- [ ] **Probar en la pista** que tapando la cámara el auto frena, y que no frena justo antes de una intersección (ahí las líneas desaparecen a propósito).
- [ ] Mostrar el modo en el visor (A2-157); hoy se ve en la imagen de depuración.
- [x] **Bug corregido en A2-154:** `dibujar_cenital` usaba una variable `modo` que no existía (un reemplazo de texto cambió dos llamadas en vez de una). El nodo se caía al mirar la vista desde arriba. Desde A2-154, antes de cada commit se corre `pyflakes`, que lo detecta en segundos.

---

## A2-153: búsqueda alrededor del polinomio previo

Commit `f5a745a`.

### Descripción actualizada (para Jira)

Antes, cada cuadro buscaba las líneas desde cero en toda la imagen (histograma y ventanas
deslizantes): un reflejo o una mancha podía generar un pico falso. Ahora, si se conoce
dónde estaba la línea en un cuadro anterior, se busca solo cerca.

- `LaneTracker` guarda el polinomio de cada línea **válida** (según los chequeos de A2-152)
  y se lo pasa al detector en el cuadro siguiente: `detect(bgr, previas)`. El detector
  sigue sin memoria.
- **`prior_search`:** se toman solo los píxeles de la máscara a menos de
  `search_margin_m` (0,05 m) en horizontal del polinomio previo, y se ajusta el polinomio
  nuevo con ellos.
- **Vuelta a `sliding_windows`, por línea** (si la izquierda se pierde y la derecha no,
  solo la izquierda se reinicia):
  - no hay polinomio previo (arranque);
  - lo encontrado cerca no pasa los chequeos de la línea: se prueban las ventanas **en el
    mismo cuadro**;
  - la línea lleva más de `lost_timeout_s` sin ser válida;
  - el chequeo de ancho descartó esa línea (lo encontrado cerca no era la línea);
  - se rehizo el detector por un cambio de parámetros (el polinomio está en píxeles de
    la vista vieja).
- **Depuración:** la imagen muestra el método de cada línea
  (`busqueda: izq prior_search  der sliding_windows`).
- Parámetro nuevo: `search_margin_m`.

### Diferencias con el ticket original, y por qué

| Ticket original | Cómo quedó | Por qué |
|---|---|---|
| Guardar los polinomios en `lane_detection_logic.py` | Los guarda `LaneTracker` y se pasan como argumento | El nodo rehace el detector en cada cambio de parámetro: la memoria se borraba. El detector queda puro y fácil de testear |
| `reinit_after_frames` (en cuadros) | Se reusa `lost_timeout_s` (en segundos) | Los cuadros por segundo bajan cuando la Jetson está cargada; es el mismo concepto ("cuánto se le cree a una medición vieja") y queda un parámetro menos, en unidades SI |
| — | Si el chequeo de ancho descarta la línea, su previo se olvida enseguida | Evita que la búsqueda quede "enganchada" a una línea equivocada que pasa sus chequeos |
| — | Al rehacer el detector se olvidan los previos | Están en píxeles de la vista cenital vieja |
| El método en el debug **y en el log** | Solo en la imagen de depuración | Agregarlo a `LaneState` sería un segundo cambio de interfaz seguido; el criterio de aceptación es verlo en el visor |
| — | Sin reinicialización periódica | El chequeo de ancho ya cubre el caso "enganchado"; sería complejidad sin necesidad |

### Pendiente

- [ ] **Correr los tests** de `bfmc_lane_detection` (no se corrieron): secuencias del ticket (corrimiento de 2 cm, reflejo lejano, línea perdida) y reglas de los previos en `LaneTracker`.
- [ ] **Medir en la Jetson** que el tiempo por cuadro no empeora: `ros2 topic hz /perception/lane` en la misma zona, antes (commit `32a97f2`) y después.
- [ ] **En la pista:** en marcha normal, el visor debería mostrar `prior_search` casi todo el tiempo, y `sliding_windows` solo al arrancar o después de perder una línea.
- [ ] Afinar `search_margin_m` (0,05 m, sin medir).
- [ ] Mostrar el método en el visor (A2-157); hoy se ve en la imagen de depuración.

---

## A2-154: filtro de Kalman del estado del carril

Commit: el de "A2-154: ..." en `prueba-jetson` (el hash se completa en el próximo ticket).

### Descripción actualizada (para Jira)

El estado del carril se filtra en percepción (`lane_kalman.py`, lógica pura), que es quien
sabe qué tan buena es cada medición. Es la práctica estándar en sistemas de asistencia:
el desvío, el rumbo y la curvatura se siguen con un Kalman dentro del módulo de visión.

- **Estado** `x = [d, ψ, κ]`: desvío (m), rumbo (rad) y curvatura (1/m), positivos a la
  izquierda, **en la fila de medición** (dentro de la vista cenital).
- **Predicción** con el avance del auto `v·dt`: `d' = d + v·dt·ψ`, `ψ' = ψ + v·dt·κ`,
  `κ' = κ`. `dt` sale de los stamps de las imágenes y `v` de `/cmd/drive` (comandada; no hay odometría).
- **Corrección** con la medición completa (`H = I`). R según el modo: `kalman_r_two_lines` o
  `kalman_r_one_line`. En `MODE_PREDICTED` y `MODE_LOST` solo predice.
- **Rechazo de outliers:** distancia de Mahalanobis al cuadrado mayor a `kalman_gate_chi2`
  (11,34: 99 % de una chi-cuadrado con 3 grados de libertad).
- **Reinicio:** al salir de `MODE_LOST` con dos líneas, al rehacer el detector, y si
  descarta mediciones seguidas por más de `lost_timeout_s`.
- **Confianza:** `1 − σ_d / kalman_max_std_m`, acotada a [0, 1].
- **`LaneState.curvature`** (1/m, positiva a la izquierda): la mide el detector siempre, con
  o sin filtro. La necesita Pure Pursuit.
- **`kalman_enabled`** (apagado por defecto) para comparar con y sin filtro.
- **Depuración:** el estado filtrado junto al medido y su centro marcado en blanco.

### Diferencias con el ticket original, y por qué

| Ticket original | Cómo quedó | Por qué |
|---|---|---|
| Medición "evaluada en el punto del auto" | En la fila de medición (0,575 m de la cámara) | La vista cubre de 0,45 a 0,95 m: medir en el auto obliga a extrapolar el polinomio 0,45 m fuera de los datos. Un error de curvatura de 0,5 m⁻¹ daría ~5 cm de error de desvío |
| Restar el giro propio del auto (opcional) | No se hizo | Necesita la distancia entre ejes, que todavía no está medida y vive en `bfmc_control`: habría que duplicar un parámetro. Se agrega si en la pista se ve atraso en las curvas |
| Rechazo de outliers | + reinicio si rechaza por más de `lost_timeout_s` | Patrón común (contador de rechazos como detector de divergencia): sin él, después de un cambio real de golpe el filtro rechazaría todo para siempre |
| Q en unidades por cuadro | Q por segundo (`Q·dt`) | El filtro se comporta igual aunque cambien los cuadros por segundo |
| — | Filtro apagado por defecto | No cambiar cómo maneja el auto hasta medir R y Q y coordinar con `bfmc_control` |
| Log liviano con el estado filtrado | No se agregó | `ros2 bag record /perception/lane` ya graba el estado publicado |

### Pendiente

- [ ] **Correr los tests** (no se corrieron): `test_lane_kalman.py` (converge, salto de 20 cm, 5 cuadros sin medición, reinicios), curvatura en `test_lane_detection_logic.py`, contrato de `LaneState` en `bfmc_interfaces`.
- [ ] **Coordinar con quien mantiene `bfmc_control` (A2-138):** con el filtro activo, `filter_alpha: 1.0` para no filtrar dos veces (el límite de velocidad de giro se mantiene: protege el servo). Avisar también del campo nuevo `LaneState.curvature`.
- [ ] **Medir R en la pista:** auto quieto en una recta, varianza de desvío, rumbo y curvatura medidos durante unos segundos; repetir en una curva.
- [ ] **Ajustar Q en la pista:** subir desde valores chicos hasta que el filtrado siga las curvas sin atrasarse en el visor.
- [ ] **Comparar con y sin filtro** (`kalman_enabled`) en las zonas del protocolo de A2-152.
- [ ] **Medir el tiempo agregado por cuadro** en la Jetson (`ros2 topic hz /perception/lane`).
- [ ] Encender el filtro por defecto cuando R y Q estén medidos.

# Análisis del carril y el control en las curvas (2026-11-17)

Por qué el auto se sale en las curvas, gira al revés o frena, y qué cambiar. Todo lo de este
documento sale del código del repo y de un simulador que corre ese código. **Nada está
validado en la pista todavía.**

## Resumen

| # | Problema | Causa encontrada | Qué se toca | Estado |
|---|---|---|---|---|
| 1 | Se sale en casi todas las curvas | El PID mide el error 0,58 m adelante: dobla antes y se queda corrido hacia adentro | `error_shift_m: 0.38` y `filter_alpha: 1.0` en `bfmc_control` | **Implementado** (rama `fix/A2-158-prueba-casos-posibles`). Falta la pista |
| 2 | Gira al revés en las curvas cerradas a la derecha | El detector toma el carril de al lado como propio: el par «punteada + borde del vecino» pasa todos los chequeos | Filtro de Kalman con el giro propio del auto, prendido (`bfmc_lane_detection`) | **Implementado**. En el simulador: 22 de 22 curvas. Falta la pista |
| 3 | Frena «de la nada» | Más de 1 s sin líneas → `carril perdido`; o una falsa intersección (paso de cebra) | Por definir, con un bag de la pista | Hipótesis |

---

## 1. Qué se analizó

La cadena completa del carril, con el código de la rama `feature/A2-158-modulos-intercambiables`
y los parámetros de los `params.yaml` del repo:

```
cámara → bfmc_lane_detection (detector + tracker) → bfmc_control (PID) → bfmc_decision → ESP32
```

- **Detector** (`lane_detection_logic.py`): separa las líneas blancas, las pasa a una vista desde
  arriba (vista cenital de 0,80 m de ancho y 0,60 m de largo), busca cada línea con ventanas
  deslizantes, ajusta un polinomio y mide el desvío, el rumbo y la curvatura del carril.
- **Tracker** (`lane_tracker.py`): recuerda las líneas del cuadro anterior y decide el modo (dos
  líneas, una, predicho o perdido).
- **Control** (`control_logic.py`, `algoritmos/pid.py`): convierte el desvío y el rumbo en un
  ángulo de dirección.
- **Decisión** (`decision_logic.py`): frena ante emergencias (por ejemplo, carril perdido) y
  lanza maniobras (por ejemplo, intersección).

## 2. Cómo se analizó: el simulador

Leer el código no alcanza para saber qué pasa en una curva, porque el detector, el control y el
movimiento del auto se afectan entre sí. Por eso se armó un **simulador de lazo cerrado**:

1. Dibuja la vista cenital que vería el auto desde su posición en una pista sintética (rectas y
   curvas, con la línea derecha continua, la central punteada y el carril vecino).
2. Pasa esa imagen por el **detector, el tracker y el control reales del repo**, sin cambios.
3. Mueve el auto con el ángulo pedido (modelo de bicicleta) y repite, 30 veces por segundo.
4. Compara lo que el auto **cree** (lo medido) contra lo que **realmente** pasa (la pista).

Se validó primero con casos simples: en recta, el desvío y el rumbo medidos coinciden con los
reales.

### Qué no modela (límites)

| No modela | Consecuencia |
|---|---|
| La cámara real: perspectiva, luz, sombras, reflejos, desenfoque | No dice nada de cómo se ven las líneas con luz real |
| El servo y el motor: obedecen al instante | El auto real reacciona más lento |
| Medidas que nadie midió | Supone: línea de 2 cm, punteada de 10 cm cada 10 cm, cámara a 0,20 m del eje trasero, distancia entre ejes de 0,26 m, latencia de 0,1 s |
| El archivo `~/.config/bfmc/lane_detection.yaml` de la Jetson | Si pisa algún valor del repo, el auto corre con otros parámetros |

Conclusión: el simulador sirve para encontrar **fallas de lógica**. Los números finos se ajustan
en la pista.

## 3. Casos probados y resultado con el código de hoy

"Se sale" = el centro del eje trasero queda a más de medio carril (0,19 m) del centro.

**Recta** (15 casos):

| Caso | Resultado hoy |
|---|---|
| Centrado; corrido 8 a 12 cm | Bien |
| Ve solo la línea izquierda; solo la derecha; solo la derecha y corrido 10 cm | Bien |
| Línea gastada 40 cm; reflejo junto a la línea; sin carril vecino | Bien |
| Ciego 0,5 s | Bien: predice y sigue |
| Ciego 1,5 s | **Frena**: a los 1,0 s pasa a `LOST` y la decisión frena |
| Arranca cruzado 20° hacia el carril vecino | **Se cambia de carril**: toma el carril de al lado como propio |
| Línea de pare | Detecta intersección (esperado) |
| Paso de cebra (medidas supuestas) | **Detecta intersección** (falso positivo) |

**Curvas** (22 casos: a la izquierda y a la derecha; radios de 1,2, 0,9, 0,66, 0,55 y 0,46 m;
con y sin carril vecino; ciego 0,5 s en la curva; entrando corrido 8 cm):

| Configuración | Curvas sin salirse |
|---|---|
| Hoy | **1 de 22** |
| Con `error_shift_m: 0.38` | **18 de 22** (las 4 que fallan son el hallazgo 2) |
| Con `error_shift_m: 0.38` + prototipo de Kalman con movimiento propio | **22 de 22** |

## 4. Hallazgos

### Hallazgo 1: el control dobla antes de tiempo y se queda corrido hacia adentro

**Qué hace hoy.** El PID suma el desvío y el rumbo del carril **medidos 0,58 m delante del eje
trasero**: `error = desvío + 0,5 × rumbo` (`algoritmos/pid.py`).

**Por qué falla, paso a paso:**

1. En una curva, el carril que se ve 0,58 m adelante ya viene girando.
2. El PID interpreta ese giro como si el auto estuviera mal alineado.
3. Entonces dobla antes de llegar a la curva: en la traza, a t = 3,0 s, va 14° cruzado sin haber
   entrado todavía.
4. En la curva encuentra un equilibrio corrido hacia adentro: el desvío medido «tira» para un
   lado y el rumbo para el otro, y se compensan. El auto queda a 28 cm del centro, con las líneas
   a 19 cm.

La medición era correcta (el desvío medido coincidía con el real en ±1 cm): **la falla es del
control, no de la detección.**

**Qué se toca.** El parámetro `error_shift_m` de `bfmc_control`, que ya existe y hoy está en `0.0`.
Con `0.38`, el PID calcula el error en el pie de la cámara en vez de 0,58 m adelante: usa el
rumbo y la curvatura para «traer» la medición hacia el auto. Así compara dónde está el auto con
dónde debería estar **ahora**, no con dónde estará el carril más adelante.

**Evidencia.**

| Configuración | Curvas sin salirse (22) |
|---|---|
| Hoy (`pid`) | 1 |
| `curvature_ff_gain: 1.0` | peor (más salidas) |
| `algorithm: pure_pursuit` | falla en las cerradas a la derecha |
| `error_shift_m: 0.5` + `curvature_ff_gain: 1.0` | falla todas las curvas a la derecha |
| **`error_shift_m: 0.38`** | **18** |

Robustez de `error_shift_m: 0.38` (8 curvas + 1 recta, sin carril vecino):

| Latencia | Velocidad | Hoy | Con 0.38 |
|---|---|---|---|
| 0,1 s | 0,3 m/s | 2 de 9 | **9 de 9** |
| 0,2 s | 0,3 m/s | 3 de 9 | **9 de 9** |
| 0,1 s | 0,4 m/s | 1 de 9 | **9 de 9** |
| 0,2 s | 0,4 m/s | 2 de 9 | **9 de 9** |

**Riesgo.** `0.38` corresponde al pie de la cámara **si** la cámara está a 0,20 m del eje trasero
(`camera_to_rear_axle_m`, sin medir). Si la medida real es otra, el mejor valor cambia.

### Hallazgo 2: el detector toma el carril de al lado como propio

**Cuándo pasa.**

- En las curvas cerradas a la derecha (radio 0,66 m o menos) con el carril vecino a la vista.
- En recta, cuando el auto arranca muy cruzado hacia el vecino.

**Por qué falla, paso a paso** (traza de la curva a la derecha de radio 0,66 m):

1. En una curva a la derecha, nuestra línea derecha es la interior de la curva y **se sale de la
   vista** por el costado.
2. Quedan a la vista dos líneas: la **central punteada** (nuestra izquierda) y el **borde
   exterior del carril vecino**.
3. Esas dos líneas están a un ancho de carril entre sí (0,39 m), así que el par **pasa todos los
   chequeos**: es un carril válido, pero es el del vecino.
4. En un solo cuadro (t = 3,43 s) el centro medido salta de −0,10 m a +0,29 m: un carril entero.
5. El control obedece y el giro pasa de −10° a +23°: **gira al revés**.

**Qué ya existe y por qué no alcanza.** El detector ya tiene tres protecciones contra confundir
líneas:

- busca cada línea cerca de donde estaba en el cuadro anterior;
- separa los arranques de las búsquedas para que dos búsquedas no tomen la misma línea;
- corrige el lado de una línea sola por continuidad (`_lado_por_continuidad`).

Ninguna cubre este caso: el par que se forma es válido y la tercera solo actúa cuando el detector
ve **una** línea.

**Qué se probó y no funcionó:**

| Intento | Resultado | Por qué |
|---|---|---|
| Prototipo: continuidad también con dos líneas | Aguanta más, pero falla igual | La búsqueda alrededor del polinomio anterior se corre de a poco hacia el vecino (+0,09 → +0,19 → +0,41 m en 0,2 s). Comparar con el cuadro anterior no frena una deriva lenta |
| Filtro de Kalman del repo (`kalman_enabled: true`) | **Empeora** (falla también sin vecino) | Está sin afinar y no descuenta el giro del propio auto, como dice su código |
| Achicar la vista a 0,80 m de largo (`calib_forward_far_m`) | Arregla esas curvas, rompe la recta con el auto corrido hacia el vecino | Cambia un caso por otro |
| Achicar el ancho de la vista (`calib_half_width_m`) | No ayuda sola | El vecino sigue entrando cuando el auto va cruzado |

**Conclusión.** Al detector le falta saber **en qué carril está**, y eso no se arregla con
parámetros. Hace falta un cambio de código diseñado para eso (ver «Pasos a seguir»).

### Hallazgo 3: las frenadas

`bfmc_decision` frena en 6 casos y publica el motivo en `/vehicle/state.reason`
(`decision_logic.py:232-260`). En el simulador aparecen dos:

- **Carril perdido**: más de `lost_timeout_s` (1,0 s) sin líneas válidas.
- **Falsa intersección**: el paso de cebra sintético activa el detector de intersección, porque
  una fila cuenta como intersección si es blanca en el 35 % del ancho. Entonces se pide la maniobra
  y el auto queda detenido mientras arranca, y después cruza derecho a 0,2 m/s. Las medidas de la
  cebra son supuestas: **hay que confirmarlo en la pista.**

Los demás motivos (obstáculo del ultrasónico, enlace con el ESP32, watchdog) solo se ven en un
bag de la pista.

---

## 5. Pasos a seguir

| Paso | Qué | Se toca | Cómo se valida |
|---|---|---|---|
| 1 | Activar `error_shift_m: 0.38` | Parámetro de `bfmc_control` (primero en vivo) | En pista: 2 vueltas sin el cambio y 2 con, grabando el bag |
| 2 | Medir lo que el simulador supone | Nada: medir con cinta | Distancia cámara–eje trasero, distancia entre ejes, ángulo real de las ruedas al tope, ancho de las líneas, largo de la punteada |
| 3 | Identidad del carril (hallazgo 2) | Código de `bfmc_lane_detection`: diseño con red team / blue team antes de implementar | Primero en el simulador (las 4 curvas que fallan y los casos que ya andan), después en pista |
| 4 | Frenadas | Según lo que diga `/vehicle/state.reason` en el bag | En pista |
| 5 | Simulador como tests | Pasarlo al repo, con los casos de este documento como tests | Que ningún cambio rompa un caso que ya anda |

### Opciones para el paso 3 (sin decidir: falta diseñarlas y probarlas)

- ~~**Usar el tipo de línea**~~ como ancla principal: **descartada**. Según la pista (dato del
  equipo, 2026-11-17), la central es punteada solo donde los dos carriles van en la misma
  dirección, y continua donde van en direcciones opuestas. Entonces:
  - en los tramos de doble mano las tres líneas son continuas y no hay nada que distinga la
    central del borde del vecino;
  - en los tramos de misma dirección el auto puede ir por cualquiera de los dos carriles, así que
    la punteada puede ser nuestra izquierda o nuestra derecha.

  Como mucho, sirve como pista extra («la punteada separa dos carriles»), no para saber en qué
  carril está el auto.
- **Seguimiento con el movimiento del auto:** predecir dónde debería estar el carril según cuánto
  avanzó y giró el auto, y rechazar las mediciones que salten un carril. El Kalman del repo es la
  base, pero hoy no descuenta el giro del auto y por eso empeora.
- **Acotar la búsqueda alrededor del polinomio anterior** para que no se pueda correr hacia la
  línea de al lado.

## 6. ¿Con esto alcanza?

**No.** El paso 1 arregla, en el simulador, la causa más frecuente de salidas en curva. Quedan
fuera:

- **El hallazgo 2** (gira al revés en las curvas cerradas a la derecha): sigue sin solución.
- **Todo lo que depende de la cámara real** (luz, sombras, reflejos): el simulador no lo prueba.
  El bag de la pista lo muestra.
- **Las intersecciones:** la maniobra es en lazo abierto (por tiempo) y la dirección está fija en
  `straight`.
- **Las medidas físicas sin medir:** pueden cambiar los valores óptimos.

Criterio propuesto para dar el carril por «suficiente»: **5 vueltas seguidas sin salirse ni
frenar sin motivo**, con el bag grabado como evidencia.

---

## 7. Decisiones e implementación (2026-11-17, rama `fix/A2-158-prueba-casos-posibles`)

### Decisiones

| Decisión | Alternativa descartada | Por qué |
|---|---|---|
| `error_shift_m: 0.38` en el PID | `curvature_ff_gain` más alto; `pure_pursuit` | En el simulador, 18 de 22 curvas contra 1 de 22; las otras dos opciones dan peor resultado (sección 4) |
| Kalman **con el giro propio** del auto, prendido | Kalman como estaba; continuidad de un cuadro; achicar la vista | Es la única opción que resuelve el hallazgo 2 sin romper otro caso: 22 de 22 curvas y 36 de 37 casos en total. Las otras empeoran o cambian un caso por otro (sección 4) |
| No usar el tipo de línea para saber el carril | Punteada = nuestra izquierda | En la pista la central es continua en los tramos de doble mano, y en los de misma mano el auto puede ir por cualquiera de los dos carriles |
| Valores en los `params.yaml` | `ros2 param set` a mano | Quedan aplicados al lanzar, sin pasos manuales. Se pueden cambiar en vivo igual |

### Qué cambió en el código

| Archivo | Cambio | Por qué |
|---|---|---|
| `bfmc_lane_detection/lane_kalman.py` | La predicción descuenta el giro del auto: `dθ = v·dt·tan(δ)/L`, `psi -= dθ`, `d -= x_ref·dθ` | Sin esto, mientras el auto dobla el filtro cree que el carril quedó quieto y descarta las mediciones buenas |
| `bfmc_lane_detection/lane_tracker.py` | Una medición que el filtro descartó **no se guarda** como polinomio previo | Si se guarda, la búsqueda del cuadro siguiente arranca desde la línea del vecino y el error se repite |
| `bfmc_lane_detection/lane_detection_node.py` | Lee el giro de `/cmd/drive` (antes solo la velocidad) y arma el filtro con la distancia a la fila de medición | El filtro necesita el giro comandado |
| `bfmc_lane_detection/config/params.yaml` | `kalman_enabled: true`; nuevos `kalman_wheelbase_m: 0.26` y `kalman_camera_to_rear_axle_m: 0.20` | Prender el filtro con el giro propio |
| `bfmc_control/config/params.yaml` | `error_shift_m: 0.38`, `filter_alpha: 1.0` | Hallazgo 1, y no filtrar dos veces (el Kalman ya suaviza) |
| `bfmc_bringup/test/test_contracts.py` | Las medidas del auto del filtro tienen que ser iguales a las del control | Las dos son la misma medida física en dos paquetes |
| `herramientas/simulador_carril/` | El simulador y los 37 casos | Repetir el análisis después de cada cambio |

### Cómo volver atrás en la pista (en vivo, sin recompilar)

```bash
ros2 param set /control error_shift_m 0.0           # hallazgo 1 como antes
ros2 param set /control filter_alpha 0.6
ros2 param set /lane_detection kalman_enabled false # hallazgo 2 como antes
ros2 param set /lane_detection kalman_wheelbase_m 0.0  # filtro prendido pero sin el giro propio
```

### Qué mirar en la pista

1. Curvas a la izquierda y a la derecha: ¿se sale? ¿para qué lado?
2. Curvas cerradas a la derecha con el carril vecino a la vista: ¿gira al revés?
3. Rectas: ¿oscila? Con `filter_alpha: 1.0` el giro es menos suave; si oscila, probar 0.8.
4. ¿Frena? Mirar `/vehicle/state` (campo `reason`).
5. Grabar el bag liviano (skill `analizar-bag`).

### Pendiente

- [ ] Validar en pista (sección «Qué mirar»).
- [ ] Medir `wheelbase_m` y `camera_to_rear_axle_m` (y copiarlas en los dos `params.yaml`).
- [ ] Falso positivo de intersección en el paso de cebra (hallazgo 3): confirmar en la pista.
- [ ] Doblar antes de la curva: con `error_shift_m` llega centrado, pero sigue empezando a girar
  20 a 60 cm antes, porque un solo polinomio cubre toda la vista.
- [ ] `test_flake8` ya fallaba antes de este cambio en `bfmc_lane_detection`, `bfmc_control` y
  `bfmc_bringup` (archivos no tocados acá). Los tests nuevos de `test_lane_kalman.py` pasan.

---

## 8. Prueba en pista del 2026-11-17 y ajuste para la curva cerrada

**Resultado de la prueba** (`hsv_ventanas` + `pid`, 0,41 m/s, vista hasta 0,75 m): recta y curva
suave, muy bien. En la curva cerrada el auto **se abre hacia afuera**.

**Qué mostró el bag** (`20261117_234430_curva_cerrada`):

- La detección veía la curva: rumbo de 16° a 45° y curvatura de 1,16 a 1,53 1/m, casi siempre con
  **una sola línea** (la exterior).
- El control pedía poco giro: mediana de **11°**, cuando hacían falta ~21°.
- Reproducción: pasando lo medido por el control del repo, se obtiene lo grabado (mediana 1,1°
  de diferencia). La causa es `error_shift_m 0.38`. Con un rumbo de 35° a 45°, la cuenta de ángulo
  chico que lleva el error hacia el auto se pasa y el PID resta giro.
- `curvature_ff_min_confidence 0.6` **no ayuda** (descartado).
- Con `error_shift_m 0.30` en pista: la cerrada mejora (se pasa un poco de las líneas), pero la
  suave empeora. **Un solo valor no sirve para las dos curvas.**

**Decisión:** `error_shift_m` depende de la curvatura medida. Hasta 0,9 1/m usa 0,38 (la suave
midió 0,35–0,89); desde 1,2 1/m usa 0,20 (la cerrada midió 1,16–1,53); en el medio, interpola. Son
3 parámetros nuevos (`error_shift_tight_m`, `error_shift_tight_kappa_start`,
`error_shift_tight_kappa_full`); `error_shift_tight_kappa_full: 0.0` lo apaga.

| Reproducción del bag | Giro mediano |
|---|---|
| Curva suave | 8,6° (igual que con 0,38, que funcionó) |
| Curva cerrada | 25° (antes, 11°) |

**Descartado:** elegir por el rumbo (la suave llega a 29° y la cerrada arranca en 16°) y arreglar la
cuenta para ángulos grandes (es la solución de fondo, pero lleva más tiempo).

**Riesgos:** los umbrales salen de un solo bag de una sola pista; se ajustan en vivo. No se corrió
el simulador con este cambio.

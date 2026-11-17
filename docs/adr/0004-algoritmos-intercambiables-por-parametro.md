# ADR-0004: Detección de carril y control intercambiables por parámetro

- **Estado:** Propuesta (2026-11-17). Falta que la apruebe el equipo y probarla en el auto.
- **Ticket:** A2-158.

## Contexto

Queremos probar en la pista otras formas de detectar el carril y de girar sin perder las que ya andan, y elegir cuál corre sin tocar código. Hasta ahora:

- `bfmc_lane_detection` tenía un solo detector: umbral fijo en HSV + ventanas deslizantes. Con sombras o sol directo, el umbral que anda en un lugar pierde la línea en otro (informe Etapa I, 4.3 y 5.2.4.4).
- `bfmc_control` ya aceptaba `algorithm: pid | pure_pursuit`, pero los dos algoritmos vivían mezclados en `control_logic.py` con `if`, y Pure Pursuit usaba una distancia de mirada fija que en las curvas más cerradas (~0,46 m de radio) quedaba más lejos que el radio.

La propuesta inicial era correr varios en paralelo y compararlos en el visor. Se descartó: en el auto corre **uno de cada uno por vez**, y se compara entre corridas.

## Decisión

1. **Patrón Strategy con registro, dentro de cada paquete.** Cada paquete tiene una carpeta con una clase por variante y un diccionario `REGISTRO` nombre → clase:
   - `bfmc_lane_detection/detectores/`: `hsv_ventanas` (el de siempre) y `l_adaptativo` (canal L de HLS con umbral adaptativo local + piso fijo + CLAHE opcional). Todos heredan de `LaneDetector` y cambian solo la segmentación (`_segmentar`); la vista cenital, las ventanas, los chequeos, el Kalman y la medición son comunes.
   - `bfmc_control/algoritmos/`: `pid` y `pure_pursuit`. Todos heredan de `AlgoritmoDeGiro` y devuelven giro y velocidad; lo que no es el algoritmo (sin carril, saturación, filtro, límite de velocidad de giro, límite de la curva imposible) lo hace `ControlLogic` igual para todos. Así, al comparar dos corridas, la única diferencia es el algoritmo.
2. **Se elige con un parámetro de texto:** `detector` en `/lane_detection` y `algorithm` en `/control`. El valor por defecto es el del `params.yaml` de cada paquete.
3. **Tres formas de elegir, todas sobre el mismo parámetro:**
   - Al lanzar: `ros2 launch bfmc_bringup system.launch.py lane_detector:=l_adaptativo control_algorithm:=pure_pursuit` (también `calibrar.sh --lane ... --control ...`).
   - Desde el visor: dos selectores en el encabezado, que se eligen **antes de Conectar**. Al conectar, el visor manda `set_parameters` a los dos nodos y se bloquean hasta desconectar.
   - En vivo: `ros2 param set /lane_detection detector l_adaptativo`.
4. **Con el auto andando (ESP32 en `RUNNING`) se rechaza el cambio de detector**, igual que la geometría. El cambio de algoritmo de control se acepta en vivo, como antes: el nuevo arranca sin estado y el filtro de salida evita el salto.
5. **Los contratos no cambian.** `LaneState` y `/control/lane_cmd` son los mismos para cualquier variante. Ninguna variante importa código de otro paquete.

Cambios que vinieron con esta decisión:

- Pure Pursuit acorta la distancia de mirada en las curvas (`lookahead_curve_m`, `lookahead_curve_kappa`) y usa la curvatura confiable (con una sola línea la curvatura es ruido).
- `ControlLogic` baja la velocidad (`infeasible_curve_speed_factor`) si la curva pide más que el tope de dirección: R_min = L / tan(δmax) ≈ 0,56 m con L = 0,26 m y 25°.
- El tope de dirección es **25°** en `bfmc_control` y en `bfmc_hardware_bridge` (lo controla `test_contracts.py`). Sale del informe, donde es la corrección del control ON/OFF, no un ángulo medido: `[por-verificar]` medir el ángulo real de las ruedas con el servo en cada tope.
- La vista cenital pasa de 0,50 a 0,80 m de ancho (`calib_half_width_m: 0.40`, `warp_width: 640`, misma escala de 1,25 mm/px) para no perder la línea de afuera en las curvas cerradas.

## Cómo agregar una variante

1. Una clase en un archivo nuevo de `detectores/` (hereda de `LaneDetector`, reemplaza `_segmentar`) o de `algoritmos/` (hereda de `AlgoritmoDeGiro`, implementa `calcular`).
2. Una línea en `REGISTRO`. En control, además, el nombre en `tipos.ALGORITMOS`.
3. Sus parámetros en el `params.yaml` y en el nodo, y una opción en los selectores del visor.
4. Los tests de contrato (`test_detectores.py`, `test_algoritmos.py`) recorren el `REGISTRO`: la variante nueva entra sola y tiene que pasarlos.

## Alternativas descartadas

- **Correr varias variantes en paralelo y compararlas en el visor.** Duplica el uso de CPU en la Jetson, y solo una puede mandar el auto: la comparación real es en la pista, una corrida por variante.
- **Un nodo por variante** (`lane_detection_hsv`, `lane_detection_l`, …). Duplica la calibración, el guardado, los topics de depuración y el launch.
- **Plugins de `pluginlib`.** Es de C++; en Python el registro es más simple y alcanza.

## Consecuencias

- Probar una idea nueva es una clase y una línea, sin tocar el resto del pipeline.
- Los ajustes de cada variante (`l_offset`, `l_min`, …) viven todos en el mismo `params.yaml` aunque la variante no esté corriendo. "Guardar" guarda los del canal L junto con los de HSV.
- El visor muestra los deslizadores de la variante elegida (HSV o canal L).

## Por verificar en el auto

- `[por-verificar]` `l_adaptativo` con sombras reales: que encuentre las dos líneas donde `hsv_ventanas` pierde una, y que no marque reflejos.
- `[por-verificar]` Pure Pursuit con Ld en curva contra PID en la curva más cerrada, a la misma velocidad.
- `[por-verificar]` Medir L (entre ejes), la cámara al eje trasero y el ángulo real de las ruedas: Pure Pursuit y R_min dependen de los tres.

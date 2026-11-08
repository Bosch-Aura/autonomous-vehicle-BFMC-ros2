# bfmc_control

Convierte el estado del carril en un ángulo de dirección y una velocidad. Por ahora usa un PID; el algoritmo se puede cambiar sin afectar al resto del sistema, porque solo importan sus entradas y salidas.

## Responsabilidad

Calcular el comando de seguimiento de carril. **No decide** si el auto frena o maniobra: eso es de `bfmc_decision`, que reenvía este comando cuando corresponde.

## Topics

| Dirección | Parámetro | Topic | Tipo | Descripción |
|---|---|---|---|---|
| Se suscribe | `lane_topic` | `/perception/lane` | `bfmc_interfaces/LaneState` | Estado del carril |
| Publica | `cmd_topic` | `/control/lane_cmd` | `ackermann_msgs/AckermannDriveStamped` | Dirección (rad, positivo a la izquierda) y velocidad (m/s) |

## Cómo funciona

1. El error es `desvío lateral + heading_weight_m_per_rad * error de rumbo`.
2. Los errores menores a `deadband_m` se ignoran.
3. PID: `kp * error + integral + kd * derivada`. El término integral está acotado por `integral_limit_rad` (anti-windup).
4. La salida se satura a `max_steer_rad`, se filtra con `filter_alpha` y se limita su velocidad de cambio a `max_steer_rate_rad_s`.
5. La velocidad baja al girar: `cruise_speed_mps * (1 - slowdown_gain * |giro| / max_steer_rad)`.

**Si el carril no se detecta** (o la confianza es menor a `min_confidence`), el PID se reinicia, la dirección vuelve al centro respetando el límite de giro y la velocidad baja a `cruise_speed_mps * lost_speed_factor`. Si pasa más de `max_dt_s` entre dos mediciones, también se reinicia.

Si el nodo deja de recibir mensajes de carril, **no publica nada**: `bfmc_decision` interpreta esa ausencia como entrada vencida.

## Pure Pursuit (A2-155)

`algorithm` elige el giro: `pid` (el de siempre, por defecto hasta validar el otro en pista) o `pure_pursuit` (`pure_pursuit.py`). Los dos comparten la saturación, el filtro y el límite de velocidad de giro de la salida, y qué hacer sin carril. Se puede cambiar en vivo (`ros2 param set /control algorithm pure_pursuit`).

Marco del auto: origen en el eje trasero, x adelante, y a la izquierda.

1. **Dónde se midió el carril.** `LaneState` trae desvío, rumbo y curvatura medidos a `reference_distance_m` del pie de la cámara, no en el auto: el punto de medición está a `x_ref = camera_to_rear_axle_m + reference_distance_m` del eje trasero.
2. **Punto objetivo** a `Ld` del eje trasero: `y = d + ψ·s + κ·s²/2`, con `s = Ld − x_ref`. El desvío ya es la posición del centro respecto del auto (positivo a la izquierda): no se le cambia el signo.
3. **Giro:** `δ = atan(2·L·sin α / l)`, con `α = atan2(y, Ld)`, `l` la distancia al punto y `L = wheelbase_m`.
4. **Ld** `= clamp(lookahead_gain_s · v, lookahead_min_m, lookahead_max_m)`. Arrancar con `lookahead_gain_s: 0` (Ld fija). Ld tiene que caer dentro de lo que ve la cámara: con la vista de 0,45 a 0,95 m delante de la cámara, de `camera_to_rear_axle_m + 0,45` a `camera_to_rear_axle_m + 0,95`; más cerca se extrapola a ciegas.
5. **Velocidad:** en las curvas, la que no supera `max_lateral_accel_mps2` (`v = √(a / |κ|)`, la práctica común de adaptación en curvas); y con poca confianza baja hasta `speed_min_factor` de la de crucero.

**Pendiente de medir en el auto:** `wheelbase_m` (centro de la rueda de adelante al de la de atrás, ruedas derechas, los dos lados) y `camera_to_rear_axle_m` (del eje trasero a la vertical de la cámara).

**Latencia.** El encabezado de `/control/lane_cmd` lleva el instante de la imagen de la que salió (el de `LaneState`), y `bfmc_decision` lo pasa a `/cmd/drive`. Para medirla: `ros2 run bfmc_bringup medir_latencia` (promedio y p99).

## Parámetros

Los de la lógica (ganancias, velocidades, filtros) se pueden cambiar **en vivo** con `ros2 param set /control <nombre> <valor>` o con el servicio `set_parameters`: el nodo valida el valor, lo aplica en el ciclo siguiente y conserva el estado del PID. Un valor inválido (por ejemplo una velocidad negativa) se rechaza y queda el anterior. Es lo que usa el deslizador de velocidad del visor.

Viven en [`config/params.yaml`](config/params.yaml). El nodo no tiene valores por defecto en el código: si falta uno, falla al arrancar.

| Parámetro | Unidad | Descripción |
|---|---|---|
| `lane_topic`, `cmd_topic`, `frame_id` | — | Nombres de topics y frame |
| `kp`, `ki`, `kd` | rad/m, rad/(m·s), rad·s/m | Ganancias del PID |
| `heading_weight_m_per_rad` | m/rad | Peso del error de rumbo frente al desvío |
| `deadband_m` | m | Zona muerta |
| `integral_limit_rad` | rad | Tope del término integral |
| `max_steer_rad` | rad | Ángulo máximo (25° según el ADR-0001) |
| `max_steer_rate_rad_s` | rad/s | Límite de velocidad de giro |
| `filter_alpha` | — | Filtro de la salida, en (0, 1]. 1 = sin filtro |
| `cruise_speed_mps` | m/s | Velocidad de crucero |
| `slowdown_gain` | — | Cuánto baja la velocidad al girar al máximo, en [0, 1) |
| `min_confidence` | — | Confianza mínima para usar el carril |
| `lost_speed_factor` | — | Fracción de la velocidad de crucero sin carril. Con 1 no baja: el carril solo orienta y, sin él, el auto sigue a ciegas con la dirección al centro |
| `max_dt_s` | s | Pausa máxima entre mediciones antes de reiniciar el PID |
| `algorithm` | — | `pid` o `pure_pursuit` |
| `wheelbase_m`, `camera_to_rear_axle_m` | m | Medidas del auto para Pure Pursuit |
| `lookahead_min_m`, `lookahead_max_m`, `lookahead_gain_s` | m, m, s | Distancia de mirada |
| `max_lateral_accel_mps2` | m/s² | Aceleración lateral máxima en curvas (Pure Pursuit) |
| `speed_min_factor` | — | Fracción mínima de la velocidad con poca confianza (Pure Pursuit) |

**Los valores de las ganancias y los límites son una primera propuesta, sin calibrar.** Se afinan en el auto. Orden sugerido: con `ki` y `kd` en 0, subir `kp` hasta que siga el carril sin oscilar; después agregar `kd` para amortiguar y, al final, `ki` si queda un error sostenido en las curvas.

## Cómo probarlo

```bash
docker compose run --rm test
```

Para correr solo este paquete dentro del contenedor (`docker compose run --rm dev`):

```bash
cd ros2_ws
colcon build --symlink-install --packages-up-to bfmc_control
colcon test --packages-select bfmc_control --event-handlers console_direct+
```

Para lanzar el nodo con sus parámetros:

```bash
source install/setup.bash
ros2 launch bfmc_control control.launch.py
```

## Estructura

- `bfmc_control/control_logic.py`: el PID, sin ROS. El tiempo entra como argumento.
- `bfmc_control/control_node.py`: conecta mensajes con la lógica.
- `config/params.yaml`: parámetros.
- `launch/control.launch.py`: arranca el nodo con sus parámetros.
- `test/`: tests de la lógica (con tiempo simulado), del nodo y de estilo.

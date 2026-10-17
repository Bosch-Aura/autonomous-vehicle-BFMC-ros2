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

## Parámetros

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
| `lost_speed_factor` | — | Fracción de la velocidad de crucero sin carril |
| `max_dt_s` | s | Pausa máxima entre mediciones antes de reiniciar el PID |

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

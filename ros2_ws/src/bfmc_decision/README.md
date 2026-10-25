# bfmc_decision

El nodo que **decide** qué hace el auto, y el **único que publica el comando final** (`/cmd/drive`). Percepción y seguridad informan hechos; este nodo los arbitra.

## Responsabilidad

Elegir en cada ciclo una sola fuente para el comando, publicar el estado del vehículo y pedir maniobras. **No ejecuta** las maniobras (eso es de `bfmc_maneuvers`) ni calcula la dirección del carril (`bfmc_control`).

## Prioridad de decisión

En cada ciclo se aplica la primera regla que corresponda:

| # | Estado | Cuándo | Comando |
|---|---|---|---|
| 0 | `STARTING` | Todavía no llegó alguna de las primeras entradas (seguridad, hardware, comando de carril) | Cero |
| 1 | `EMERGENCY` | Seguridad indica emergencia, o una entrada se venció, o el enlace con el ESP32 cayó | Cero |
| 2 | `MANEUVER` | Hay una maniobra pedida o en curso | El de la maniobra (cero si no hay comando vigente) |
| 3 | `MANEUVER` (pedido) | Una señal o la intersección del carril dispara un evento | Cero hasta que el servidor responda |
| 4 | `LANE_FOLLOW` | Nada de lo anterior | El comando de `bfmc_control` |

## Topics

| Dirección | Topic | Tipo |
|---|---|---|
| Se suscribe | `/perception/lane` | `bfmc_interfaces/LaneState` |
| Se suscribe | `/perception/signs` | `bfmc_interfaces/SignDetectionArray` |
| Se suscribe | `/safety/status` | `bfmc_interfaces/SafetyStatus` |
| Se suscribe | `/hardware/status` | `bfmc_interfaces/HardwareStatus` |
| Se suscribe | `/control/lane_cmd` | `ackermann_msgs/AckermannDriveStamped` |
| Se suscribe | `/maneuver/cmd` | `ackermann_msgs/AckermannDriveStamped` |
| **Publica** | **`/cmd/drive`** | `ackermann_msgs/AckermannDriveStamped` (único publicador) |
| Publica | `/vehicle/state` | `bfmc_interfaces/VehicleState` |
| Cliente de acción | `/maneuver/execute` | `bfmc_interfaces/ManeuverExecute` |

## Emergencia

Cualquiera de estas condiciones detiene el auto:

| Condición | Rige durante una maniobra |
|---|---|
| `/safety/status` indica emergencia | Sí |
| `/safety/status` vencido (`safety_timeout_s`) | Sí |
| `/hardware/status` vencido (`hardware_timeout_s`) | Sí |
| Enlace con el ESP32 caído, o watchdog del ESP32 cortado | Sí |
| Comando de carril vencido (`lane_cmd_timeout_s`) | **No** |
| Carril sin detectar por más de `lane_lost_timeout_s` | **No** |

Las dos últimas **no rigen durante una maniobra**: dentro de una intersección las líneas del carril desaparecen a propósito y la maniobra no las usa. Al terminar la maniobra, lo observado del carril durante ella se descarta y se empieza de cero.

Una emergencia durante una maniobra la **cancela**. Cuando las condiciones se normalizan, el estado vuelve solo a `LANE_FOLLOW`.

## Eventos que piden una maniobra

| Evento | Maniobra |
|---|---|
| Una señal de `stop_labels` con confianza mínima y a `stop_distance_m` o menos | `stop` |
| Una señal de `intersection_labels` a `intersection_distance_m` o menos, **o** `intersection_min_frames` cuadros seguidos con intersección en el carril | `intersection` en `intersection_direction` |

- El **stop tiene prioridad** sobre la intersección.
- Una señal **sin distancia conocida** (`NaN`) se ignora: no se puede juzgar si está cerca.
- Las detecciones más viejas que `signs_timeout_s` se ignoran, por si el detector deja de publicar.
- Después de cada maniobra (terminada, rechazada o cancelada) hay una espera de `maneuver_cooldown_s`, para no repetir la misma señal.
- Si el servidor de maniobras no responde en `maneuver_request_timeout_s`, o la maniobra supera `maneuver_timeout_s`, se cancela y se informa.

## Parámetros

Viven en [`config/params.yaml`](config/params.yaml). El nodo no tiene valores por defecto en el código: si falta uno, falla al arrancar. **Todos los tiempos y umbrales son una primera propuesta, sin calibrar** (marcados `[asumido]`).

| Grupo | Parámetros |
|---|---|
| Frecuencia y topics | `rate_hz`, `*_topic`, `maneuver_action_name`, `frame_id` |
| Entradas vencidas | `safety_timeout_s`, `hardware_timeout_s`, `lane_cmd_timeout_s`, `maneuver_cmd_timeout_s`, `signs_timeout_s`, `lane_lost_timeout_s` |
| Maniobras | `maneuver_request_timeout_s`, `maneuver_timeout_s`, `maneuver_cooldown_s` |
| Señales | `min_sign_confidence`, `stop_distance_m`, `intersection_distance_m`, `stop_labels`, `intersection_labels` |
| Intersección | `intersection_direction` (`left`, `right` o `straight`), `intersection_min_frames` |

`intersection_direction` es fija mientras no exista un planificador de ruta que diga hacia dónde ir.

## Cómo probarlo

```bash
docker compose run --rm test
```

Los tests de la lógica usan tiempo simulado y cubren cada nivel de prioridad, cada transición y el ciclo de vida de una maniobra. Los del nodo usan un servidor de acción falso que hace de `bfmc_maneuvers`.

Para correr solo este paquete dentro del contenedor (`docker compose run --rm dev`):

```bash
cd ros2_ws
colcon build --symlink-install --packages-up-to bfmc_decision
colcon test --packages-select bfmc_decision --event-handlers console_direct+
```

## Estructura

- `decision_logic.py`: la máquina de estados, sin ROS. El tiempo entra como argumento.
- `decision_node.py`: conecta los topics y el cliente de la acción con la lógica.
- `config/params.yaml`, `launch/decision.launch.py` y `test/`.

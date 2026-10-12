# bfmc_safety

Informa si el auto debe detenerse según el sensor ultrasónico. **Informa: no frena por sí mismo.** `bfmc_decision` y `bfmc_hardware_bridge` leen su estado y actúan.

## Responsabilidad

Convertir las lecturas del ultrasónico en un estado de seguridad. No envía comandos al motor.

## Topics

| Dirección | Parámetro | Topic | Tipo | Descripción |
|---|---|---|---|---|
| Se suscribe | `ultrasonic_topic` | `/hardware/ultrasonic` | `sensor_msgs/Range` | Distancia al obstáculo de adelante (calidad *best effort*) |
| Publica | `status_topic` | `/safety/status` | `bfmc_interfaces/SafetyStatus` | Estado de seguridad, a `publish_rate_hz` |

Publica a frecuencia fija, no solo cuando llega una lectura: así puede avisar una emergencia cuando el sensor **deja** de enviar datos.

## Cómo se interpreta una lectura

La convención es la de `sensor_msgs/Range` (REP-117):

| Lectura | Se interpreta como |
|---|---|
| Distancia finita dentro del alcance | Esa distancia |
| Más allá del alcance, o `+inf` | **Camino despejado.** `distance_m` se informa como el alcance máximo |
| Más cerca del mínimo, o `-inf` | Obstáculo pegado al sensor, a la distancia mínima |
| `NaN` | Error del sensor |

Importante: el ultrasónico sin eco (el ESP32 informa `valid:false`) significa que **no hay nada dentro del alcance**, no que el sensor falló. Si se tratara como emergencia, el auto frenaría en cada tramo libre. Por eso `bfmc_hardware_bridge` debe publicar `+inf` en esos casos.

## Cuándo hay emergencia

| Motivo (`SafetyStatus.reason`) | Condición |
|---|---|
| `REASON_OBSTACLE` | La distancia baja de `stop_distance_m` |
| `REASON_SENSOR_STALE` | No hay lecturas, o la última tiene más de `stale_timeout_s` |
| `REASON_SENSOR_INVALID` | La lectura es `NaN` y `emergency_on_invalid` es `true` |

**Histéresis:** la emergencia por obstáculo empieza por debajo de `stop_distance_m` y termina solo por encima de `resume_distance_m`. Entre los dos umbrales mantiene el estado anterior, para que no oscile con una lectura en el borde.

**Sin lecturas desde el arranque** también es emergencia: sin datos no se sabe qué hay adelante.

## Parámetros

Viven en [`config/params.yaml`](config/params.yaml). El nodo no tiene valores por defecto en el código: si falta uno, falla al arrancar.

| Parámetro | Unidad | Descripción |
|---|---|---|
| `ultrasonic_topic`, `status_topic`, `frame_id` | — | Topics y frame |
| `publish_rate_hz` | Hz | Frecuencia del estado |
| `stop_distance_m` | m | Por debajo hay emergencia. **Asumido** (0,30 m) |
| `resume_distance_m` | m | Por encima se libera. Debe ser mayor que la anterior. **Asumido** (0,40 m) |
| `stale_timeout_s` | s | Antigüedad máxima de una lectura. **Asumido** (0,3 s) |
| `emergency_on_invalid` | — | Si una lectura `NaN` cuenta como emergencia |

## Cómo probarlo

```bash
docker compose run --rm test
```

Para correr solo este paquete dentro del contenedor (`docker compose run --rm dev`):

```bash
cd ros2_ws
colcon build --symlink-install --packages-up-to bfmc_safety
colcon test --packages-select bfmc_safety --event-handlers console_direct+
```

Para lanzar el nodo con sus parámetros:

```bash
source install/setup.bash
ros2 launch bfmc_safety safety.launch.py
```

## Estructura

- `bfmc_safety/safety_logic.py`: las reglas de seguridad, sin ROS. El tiempo entra como argumento.
- `bfmc_safety/safety_node.py`: conecta mensajes con la lógica.
- `config/params.yaml`: parámetros.
- `launch/safety.launch.py`: arranca el nodo con sus parámetros.
- `test/`: tests de la lógica (con tiempo simulado), del nodo y de estilo.

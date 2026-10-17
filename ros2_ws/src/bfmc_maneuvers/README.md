# bfmc_maneuvers

Ejecuta maniobras puntuales cuando `bfmc_decision` las pide. Patrón **Strategy**: cada maniobra es una clase con una sola razón para cambiar, y agregar una nueva no obliga a tocar las anteriores.

## Responsabilidad

Ejecutar una maniobra en lazo abierto y publicar su comando mientras dura. **No decide** cuándo hacerla: eso es de `bfmc_decision`.

## Interfaces

| Tipo | Nombre (parámetro) | Interfaz | Descripción |
|---|---|---|---|
| Action server | `/maneuver/execute` (`action_name`) | `bfmc_interfaces/ManeuverExecute` | Pedido de una maniobra |
| Publica | `/maneuver/cmd` (`cmd_topic`) | `ackermann_msgs/AckermannDriveStamped` | Comando, solo mientras hay una maniobra activa |

**El pedido** lleva `maneuver` (`"stop"` o `"intersection"`) y `direction` (`"left"`, `"right"` o `"straight"`, obligatoria solo para la intersección). Se rechaza si la maniobra o la dirección no son válidas, o si **ya hay otra maniobra activa**: solo una a la vez. Se puede **cancelar** en cualquier momento. Mientras se ejecuta informa el tiempo transcurrido (`elapsed_s`), y al terminar o cancelar publica un último comando de parada.

## Maniobras

Ambas son secuencias de **tramos** con velocidad y dirección constantes. La duración, la velocidad y el ángulo de cada tramo son parámetros.

| Maniobra | Tramos |
|---|---|
| `stop` | 1. Detenido `stop.hold_s`. 2. Avanza derecho `stop.exit_s` a `stop.exit_speed_mps` para salir de la zona de la señal (con `exit_s` en 0 no hay avance) |
| `intersection` `left` / `right` | 1. Avanza derecho `approach_s`. 2. Gira `turn_s` con `steer_rad` (izquierda positivo, derecha negativo) |
| `intersection` `straight` | Avanza derecho `intersection.straight_s` |

Dentro de una intersección las líneas del carril desaparecen: por eso la maniobra es **en lazo abierto**, por tiempo, y no sigue el carril. Eso la hace sensible a la velocidad real del auto; los tiempos se calibran en la pista.

## Parámetros

Viven en [`config/params.yaml`](config/params.yaml). El nodo no tiene valores por defecto en el código: si falta uno, falla al arrancar. **Todos los tiempos, velocidades y ángulos son una primera propuesta, sin calibrar** (marcados `[asumido]`).

| Parámetro | Unidad | Descripción |
|---|---|---|
| `action_name`, `cmd_topic`, `frame_id` | — | Nombre de la acción, topic y frame |
| `rate_hz` | Hz | Frecuencia del comando |
| `stop.hold_s`, `stop.exit_s`, `stop.exit_speed_mps` | s, s, m/s | Stop |
| `intersection.speed_mps` | m/s | Velocidad en la intersección |
| `intersection.straight_s` | s | Duración de cruzar derecho |
| `intersection.left.*`, `intersection.right.*` | s, s, rad | `approach_s`, `turn_s` y `steer_rad` (magnitud, siempre positiva) |

## Cómo agregar una maniobra

1. Creá `strategies/<nombre>.py` con una clase que implemente `ManeuverStrategy` (o derive de `LegStrategy` si es una secuencia de tramos).
2. Sumá su rama en `strategies/factory.py` y sus parámetros en `params.yaml` y en el nodo.
3. Escribí su test con tiempo simulado.

La skill `nuevo-nodo` genera el esqueleto: `render.py maniobra --nombre <nombre> --clase <Clase>`.

## Cómo probarlo

```bash
docker compose run --rm test
```

Los tests de las maniobras usan tiempo simulado: una maniobra de varios segundos se verifica en milisegundos. Los del nodo usan un cliente de acción real contra el servidor, con los tiempos acortados.

Para pedir una maniobra a mano con el nodo corriendo (`ros2 launch bfmc_maneuvers maneuvers.launch.py`):

```bash
ros2 action send_goal /maneuver/execute bfmc_interfaces/action/ManeuverExecute "{maneuver: intersection, direction: left}" --feedback
```

## Estructura

- `strategies/base.py`: el contrato de toda maniobra (`start` y `tick`, con el tiempo como argumento).
- `strategies/legs.py`: la secuencia de tramos, común a las maniobras.
- `strategies/stop.py`, `strategies/intersection.py`: las maniobras.
- `strategies/factory.py`: crea la estrategia que corresponde al nombre pedido.
- `maneuvers_node.py`: el action server. Corre con `MultiThreadedExecutor`, porque el pedido, la cancelación y la ejecución se atienden a la vez.
- `config/params.yaml`, `launch/maneuvers.launch.py` y `test/`.

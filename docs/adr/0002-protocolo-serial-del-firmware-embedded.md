# ADR-0002: El bridge usa el protocolo del firmware de `Bosch-Aura/embedded`

- **Estado:** Propuesta (2026-10-17)
- **Reemplaza:** las secciones 2 a 6 del [ADR-0001](0001-hardware-de-control-y-protocolo-serial.md) (formato, mensajes, watchdog y unidades). La sección 1 (hardware) sigue vigente.
- **Ticket:** A2-146
- **Ajustado por:** el [ADR-0003](0003-el-esp32-es-la-autoridad-ante-obstaculos.md), que decide qué hace el sistema con los eventos de obstáculo.
- **Actualizado:** 2026-11-07, para el firmware de `embedded` `main` (`3f4113a`, 2026-11-06). Esa versión reescribió el firmware: ya no existen `CMD_RECEIVED`, `STATE_AUTO_TRANSITION`, `FAULT_TRIGGERED` ni el estado `ARMED`, solo emite eventos cuando algo cambia y agrega `M:OBSTACLE_RESET`. La versión anterior de este ADR describía el firmware del 2026-10-24 (`440b63a`).

## Contexto

El ADR-0001 definía un protocolo JSON propio (`drive` / `brake` / `ultra` / `status`). Ese firmware nunca llegó al auto: el que corre es el de [`Bosch-Aura/embedded`](https://github.com/Bosch-Aura/embedded), que ya tiene su protocolo, documentado en `docs/BRAIN_TEAM_PROTOCOL.md` de ese repo y usado por AURA-2026. El bridge y el firmware no se entendían.

Se decidió adaptar el bridge y no tocar el firmware.

## Decisión

### 1. Enlace

USB del DevKit, a **115200 baudios**. En la Jetson, el driver de WCH expone el CH340 como `/dev/ttyCH341USB0` (no `/dev/ttyUSB0`). El firmware también escucha por UART en los GPIO 16/17 a 921600; el bridge no la usa.

### 2. Jetson → ESP32

Una línea ASCII por comando: `CANAL:COMANDO:VALOR\n`.

| Comando | Valor | Uso en el bridge |
|---|---|---|
| `C:SET_SPEED` | duty 0–255, siempre ≥ 0. Queda latcheado | En cada ciclo |
| `C:SET_STEER` | servo 50 (izquierda) – 105 (centro) – 160 (derecha) | En cada ciclo |
| `C:SET_DIR` | 1 adelante, 0 atrás | Solo cuando cambia el sentido. El firmware lo recuerda y hace la inversión con su tiempo muerto |
| `E:BRAKE_NOW` | 0 | El bridge no lo manda (el firmware frena solo ante un obstáculo, ADR-0003). Es de un solo uso: el próximo `C:SET_SPEED` vuelve a mover el auto |
| `M:SYS_ARM` | 0 | Al abrir el puerto o si el ESP32 se reinicia, hasta que lo confirma. Y cuando se pide por servicio |
| `M:SYS_DISARM` | 0 | Al cerrar el nodo, después de `C:SET_SPEED:0` |
| `M:SYS_STOP` / `M:SYS_RESUME` | 0 | Por el servicio `/hardware/stop`, y como latido (sección 4) |
| `M:OBSTACLE_RESET` | 0 | Por el servicio `/hardware/obstacle_reset`, o solo, una vez por frenada, si el sensor se queda sin lecturas (ADR-0003) |

Se envía como mucho un comando `M:` por ciclo, con esta prioridad: stop o arranque sin confirmar, armado, reset de obstáculo y latido. El firmware actual procesa varios en orden, pero así cada uno tiene su confirmación sin ambigüedad.

El firmware **no tiene modos de manejo** (`embedded` PR #5 sacó `M:SYS_MODE` y `EVENT:MODE_CHANGED`): si el auto va solo o lo maneja una persona se decide en la Jetson. El bridge no manda ningún modo; arma directamente.

### 3. ESP32 → Jetson

Solo las líneas `EVENT:<NOMBRE>[:<arg>...]` son protocolo; el resto es depuración y se descarta. El bridge usa:

| Evento | Qué hace el bridge |
|---|---|
| `SYSTEM_READY` | La placa se reinició: vuelve a enviar el stop pedido y la arma |
| `STATE_CHANGED:<DISARMED\|RUNNING\|STOPPED\|FAULT>` | Lo publica en `esp32_state` |
| `WATCHDOG_TIMEOUT`, `ESTOP_TRIGGERED:GPIO` | FAULT |
| `CMD_EXECUTED:<SYS_ARM\|SYS_STOP\|SYS_RESUME\|OBSTACLE_RESET>` | Confirma el pedido correspondiente |
| `OBSTACLE_DETECTED:<cm>` / `OBSTACLE_CLEARED` | `obstacle_stop` y `obstacle_distance_m` |
| `ULTRASONIC_SILENT` / `ULTRASONIC_OK` | `ultrasonic_silent` y el destrabe automático (ADR-0003) |

El firmware **solo emite eventos cuando algo cambia**: `CMD_EXECUTED:SET_SPEED` sale cuando cambia la velocidad aplicada, no a cada comando. Con el auto a velocidad constante no dice nada.

### 4. Ciclo, watchdog y latido

El bridge envía velocidad y dirección a 20 Hz aunque no cambien. Eso alimenta el watchdog del firmware:

```
período del bridge 50 ms  <<  watchdog 300 ms
```

El firmware deja la velocidad latcheada (ya no hay TTL de 200 ms). Sin un comando vigente (más viejo que `cmd_timeout_s`), el bridge envía velocidad 0 y dirección al centro.

En el otro sentido, como el firmware no emite nada mientras nada cambia, el bridge necesita provocar una respuesta para saber que el ESP32 sigue vivo. **Latido:** cada `heartbeat_period_s` (0,3 s) reenvía el pedido de stop vigente (`M:SYS_RESUME:0` si el auto puede andar, `M:SYS_STOP:0` si está detenido). El ESP32 ya está en ese estado, así que no cambia nada, pero lo confirma siempre con `CMD_EXECUTED`, en cualquier estado (también en FAULT o DISARMED). `link_up` cae si pasan `link_timeout_s` (1 s) sin ningún evento.

Alternativas descartadas: `M:PING:0` alimenta el watchdog pero no tiene respuesta; `M:SYS_ARM:0` también se confirma siempre, pero sacaría al ESP32 de un FAULT sin que nadie lo pida; un evento periódico en el firmware exigía modificar `embedded`.

### 5. Armado, STOPPED y FAULT

`STOPPED` es un estado **armado pero quieto** del firmware. Se entra por `M:SYS_STOP:0` (frena y congela el servo), que queda enganchado hasta `M:SYS_RESUME:0`, o por un obstáculo del ultrasónico (bloquea solo el avance; el servo obedece y se puede ir marcha atrás). Vuelve solo a `RUNNING` cuando no queda ninguna causa. Para el bridge cuenta como armado: no lo rearma y no es FAULT.

Otros nodos detienen o arrancan el auto con el servicio `/hardware/stop` (`std_srvs/SetBool`): `true` envía `M:SYS_STOP:0` y `false` `M:SYS_RESUME:0`. El bridge guarda el último pedido, lo reintenta cada `arm_retry_s` hasta que llegue `EVENT:CMD_EXECUTED:SYS_STOP`/`SYS_RESUME`, después lo sigue mandando como latido, y si el ESP32 se reinicia (y olvida el stop) se lo vuelve a enviar antes de armarlo. Arrancar no mueve el auto por sí solo: avanza con el próximo `/cmd/drive`, y con un obstáculo delante sigue en `STOPPED`.


El firmware entra en `FAULT` por watchdog o por E-STOP, y **no sale solo**. El bridge arma al abrir el puerto y cuando el ESP32 se reinicia (`SYSTEM_READY`), pero **nunca rearma después de un FAULT**. Para eso está el servicio `/hardware/arm` (`std_srvs/Trigger`). `/hardware/status` publica el FAULT en `watchdog_tripped`.

### 6. Unidades

En ROS todo sigue en SI. Solo el bridge convierte:

- **Velocidad:** `|v|` en m/s se mapea linealmente de `duty_start` (140) a `duty_max` (255) en `max_speed_mps`, y el signo va en `C:SET_DIR`. Por debajo de 140 el motor no arranca desde parado y el firmware no avisa; por eso cualquier velocidad distinta de 0 da al menos 140, igual que en Brain-Aura.
- **Dirección:** el ángulo en rad (positivo a la izquierda, REP-103) se mapea a cada lado del servo por separado, con `max_steer_rad` (30°) en el tope.

Los topes del servo tienen que coincidir con los del firmware (`include/hardware.h`, que puede pisar `Brain-Aura/hardware.env` al compilar).

## Valores asumidos, a verificar en el auto

- `[asumido]` **0,5 m/s equivale al duty 255.** No hay medición de m/s contra duty; el lazo de velocidad es abierto.
- `[por-verificar]` **Que positivo sea izquierda** con el servo real. `hardware.env` marca esa convención como sin verificar (`BRAIN_INVERTIR_DIRECCION`), y AURA-2026 (`brain/lane_detection/angle_converter.py`) usa la opuesta: 50 derecha, 160 izquierda. Se comprueba mandando `C:SET_STEER:50`.
- `[por-verificar]` **30° en el tope del servo.** Es el valor de AURA-2026; Brain-Aura usaba 25° (del auto con Nucleo) y la guía del firmware usa 45° como ejemplo. Hay que medir el ángulo de las ruedas en 50 y en 160.
- `[asumido]` **Latido de 0,3 s y enlace caído a 1 s.**

## Consecuencias

- El bridge habla con el firmware que está flasheado hoy, sin tocar `embedded`.
- **`/hardware/ultrasonic` deja de publicarse:** el firmware no manda lecturas continuas. El frenado por obstáculo lo hace el propio ESP32 (pasa a `STOPPED`) y avisa con `EVENT:OBSTACLE_DETECTED:<cm>` y `EVENT:OBSTACLE_CLEARED`. Qué se hace con esos eventos lo decide el [ADR-0003](0003-el-esp32-es-la-autoridad-ante-obstaculos.md): se elimina `bfmc_safety` y el bridge los informa en `/hardware/status`.
- `HardwareStatus.firmware_version` queda vacío y `last_seq` en 0: el firmware no informa versión ni confirma secuencias.
- Abrir el puerto reinicia el ESP32 en esta Jetson, aunque DTR y RTS queden en reposo. El bridge lo detecta por `SYSTEM_READY` y vuelve a configurar y armar.
- El latido agrega unas 3 líneas cortas por segundo al enlace serie, que a 115200 baudios es despreciable.

## Problemas del firmware vistos al probar con el auto (2026-10-17)

1. **El ultrasónico frenaba siempre:** había un obstáculo real a 22 cm. El sensor funciona.
2. **`WATCHDOG_TIMEOUT` justo después de armar:** era una resta sin signo que daba la vuelta (`Heartbeat age: 4294967271 ms`). Corregido en `embedded` (commit `fa7b3b6`, rama `fix/plan-mejoras-fase0-fase2`).
3. **Dos eventos pegados en una línea:** `Serial.println()` hace dos escrituras y varias tareas imprimían a la vez. Corregido en el mismo commit: cada línea se escribe entera bajo un mutex.

Con esos arreglos, el bridge mantiene el ESP32 en RUNNING, y el watchdog real sigue saltando a los ~300 ms si se corta el enlace.

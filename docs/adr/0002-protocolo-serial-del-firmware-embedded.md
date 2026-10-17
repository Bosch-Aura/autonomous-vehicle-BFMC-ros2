# ADR-0002: El bridge usa el protocolo del firmware de `Bosch-Aura/embedded`

- **Estado:** Propuesta (2026-10-17)
- **Reemplaza:** las secciones 2 a 6 del [ADR-0001](0001-hardware-de-control-y-protocolo-serial.md) (formato, mensajes, watchdog y unidades). La sección 1 (hardware) sigue vigente.
- **Ticket:** A2-146

## Contexto

El ADR-0001 definía un protocolo JSON propio (`drive` / `brake` / `ultra` / `status`). Ese firmware nunca llegó al auto: el que corre es el de [`Bosch-Aura/embedded`](https://github.com/Bosch-Aura/embedded), que ya tiene su protocolo, documentado en `docs/BRAIN_TEAM_PROTOCOL.md` de ese repo y usado por Brain-Aura. El bridge y el firmware no se entendían.

Se decidió adaptar el bridge y no tocar el firmware.

## Decisión

### 1. Enlace

USB del DevKit, a **115200 baudios**. En la Jetson, el driver de WCH expone el CH340 como `/dev/ttyCH341USB0` (no `/dev/ttyUSB0`). El firmware también escucha por UART en los GPIO 16/17 a 921600; el bridge no la usa.

### 2. Jetson → ESP32

Una línea ASCII por comando: `CANAL:COMANDO:VALOR\n`.

| Comando | Valor | Uso en el bridge |
|---|---|---|
| `C:SET_SPEED` | duty 0–255, siempre ≥ 0 | En cada ciclo |
| `C:SET_STEER` | servo 50 (izquierda) – 105 (centro) – 160 (derecha) | En cada ciclo |
| `C:SET_DIR` | 1 adelante, 0 atrás | Solo cuando cambia el sentido. El firmware lo recuerda y hace la inversión con su tiempo muerto |
| `E:BRAKE_NOW` | 0 | Al empezar una emergencia de `/safety/status`. Bloquea la tracción 5 s |
| `M:SYS_MODE` | 1 AUTO, 0 MANUAL | Al abrir el puerto o si el ESP32 se reinicia, hasta que lo confirma |
| `M:SYS_ARM` | 0 | Después del modo, hasta que lo confirma. Y cuando se pide por servicio |
| `M:SYS_DISARM` | 0 | Al cerrar el nodo, después de `C:SET_SPEED:0` |

Se envía **como mucho un comando `M:` por ciclo**: el firmware guarda un solo comando de gestión, y un segundo que llegue antes de que lo procese pisa al primero.

### 3. ESP32 → Jetson

Solo las líneas `EVENT:<NOMBRE>[:<arg>...]` son protocolo; el resto es depuración y se descarta. El bridge usa `STATE_CHANGED`, `STATE_AUTO_TRANSITION`, `MODE_CHANGED`, `CMD_RECEIVED:SYS_MODE`, `CMD_EXECUTED:SYS_ARM`, `SYSTEM_READY`, `WATCHDOG_TIMEOUT`, `FAULT_TRIGGERED` y `ESTOP_TRIGGERED`.

Cualquier evento cuenta como señal de vida. El firmware responde `EVENT:CMD_RECEIVED:SET_SPEED:<v>` a cada velocidad, así que al enviarla en cada ciclo siempre hay tráfico de vuelta.

### 4. Ciclo, TTL y watchdog

El bridge envía velocidad y dirección a 20 Hz aunque no cambien. La jerarquía del firmware queda así:

```
período del bridge 50 ms  <  TTL de los comandos 200 ms  <  watchdog AUTO 300 ms
```

Sin un comando vigente (más viejo que `cmd_timeout_s`), el bridge envía velocidad 0 y dirección al centro. No hace falta `M:PING`.

### 5. Armado y FAULT

El firmware entra en `FAULT` por watchdog, por E-STOP o por un obstáculo del ultrasónico, y **no sale solo**. El bridge configura el modo y arma al abrir el puerto y cuando el ESP32 se reinicia (`SYSTEM_READY`), pero **nunca rearma después de un FAULT**. Para eso está el servicio `/hardware/arm` (`std_srvs/Trigger`). `/hardware/status` publica el FAULT en `watchdog_tripped`.

### 6. Unidades

En ROS todo sigue en SI. Solo el bridge convierte:

- **Velocidad:** `|v|` en m/s se mapea linealmente de `duty_start` (140) a `duty_max` (255) en `max_speed_mps`, y el signo va en `C:SET_DIR`. Por debajo de 140 el motor no arranca desde parado y el firmware no avisa; por eso cualquier velocidad distinta de 0 da al menos 140, igual que en Brain-Aura.
- **Dirección:** el ángulo en rad (positivo a la izquierda, REP-103) se mapea a cada lado del servo por separado, con `max_steer_rad` (25°) en el tope.

Los topes y el umbral tienen que coincidir con `Brain-Aura/hardware.env`, de donde los toma el firmware al compilar.

## Valores asumidos, a verificar en el auto

- `[asumido]` **0,5 m/s equivale al duty 255.** No hay medición de m/s contra duty; el lazo de velocidad es abierto.
- `[por-verificar]` **Que positivo sea izquierda** con el servo real. `hardware.env` marca esa convención como sin verificar (`BRAIN_INVERTIR_DIRECCION`).

## Consecuencias

- El bridge habla con el firmware que está flasheado hoy, sin tocar `embedded`.
- **`/hardware/ultrasonic` deja de publicarse:** el firmware no informa distancias. El frenado por obstáculo lo hace el propio ESP32. `bfmc_safety` va a necesitar otra fuente, o que el firmware publique la lectura.
- `HardwareStatus.firmware_version` queda vacío y `last_seq` en 0: el firmware no informa versión ni confirma secuencias.
- Abrir el puerto reinicia el ESP32 en esta Jetson, aunque DTR y RTS queden en reposo. El bridge lo detecta por `SYSTEM_READY` y vuelve a configurar y armar.

## Problemas del firmware vistos al probar con el auto (2026-10-17)

1. **El ultrasónico frenaba siempre:** había un obstáculo real a 22 cm. El sensor funciona.
2. **`WATCHDOG_TIMEOUT` justo después de armar:** era una resta sin signo que daba la vuelta (`Heartbeat age: 4294967271 ms`). Corregido en `embedded` (commit `fa7b3b6`, rama `fix/plan-mejoras-fase0-fase2`).
3. **Dos eventos pegados en una línea:** `Serial.println()` hace dos escrituras y varias tareas imprimían a la vez. Corregido en el mismo commit: cada línea se escribe entera bajo un mutex.

Con esos arreglos, el bridge mantiene el ESP32 en RUNNING en modo AUTO, y el watchdog real sigue saltando a los ~300 ms si se corta el enlace.

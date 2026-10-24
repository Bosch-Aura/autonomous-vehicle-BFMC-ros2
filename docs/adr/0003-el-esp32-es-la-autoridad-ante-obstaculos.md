# ADR-0003: El ESP32 es la autoridad ante obstáculos y se elimina `bfmc_safety`

- **Estado:** Propuesta (2026-10-24). Falta que la aprueben las tres personas del equipo y probarla en el auto.
- **Ajusta:** el [ADR-0001](0001-hardware-de-control-y-protocolo-serial.md) (la referencia a `bfmc_safety` en el watchdog) y las consecuencias del [ADR-0002](0002-protocolo-serial-del-firmware-embedded.md) (`/hardware/ultrasonic` y `bfmc_safety`).
- **Ticket:** por asignar.

## Contexto

La arquitectura original tenía un nodo `bfmc_safety`: recibía distancias del ultrasónico por `/hardware/ultrasonic`, decidía si había que parar y lo publicaba en `/safety/status`. `bfmc_decision` y `bfmc_hardware_bridge` lo leían.

El firmware que corre en el ESP32 (`Bosch-Aura/embedded`) resuelve el obstáculo por su cuenta: con 3 lecturas válidas a menos de 30 cm frena y pasa a `STOPPED`, y con 3 lecturas a 35 cm o más (o 10 seguidas sin eco) vuelve a `RUNNING`. Avisa con `EVENT:OBSTACLE_DETECTED:<cm>` y `EVENT:OBSTACLE_CLEARED`. **No manda lecturas continuas.**

Sin lecturas, `bfmc_safety` declaraba emergencia (`SENSOR_STALE`) y `bfmc_decision` dejaba el auto detenido para siempre: con el firmware real, la cadena completa no podía moverse.

## Decisión

1. **El ESP32 es la única autoridad ante obstáculos.** Detecta, frena y se libera por su cuenta, con los umbrales de su firmware.
2. **Se elimina `bfmc_safety`** y el mensaje `SafetyStatus`. Tampoco hay `/safety/status` ni `/hardware/ultrasonic`.
3. **El puente informa el obstáculo** en `HardwareStatus`: `obstacle_stop` (verdadero mientras el ESP32 está detenido por un obstáculo) y `obstacle_distance_m` (la distancia a la que lo vio, `NaN` si no la informó). Lo toma de `OBSTACLE_DETECTED`, de `STATE_AUTO_TRANSITION:RUNNING->STOPPED:OBSTACLE` o de ambos, y lo borra con `OBSTACLE_CLEARED` o con cualquier otro cambio de estado. Se publica **al instante** cuando cambia, además de a `status_rate_hz`.
4. **`bfmc_decision` trata `obstacle_stop` como emergencia:** comando en cero y cancelación de la maniobra en curso. Ya no espera ningún estado de seguridad para salir de `STARTING`.
5. **El puente no manda `E:BRAKE_NOW` por un obstáculo:** el firmware ya frenó. El comando sigue existiendo en el protocolo.

## Alternativas descartadas

- **Mantener `bfmc_safety` y alimentarlo con una lectura sintética** armada a partir de los eventos (distancia al detectar, "despejado" al liberarse). Conserva el nodo pero agrega un rodeo: convertir un evento en una distancia y volver a interpretarla con los mismos umbrales que ya aplicó el firmware.
- **Un publicador falso de "camino despejado"** solo para probar. No protege nada del lado de la Jetson.
- **Que el firmware publique lecturas continuas.** Exige tocar `embedded` y reflashear, y no resuelve la duplicación de umbrales.

## Consecuencias

- Menos piezas y sin latencia de serial en la frenada. La parada funciona aunque se caiga el software de la Jetson.
- **No queda un segundo detector de obstáculos en la Jetson.** Si el ultrasónico o el firmware fallan, nada más frena por obstáculo. El watchdog del ESP32 sigue cubriendo la pérdida del enlace.
- Los umbrales (30 y 35 cm, 3 lecturas) viven en el firmware: cambiarlos exige volver a flashear.
- La Jetson conoce la distancia solo en el momento de la detección, no de forma continua.
- Depende de la versión del firmware: los eventos de obstáculo y el estado `STOPPED` son del 24/10/2026 (commit `f9db3f4` de `embedded`). Con un firmware anterior el puente no se entera del obstáculo.
- Tras despejarse, el firmware no arranca solo: pide un `C:SET_SPEED` nuevo. El puente envía la velocidad en cada ciclo, así que el auto retoma apenas `bfmc_decision` vuelve a seguir el carril.
- `HardwareStatus` cambia (dos campos nuevos) y `SafetyStatus` desaparece de `bfmc_interfaces`.

## Por verificar en el auto

- `[por-verificar]` Que el firmware flasheado incluya los eventos de obstáculo (se comprueba con un monitor serie: `OBSTACLE_DETECTED` al acercar una mano a menos de 30 cm).
- `[por-verificar]` El retraso desde que aparece un obstáculo hasta que `bfmc_decision` pone el comando en cero.
- `[por-verificar]` Que una maniobra en curso se cancele ante un obstáculo y no retome desfasada.

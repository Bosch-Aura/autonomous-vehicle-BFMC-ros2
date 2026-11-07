# ADR-0003: El ESP32 es la autoridad ante obstáculos y se elimina `bfmc_safety`

- **Estado:** Propuesta (2026-10-24). Falta que la aprueben las tres personas del equipo y probarla en el auto.
- **Ajusta:** el [ADR-0001](0001-hardware-de-control-y-protocolo-serial.md) (la referencia a `bfmc_safety` en el watchdog) y las consecuencias del [ADR-0002](0002-protocolo-serial-del-firmware-embedded.md) (`/hardware/ultrasonic` y `bfmc_safety`).
- **Ticket:** por asignar.
- **Actualizado:** 2026-11-07, para el firmware de `embedded` `main` (`3f4113a`): cómo se libera el obstáculo y el destrabe automático (decisión 6).

## Contexto

La arquitectura original tenía un nodo `bfmc_safety`: recibía distancias del ultrasónico por `/hardware/ultrasonic`, decidía si había que parar y lo publicaba en `/safety/status`. `bfmc_decision` y `bfmc_hardware_bridge` lo leían.

El firmware que corre en el ESP32 (`Bosch-Aura/embedded`) resuelve el obstáculo por su cuenta: con 3 lecturas válidas seguidas a menos de 30 cm bloquea el avance y pasa a `STOPPED`, y con 3 lecturas válidas a 35 cm o más vuelve a `RUNNING`. Avisa con `EVENT:OBSTACLE_DETECTED:<cm>` y `EVENT:OBSTACLE_CLEARED`. **No manda lecturas continuas.**

Desde el firmware del 2026-11-06, **las lecturas sin eco se ignoran**: no frenan ni sueltan, porque pueden ser espacio abierto, un objeto blando o un sensor desconectado. La versión anterior soltaba con 10 lecturas seguidas sin eco. Consecuencia: si el obstáculo se va de costado y adelante no queda nada a menos de 4 m, el bloqueo no se suelta solo. Para eso el firmware agrega `M:OBSTACLE_RESET:0`, y avisa `EVENT:ULTRASONIC_SILENT` cuando pasan 2 s sin ninguna lectura válida (`EVENT:ULTRASONIC_OK` al volver).

Sin lecturas, `bfmc_safety` declaraba emergencia (`SENSOR_STALE`) y `bfmc_decision` dejaba el auto detenido para siempre: con el firmware real, la cadena completa no podía moverse.

## Decisión

1. **El ESP32 es la única autoridad ante obstáculos.** Detecta, frena y se libera por su cuenta, con los umbrales de su firmware.
2. **Se elimina `bfmc_safety`** y el mensaje `SafetyStatus`. Tampoco hay `/safety/status` ni `/hardware/ultrasonic`.
3. **El puente informa el obstáculo** en `HardwareStatus`: `obstacle_stop` (verdadero mientras el ESP32 bloquea el avance) y `obstacle_distance_m` (la distancia a la que lo vio, `NaN` si no la informó). Lo toma de `OBSTACLE_DETECTED` y lo borra con `OBSTACLE_CLEARED` o si la placa se reinicia. También informa `ultrasonic_silent`. Se publica **al instante** cuando cambia, además de a `status_rate_hz`.
4. **`bfmc_decision` trata `obstacle_stop` como emergencia:** comando en cero y cancelación de la maniobra en curso. Ya no espera ningún estado de seguridad para salir de `STARTING`.
5. **El puente no manda `E:BRAKE_NOW` por un obstáculo:** el firmware ya frenó. El comando sigue existiendo en el protocolo.
6. **Destrabe automático, una vez por frenada.** Si el ESP32 está bloqueado por un obstáculo y llega `ULTRASONIC_SILENT`, el puente manda `M:OBSTACLE_RESET:0` (parámetro `auto_obstacle_reset`, activado). Si el ESP32 vuelve a bloquear antes de `obstacle_reblock_window_s` (3 s), el puente no insiste y el auto queda detenido hasta un destrabe a mano: el servicio `/hardware/obstacle_reset` o el botón del visor. Un despeje del propio firmware o un destrabe a mano vuelven a habilitar el automático.

   Por qué: en una pista abierta, cuando el obstáculo se va no queda nada al alcance del sensor, y sin esto el auto se quedaría esperando a una persona. El reset **no le quita la autoridad al ESP32**: solo borra el bloqueo y el firmware vuelve a mirar. Si el obstáculo sigue ahí, lo detecta y frena en unos 150 ms. La Jetson nunca decide que el camino está libre; solo pide que se vuelva a comprobar, y solo cuando el sensor no tiene ninguna lectura válida (si hubiera algo al alcance, habría lecturas).

## Alternativas descartadas

- **Destrabe solo a mano** (lo que hace AURA-2026 en `feature/hardware_embedded`, con un botón "RESET OBSTACLE"). Es lo más conservador, pero en competencia no hay nadie para apretarlo. Sigue disponible apagando `auto_obstacle_reset`.
- **Volver a soltar con N lecturas sin eco en el firmware.** Es más rápido que esperar 2 s, pero exige modificar `embedded`, que eliminó esa regla a propósito el 2026-11-06.

- **Mantener `bfmc_safety` y alimentarlo con una lectura sintética** armada a partir de los eventos (distancia al detectar, "despejado" al liberarse). Conserva el nodo pero agrega un rodeo: convertir un evento en una distancia y volver a interpretarla con los mismos umbrales que ya aplicó el firmware.
- **Un publicador falso de "camino despejado"** solo para probar. No protege nada del lado de la Jetson.
- **Que el firmware publique lecturas continuas.** Exige tocar `embedded` y reflashear, y no resuelve la duplicación de umbrales.

## Consecuencias

- Menos piezas y sin latencia de serial en la frenada. La parada funciona aunque se caiga el software de la Jetson.
- **No queda un segundo detector de obstáculos en la Jetson.** Si el ultrasónico o el firmware fallan, nada más frena por obstáculo. El watchdog del ESP32 sigue cubriendo la pérdida del enlace.
- **Riesgo del destrabe automático:** un objeto a menos de 2 cm, muy blando o en ángulo tampoco devuelve eco. Si el auto está frenado frente a algo así, o el sensor se desconecta justo en ese momento, el auto avanza hasta que el ESP32 vuelva a bloquear (unos 150 ms, unos 5 cm a 0,3 m/s) o, con el sensor desconectado, sin volver a frenar. Se acota a un solo intento por frenada y el visor muestra el sensor sin lecturas.
- Los umbrales (30 y 35 cm, 3 lecturas) viven en el firmware: cambiarlos exige volver a flashear.
- La Jetson conoce la distancia solo en el momento de la detección, no de forma continua.
- Depende de la versión del firmware: `OBSTACLE_RESET`, `ULTRASONIC_SILENT` y `CMD_EXECUTED` como confirmación son del 06/11/2026 (`embedded` `3f4113a`).
- Tras despejarse, el firmware no arranca solo: pide un `C:SET_SPEED` nuevo. El puente envía la velocidad en cada ciclo, así que el auto retoma apenas `bfmc_decision` vuelve a seguir el carril.
- `HardwareStatus` cambia (dos campos nuevos) y `SafetyStatus` desaparece de `bfmc_interfaces`.

## Por verificar en el auto

- `[por-verificar]` Que el firmware flasheado incluya los eventos de obstáculo (se comprueba con un monitor serie: `OBSTACLE_DETECTED` al acercar una mano a menos de 30 cm).
- `[por-verificar]` El retraso desde que aparece un obstáculo hasta que `bfmc_decision` pone el comando en cero.
- `[por-verificar]` Que una maniobra en curso se cancele ante un obstáculo y no retome desfasada.
- `[por-verificar]` Que, al sacar la mano de costado en pista abierta, el auto siga solo a los ~2 s, y que con la mano todavía delante vuelva a frenar y se quede detenido.

# bfmc_hardware_bridge

Puente entre ROS2 y el ESP32. Es el **único** nodo que abre el puerto serial: ningún otro lo toca. Traduce los comandos de ROS al protocolo del firmware de [`Bosch-Aura/embedded`](https://github.com/Bosch-Aura/embedded) y publica el estado que informa el ESP32. El protocolo está en el [ADR-0002](../../../docs/adr/0002-protocolo-serial-del-firmware-embedded.md).

## Responsabilidad

Hablar con el ESP32. **No decide** nada: reenvía lo que decide `bfmc_decision` (con 0 y dirección al centro si el comando es viejo) e informa lo que el ESP32 reporta, incluido que se detuvo por un obstáculo.

## Topics y servicios

| Dirección | Parámetro | Nombre | Tipo | Descripción |
|---|---|---|---|---|
| Se suscribe | `cmd_topic` | `/cmd/drive` | `ackermann_msgs/AckermannDriveStamped` | Comando final (m/s y rad) |
| Publica | `status_topic` | `/hardware/status` | `bfmc_interfaces/HardwareStatus` | Enlace vivo, si el ESP32 está en FAULT (`watchdog_tripped`) y si está detenido por un obstáculo (`obstacle_stop`, `obstacle_distance_m`) |
| Servicio | `arm_service` | `/hardware/arm` | `std_srvs/Trigger` | Rearma el ESP32 después de un FAULT |
| Servicio | `stop_service` | `/hardware/stop` | `std_srvs/SetBool` | `true`: detener (STOPPED). `false`: arrancar |
| Servicio | `obstacle_reset_service` | `/hardware/obstacle_reset` | `std_srvs/Trigger` | Suelta a mano el bloqueo por obstáculo (`M:OBSTACLE_RESET:0`) |

El ultrasónico lo maneja el propio ESP32: frena solo ante un obstáculo y avisa con `OBSTACLE_DETECTED:<cm>` y `OBSTACLE_CLEARED`. Este nodo **no publica** `/hardware/ultrasonic`; informa el obstáculo y si el sensor está sin lecturas (`ultrasonic_silent`) en `/hardware/status`.

## Cómo funciona

En cada ciclo (`send_rate_hz`, 20 Hz) el nodo lee el puerto, procesa los eventos del ESP32 y envía:

| Situación | Líneas enviadas |
|---|---|
| Comando vigente | `C:SET_DIR` (solo si cambió el sentido), `C:SET_STEER`, `C:SET_SPEED` |
| Sin comandos, o el último tiene más de `cmd_timeout_s` | `C:SET_STEER:<centro>`, `C:SET_SPEED:0` |
| Además, como mucho una línea de gestión | Por prioridad: stop o arranque sin confirmar, `M:SYS_ARM`, `M:OBSTACLE_RESET` y el latido |

Reenviar velocidad y dirección en cada ciclo alimenta el **watchdog** del firmware (300 ms). El firmware deja la velocidad latcheada, así que el puente es quien la baja a 0 cuando el comando de ROS se vuelve viejo.

**Latido:** el firmware solo emite eventos cuando algo cambia; andando a velocidad constante no dice nada. Para saber que sigue vivo, el puente reenvía cada `heartbeat_period_s` (0,3 s) el pedido de stop vigente: `M:SYS_RESUME:0` si el auto puede andar, `M:SYS_STOP:0` si está detenido. El ESP32 ya está así, así que no cambia nada, pero siempre contesta `EVENT:CMD_EXECUTED:SYS_RESUME` (o `SYS_STOP`), en cualquier estado.

**Arranque:** al abrir el puerto, o si el ESP32 se reinicia (`EVENT:SYSTEM_READY`), el nodo arma la placa solo, reintentando hasta que el firmware confirme (`CMD_EXECUTED:SYS_ARM`). En esta Jetson, **abrir el puerto reinicia la placa**; el nodo lo detecta y la vuelve a armar. El firmware no tiene modos AUTO/MANUAL, así que no se configura ninguno.

**STOPPED:** el ESP32 queda armado pero quieto por `M:SYS_STOP:0` (frena y congela el servo) o por un obstáculo (bloquea solo el avance; el servo obedece). Vuelve solo a `RUNNING` cuando no queda ninguna causa. El nodo lo registra como estado `STOPPED`; no es FAULT y no lo rearma.

**Obstáculo:** el nodo lo detecta con `EVENT:OBSTACLE_DETECTED:<cm>` y lo publica en `obstacle_stop` hasta `OBSTACLE_CLEARED` o un reinicio de la placa (el bloqueo del firmware no depende del estado: sobrevive a un FAULT o a desarmar). La distancia va en `obstacle_distance_m` (`NaN` si el firmware no la informó). **El puente no manda ningún freno por esto**: frena el firmware, y `bfmc_decision` decide qué hacer. Al despejarse, el firmware pide un `C:SET_SPEED` nuevo; como el puente envía la velocidad en cada ciclo, el auto retoma apenas `bfmc_decision` vuelva a pedir movimiento.

**Destrabe del obstáculo (ADR-0003):** el firmware solo suelta el bloqueo con 3 lecturas válidas a 35 cm o más. Si el obstáculo se va de costado y adelante no queda nada al alcance del sensor (4 m), no llegan lecturas válidas y el auto queda detenido; a los 2 s el firmware avisa `EVENT:ULTRASONIC_SILENT`. Con `auto_obstacle_reset: true`, el puente manda entonces `M:OBSTACLE_RESET:0` **una sola vez por frenada**. Si el obstáculo seguía ahí, el ESP32 vuelve a bloquear en unos 150 ms; si eso pasa antes de `obstacle_reblock_window_s`, el puente no insiste y el auto queda detenido hasta que alguien llame a `/hardware/obstacle_reset` (o el botón "Destrabar obstáculo" del visor). Un despeje del propio firmware o un destrabe a mano vuelven a habilitar el automático.

```bash
ros2 service call /hardware/obstacle_reset std_srvs/srv/Trigger
```

**Arranque detenido:** con `start_stopped: true` (el valor del YAML) el puente le pide `M:SYS_STOP:0` al ESP32 apenas lo arma, y se lo vuelve a pedir si la placa se reinicia. El auto no se mueve hasta que alguien llame a `/hardware/stop` con `false`. Con `false` en el parámetro, el auto arranca y se mueve con el primer `/cmd/drive`.

**Detener y arrancar desde otro nodo:** el servicio `/hardware/stop` (`std_srvs/SetBool`) deja el ESP32 en `STOPPED` con `true` (`M:SYS_STOP:0`, queda enganchado) y lo suelta con `false` (`M:SYS_RESUME:0`). El nodo reintenta hasta que el ESP32 confirme y, si la placa se reinicia, le vuelve a enviar el stop. Arrancar no mueve el auto: avanza con el próximo `/cmd/drive`.

```bash
ros2 service call /hardware/stop std_srvs/srv/SetBool "{data: true}"    # detener
ros2 service call /hardware/stop std_srvs/srv/SetBool "{data: false}"   # arrancar
```

**FAULT:** el ESP32 entra en FAULT por watchdog o E-STOP, frena y no sale solo. El nodo lo publica en `watchdog_tripped` y lo registra como error, pero **nunca rearma solo**. Para seguir:

```bash
ros2 service call /hardware/arm std_srvs/srv/Trigger
```

**Unidades:** dentro de ROS todo es SI. La conversión ocurre solo acá (`units.py`):

- La velocidad se mapea de `duty_start` a `duty_max` y el signo va en `C:SET_DIR`. Cualquier velocidad distinta de 0 da al menos `duty_start`, porque por debajo el motor no arranca desde parado.
- La dirección (positivo a la izquierda) se mapea a cada lado del servo por separado, con el tope en `max_steer_rad`.

Un valor `NaN` o infinito se trata como 0.

**Enlace:** `link_up` es verdadero mientras lleguen eventos del ESP32 dentro de `link_timeout_s` (1 s). El latido garantiza una respuesta cada 0,3 s. La depuración (`[Control] ...`) no cuenta como señal de vida.

**Si el puerto falla** (cable desconectado), el nodo lo cierra, informa el enlace caído y reintenta abrirlo cada `reconnect_period_s`. **Al cerrar el nodo** envía velocidad 0 y desarma.

## Parámetros

Viven en [`config/params.yaml`](config/params.yaml). El nodo no tiene valores por defecto en el código: si falta uno, falla al arrancar.

| Parámetro | Unidad | Descripción |
|---|---|---|
| `serial_port`, `baud_rate` | —, baudios | USB del ESP32: `/dev/ttyCH341USB0` a 115200 en esta Jetson |
| `send_rate_hz` | Hz | Frecuencia del ciclo. Su período tiene que quedar bien por debajo del watchdog de 300 ms del firmware |
| `arm_retry_s` | s | Espera entre reintentos de armar, detener, arrancar o destrabar mientras el ESP32 no confirme |
| `heartbeat_period_s` | s | Cada cuánto se reenvía el stop vigente como latido. Menor que `link_timeout_s`. **Asumido** (0,3 s) |
| `auto_obstacle_reset` | bool | Si el puente destraba solo un obstáculo cuando el sensor se queda sin lecturas |
| `obstacle_reblock_window_s` | s | Si el ESP32 vuelve a bloquear antes de esto tras un destrabe automático, no se insiste. **Asumido** (3 s) |
| `start_stopped` | bool | Si el auto arranca detenido hasta que se pida arrancar en `/hardware/stop` |
| `reconnect_period_s` | s | Espera entre intentos de reabrir el puerto |
| `cmd_timeout_s` | s | Antigüedad máxima de un comando. **Asumido** (0,25 s) |
| `link_timeout_s` | s | Tiempo sin eventos para dar el enlace por caído. **Asumido** (1 s) |
| `max_speed_mps` | m/s | Velocidad que corresponde a `duty_max`. **Asumido**: no hay calibración |
| `max_steer_rad` | rad | Ángulo de las ruedas con el servo en un tope (50 o 160). **Por verificar** (30°) |
| `duty_start`, `duty_max` | duty | Umbral de arranque y máximo del motor |
| `servo_left`, `servo_center`, `servo_right` | servo | Topes del servo |
| `*_topic`, `*_service`, `status_frame_id`, `status_rate_hz` | — | Nombres y frecuencia del estado |

Los topes del servo (`servo_left`, `servo_center`, `servo_right`) tienen que coincidir con los del firmware (`include/hardware.h`, o `Brain-Aura/hardware.env` si existe). `max_steer_rad` no lo usa el firmware: es la equivalencia entre el tope del servo y el ángulo real de las ruedas, y hay que medirlo en el auto. También hay que comprobar que el servo en 50 gire hacia la izquierda.

## Cómo usarlo en el auto

Con el ESP32 conectado por USB y el firmware de `embedded` cargado:

```bash
ros2 launch bfmc_hardware_bridge hardware_bridge.launch.py
ros2 topic echo /hardware/status
```

`link_up: true` indica que el ESP32 responde, `watchdog_tripped: false`, que no está en FAULT, `obstacle_stop: false`, que no está detenido por un obstáculo, y `ultrasonic_silent`, si el sensor está sin lecturas. Para usar otro puerto sin editar el YAML:

```bash
ros2 run bfmc_hardware_bridge hardware_bridge --ros-args --params-file config/params.yaml -p serial_port:=/dev/ttyUSB0
```

Si dice que no tiene permiso sobre el puerto: `sudo usermod -aG dialout $USER` y volver a iniciar sesión. Ningún otro programa (Brain-Aura, un monitor serie) puede tener el puerto abierto al mismo tiempo.

## Cómo probarlo

```bash
docker compose run --rm test
```

Los tests no necesitan el ESP32: el nodo se prueba con un puerto simulado que hace de ESP32, y la clase del puerto real, con la URL `loop://` de pyserial. Que el ESP32 de verdad responda solo se comprueba con el auto.

Para correr solo este paquete dentro del contenedor (`docker compose run --rm dev`):

```bash
cd ros2_ws
colcon build --symlink-install --packages-up-to bfmc_hardware_bridge
colcon test --packages-select bfmc_hardware_bridge --event-handlers console_direct+
```

## Estructura

- `protocol.py`: arma los comandos y lee los eventos del firmware. Sin ROS ni puerto.
- `line_buffer.py`: arma líneas a partir de los bytes que llegan en trozos.
- `units.py`: conversión de unidades SI al formato del ESP32.
- `hardware_bridge_logic.py`: qué enviar, cuándo armar y qué estado informa el ESP32. El tiempo entra como argumento.
- `serial_port.py`: el puerto, inyectable. Trae la implementación real con pyserial.
- `hardware_bridge_node.py`: conecta todo con ROS.
- `config/params.yaml`, `launch/hardware_bridge.launch.py` y `test/`.

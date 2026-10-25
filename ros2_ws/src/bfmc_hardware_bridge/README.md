# bfmc_hardware_bridge

Puente entre ROS2 y el ESP32. Es el **único** nodo que abre el puerto serial: ningún otro lo toca. Traduce los comandos de ROS al protocolo del ESP32 y publica lo que el ESP32 reporta. El protocolo está en el [ADR-0001](../../../docs/adr/0001-hardware-de-control-y-protocolo-serial.md).

## Responsabilidad

Hablar con el ESP32. **No decide** nada: reenvía lo que decide `bfmc_decision`, salvo que frena si `bfmc_safety` lo pide o si el comando es viejo.

## Topics

| Dirección | Parámetro | Topic | Tipo | Descripción |
|---|---|---|---|---|
| Se suscribe | `cmd_topic` | `/cmd/drive` | `ackermann_msgs/AckermannDriveStamped` | Comando final (m/s y rad) |
| Se suscribe | `safety_topic` | `/safety/status` | `bfmc_interfaces/SafetyStatus` | Si indica emergencia, el puente frena |
| Publica | `ultrasonic_topic` | `/hardware/ultrasonic` | `sensor_msgs/Range` | Distancia al obstáculo (calidad *best effort*) |
| Publica | `status_topic` | `/hardware/status` | `bfmc_interfaces/HardwareStatus` | Estado del enlace y del watchdog del ESP32 |

## Cómo funciona

En cada ciclo (`send_rate_hz`) el nodo lee el puerto, procesa lo recibido y envía **una trama**:

| Situación | Trama enviada |
|---|---|
| Comando vigente y sin emergencia | `drive` con el último comando, convertido y recortado |
| `bfmc_safety` indica emergencia | `brake` |
| Sin comandos, o el último tiene más de `cmd_timeout_s` | `brake` |

Reenviar en cada ciclo hace de **heartbeat**: el watchdog del ESP32 (300 ms) corta los motores si dejan de llegar tramas. El puente nunca queda en silencio con el auto en marcha.

**Unidades:** dentro de ROS todo es SI. La conversión a mm/s y décimas de grado ocurre solo acá (`units.py`) y se recorta a `max_speed_mps` y `max_steer_rad`. Un valor `NaN` o infinito se trata como 0.

**Ultrasónico:** una lectura válida se publica en metros. Si el ESP32 informa `valid:false` (sin eco), se publica `+inf`: **camino despejado**, la convención de `sensor_msgs/Range`. `bfmc_safety` lo interpreta así y no frena.

**Enlace:** `link_up` es verdadero mientras lleguen mensajes válidos del ESP32 dentro de `link_timeout_s`. Una línea que no se puede interpretar se descarta y **no** cuenta como señal de vida.

**Si el puerto falla** (cable desconectado), el nodo lo cierra, informa el enlace caído y reintenta abrirlo cada `reconnect_period_s`.

## Parámetros

Viven en [`config/params.yaml`](config/params.yaml). El nodo no tiene valores por defecto en el código: si falta uno, falla al arrancar.

| Parámetro | Unidad | Descripción |
|---|---|---|
| `serial_port` | — | Puerto del ESP32. **Por verificar**: depende de cómo se conecte |
| `baud_rate` | baudios | **Asumido** (115200, ADR-0001). Debe coincidir con el firmware |
| `send_rate_hz` | Hz | Frecuencia de las tramas (heartbeat) |
| `reconnect_period_s` | s | Espera entre intentos de reabrir el puerto |
| `cmd_timeout_s` | s | Antigüedad máxima de un comando. **Asumido** (0,25 s) |
| `link_timeout_s` | s | Tiempo sin mensajes para dar el enlace por caído. **Asumido** (0,5 s) |
| `max_speed_mps`, `max_steer_rad` | m/s, rad | Límites de los comandos (ADR-0001). **Asumidos** |
| `ultrasonic_min_range_m`, `ultrasonic_max_range_m`, `ultrasonic_fov_rad` | m, rad | Alcance y apertura del sensor, para `sensor_msgs/Range`. **Asumidos** |
| `*_topic`, `*_frame_id`, `status_rate_hz` | — | Topics, frames y frecuencia del estado |

## Cómo usarlo en el auto

Con el ESP32 conectado y el firmware cargado:

```bash
ros2 launch bfmc_hardware_bridge hardware_bridge.launch.py
ros2 topic echo /hardware/status
```

`link_up: true` indica que el ESP32 responde. Para usar otro puerto sin editar el YAML:

```bash
ros2 run bfmc_hardware_bridge hardware_bridge --ros-args --params-file config/params.yaml -p serial_port:=/dev/ttyACM0
```

Si dice que no tiene permiso sobre el puerto: `sudo usermod -aG dialout $USER` y volver a iniciar sesión.

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

- `protocol.py`: arma y lee las tramas del ADR-0001. Sin ROS ni puerto.
- `line_buffer.py`: arma líneas a partir de los bytes que llegan en trozos.
- `units.py`: conversión de unidades SI al formato del ESP32.
- `hardware_bridge_logic.py`: qué trama enviar y qué reporta el ESP32. El tiempo entra como argumento.
- `serial_port.py`: el puerto, inyectable. Trae la implementación real con pyserial.
- `hardware_bridge_node.py`: conecta todo con ROS.
- `config/params.yaml`, `launch/hardware_bridge.launch.py` y `test/`.

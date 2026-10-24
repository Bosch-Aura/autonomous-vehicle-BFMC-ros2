# bfmc_interfaces

Mensajes y acción propios del sistema. Es el único paquete que define tipos: los demás nodos se comunican por mensajes y nunca importan código de otro paquete. Siempre que alcanza, se usan tipos estándar (`sensor_msgs`, `ackermann_msgs`) y no se definen propios.

## Convenciones

- Unidades SI: metros, radianes, segundos, m/s.
- Marco del vehículo: x hacia adelante, y hacia la izquierda. Un ángulo o un desvío **positivo** es hacia la **izquierda**.
- Un valor desconocido de tipo `float32` se representa con `NaN`.

## Mensajes

| Mensaje | Lo publica | Lo consumen | Para qué sirve |
|---|---|---|---|
| `LaneState` | `bfmc_lane_detection` | `bfmc_control`, `bfmc_decision` | Desvío lateral y de rumbo del carril, confianza e intersección detectada |
| `SignDetection` | (parte de `SignDetectionArray`) | — | Una señal: tipo, confianza, distancia y caja en píxeles |
| `SignDetectionArray` | `bfmc_sign_detection` | `bfmc_decision` | Señales detectadas en un cuadro |
| `HardwareStatus` | `bfmc_hardware_bridge` | `bfmc_decision` | Estado del enlace con el ESP32, de su watchdog y el estado que reporta (`esp32_state`) y si está detenido por un obstáculo (y a qué distancia lo vio) |
| `VehicleState` | `bfmc_decision` | dashboard | Estado de la máquina de estados y maniobra activa |

## Acción

`ManeuverExecute`: `bfmc_decision` pide una maniobra y `bfmc_maneuvers` la ejecuta.

| Parte | Campos |
|---|---|
| Meta | `maneuver` (`"stop"` o `"intersection"`), `direction` (`"left"`, `"right"` o `"straight"`, solo para intersección) |
| Resultado | `success`, `message` |
| Realimentación | `elapsed_s` |

## Cómo probarlo

Desde la raíz del repositorio, con Docker:

```bash
docker compose run --rm test
```

Los tests de [`test/test_interfaces.py`](test/test_interfaces.py) verifican los campos, los tipos y las constantes de cada mensaje. Si cambiás un mensaje, el test falla a propósito: avisá a los paquetes que lo usan.

## Cómo agregar o cambiar un mensaje

1. Editá el `.msg` y agregalo a `CMakeLists.txt` si es nuevo.
2. Documentá cada campo con su unidad y su convención de signo.
3. Actualizá el test y la tabla de este README.
4. Revisá los paquetes que lo consumen.

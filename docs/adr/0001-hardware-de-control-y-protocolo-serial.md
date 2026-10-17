# ADR-0001: Hardware de control y protocolo serial Jetson ↔ ESP32

- **Estado:** Aprobada (2026-10-11)
- **Contexto de decisión:** arquitectura ROS2 sobre el template del repositorio (ver [ARQUITECTURA.md](../ARQUITECTURA.md))

## Contexto

La versión anterior del proyecto (AURA-2026) tenía hardware y documentación contradictorios: el código usaba el protocolo `#steer:..;;`, el ultrasónico llegaba por otro puerto, y los docs describían un ESP32 a 921600 baud con comandos `CONTROL:SET_*`. Varios hilos escribían al serial al mismo tiempo.

El equipo confirmó que el auto usa **un solo ESP32**. El firmware que había entonces en este repo era un placeholder, así que el protocolo se define desde cero. Hoy el firmware del ESP32 vive en el repo [`Bosch-Aura/embedded`](https://github.com/Bosch-Aura/embedded).

## Decisión

### 1. Hardware

- **Jetson Orin Nano** (Ubuntu 22.04, ROS2 Humble): percepción, decisión y control.
- **Un ESP32**: motores (vía puente H), dirección y sensor ultrasónico. Es el único microcontrolador del sistema.
- **Un único enlace serial** entre ambos. Solo `bfmc_hardware_bridge` abre ese puerto; ningún otro nodo escribe ni lee de él.

### 2. Formato

Líneas de texto UTF-8, un objeto JSON por línea, terminadas en `\n` (NDJSON), en ambos sentidos. Se elige por ser legible en un monitor serial y porque el sensor ultrasónico del proyecto anterior ya usaba este formato. Las líneas que no se pueden interpretar se descartan sin romper el enlace.

### 3. Mensajes Jetson → ESP32

| Mensaje | Campos | Significado |
|---|---|---|
| `drive` | `seq`, `v` (mm/s, entero), `s` (décimas de grado, entero) | Velocidad y dirección deseadas |
| `brake` | `seq` | Freno inmediato |

`v` positivo es avance y negativo es marcha atrás. `s` positivo es giro a la izquierda.

### 4. Mensajes ESP32 → Jetson

| Mensaje | Campos | Frecuencia |
|---|---|---|
| `ultra` | `id` (`"front"`), `dist_cm`, `valid` | 20 Hz |
| `status` | `fw` (versión), `wd` (si el watchdog cortó los motores), `last_seq` | 2 Hz y ante cada evento |

### 5. Watchdog y heartbeat

- No hay mensaje de heartbeat aparte: **los mensajes `drive` a 20 Hz o más funcionan como heartbeat**. Si no hay tráfico, el bridge reenvía el último `drive` válido.
- Si el ESP32 no recibe ningún mensaje válido en **300 ms**, corta los motores y centra la dirección. Retoma al recibir el siguiente `drive`.
- Del lado Jetson, si el bridge deja de recibir `ultra` por más de 300 ms, deja de publicar `/hardware/ultrasonic` y publica `/hardware/status` con el enlace como caído. `bfmc_safety` trata esa ausencia como lectura inválida.

### 6. Unidades

En ROS todo es SI (m/s, rad). **Solo el bridge convierte** al formato del ESP32 (mm/s y décimas de grado). El ESP32 aplica los límites físicos y recorta cualquier valor fuera de rango.

## Valores asumidos, a verificar antes de implementar

- `[asumido]` **115200 baud.** Es el valor que usaba el enlace del ultrasónico en AURA-2026. Verificar que alcance para `drive` + `ultra` a 20 Hz (≈ 3 KB/s).
- `[asumido]` **Rango de velocidad ±500 mm/s y de dirección ±25,0°.** Salen del código anterior; confirmar con el auto actual.
- `[por-verificar]` Puerto serial de la Jetson (`/dev/ttyTHS1` o USB) y cómo se conecta el ESP32.
- `[por-verificar]` Que motores, dirección y ultrasónico cuelguen del mismo ESP32.
- `[asumido]` **Watchdog de 300 ms.** Es el orden de magnitud del `stale_s` del proyecto anterior; ajustar tras medir la latencia real.

## Consecuencias

- Se elimina la condición de carrera sobre el puerto: un solo escritor.
- Si se cae la Jetson, el auto se detiene solo por el watchdog del ESP32, sin depender de software de alto nivel.
- El protocolo de AURA-2026 (`#steer`, `#speed`, `#brake`, `#alive`) deja de usarse.
- El firmware y el bridge se escriben contra este contrato. Cualquier cambio de campos se registra en un ADR nuevo.

## Alternativas descartadas

- **Heredar el protocolo `#key:valor;;` de AURA-2026.** Usaba unidades mezcladas (0–255, mm/s, servo 50–160).
- **Protocolo binario con CRC.** Más eficiente, pero más difícil de depurar. Se reconsidera si el enlace se satura.
- **micro-ROS en el ESP32.** Evita el bridge, pero añade una curva de aprendizaje alta y una dependencia grande sobre un firmware que todavía no existe.

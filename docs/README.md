# docs

Documentación del proyecto AURA — Bosch Future Mobility Challenge.

## Índice

- [`ARQUITECTURA.md`](ARQUITECTURA.md): arquitectura del sistema, paquetes, topics y reglas de diseño.
- [`adr/`](adr/): decisiones de arquitectura numeradas.
  - [ADR-0001](adr/0001-hardware-de-control-y-protocolo-serial.md): hardware de control y protocolo serial Jetson ↔ ESP32 (el protocolo quedó reemplazado por el ADR-0002).
  - [ADR-0002](adr/0002-protocolo-serial-del-firmware-embedded.md): el bridge usa el protocolo del firmware de `Bosch-Aura/embedded`.

## Qué va acá

- Arquitectura del sistema (diagrama de nodos ROS2, flujo de datos)
- Guías de setup del entorno (Jetson, ESP32, dependencias)
- Decisiones de diseño y ADRs
- Manual de uso del vehículo
- Notas de la competencia

## Convención

Los documentos técnicos importantes van en Markdown (`.md`) para que sean versionables y legibles en GitHub.

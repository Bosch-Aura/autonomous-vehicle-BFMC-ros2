# frontend

Dashboard de monitoreo y control del vehículo.

El stack tecnológico se define en un ticket aparte. La comunicación con ROS2 será exclusivamente por mensajes (sin imports directos de código).

## Qué va acá

- Interfaz web o de escritorio para visualizar el estado del vehículo en tiempo real
- Controles manuales de emergencia
- Visualización de sensores y cámara

## Qué hay hoy

- [`visor-calibracion/`](visor-calibracion/README.md): una página provisoria de solo lectura para calibrar `bfmc_lane_detection` (imagen de la cámara y datos del carril). No es el dashboard.

## Pendiente

- Definir framework (falta crear el ticket en Jira)
- Definir protocolo de mensajes con ROS2

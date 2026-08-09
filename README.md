# Autonomous Vehicle BFMC ROS2

Vehículo autónomo a escala desarrollado por el equipo **AURA** (Universidad Austral) para la competencia **Bosch Future Mobility Challenge (BFMC) 2025-2026**.

## Descripción

Sistema de navegación autónoma para un vehículo a escala 1:10, construido sobre ROS2, con una Jetson como unidad de procesamiento principal y un ESP32 encargado del control de bajo nivel (motores y sensor ultrasónico).

## Estructura del repositorio

```
autonomous-vehicle-BFMC-ros2/
├── jetson/
│   └── ros2_ws/
│       └── src/          # Paquetes ROS2
├── esp32/
│   ├── main/              # Firmware principal del ESP32
│   └── tests_componentes/ # Sketches de prueba individuales por componente
├── flash.sh                # Script de flasheo automático del ESP32
└── docs/                   # Documentación del proyecto
```

## Hardware

- Jetson (unidad de procesamiento principal, corre ROS2)
- ESP32 (control de motores y sensor ultrasónico)
- Puente H
- Sensor ultrasónico
- Cámara de profundidad RealSense

## Equipo

Equipo AURA — Universidad Austral

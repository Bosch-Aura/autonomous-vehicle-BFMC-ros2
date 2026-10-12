# Autonomous Vehicle BFMC ROS2

Vehículo autónomo a escala desarrollado por el equipo **AURA** (Universidad Austral) para la competencia **Bosch Future Mobility Challenge (BFMC) 2025-2026**.

## Descripción

Sistema de navegación autónoma para un vehículo a escala 1:10. La Jetson Orin Nano corre ROS2 y maneja la percepción y planificación. El ESP32 se encarga del control de bajo nivel (motores y sensor ultrasónico).

## Estructura del repositorio

```
autonomous-vehicle-BFMC-ros2/
├── ros2_ws/src/          # Paquetes ROS2
├── firmware/esp32/       # Firmware del ESP32
├── frontend/             # Dashboard de monitoreo (stack por definir)
├── docs/                 # Documentación
└── .github/              # CI y configuración de GitHub
```

## Cómo clonar y configurar

```bash
git clone https://github.com/tu-org/autonomous-vehicle-BFMC-ros2.git
cd autonomous-vehicle-BFMC-ros2

# Instalar dependencias ROS2
source /opt/ros/humble/setup.bash
cd ros2_ws
rosdep install --from-paths src --ignore-src -r -y
```

## Cómo correr los tests

```bash
cd ros2_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install
colcon test
colcon test-result --verbose
```

## Flujo de trabajo

1. Crear rama desde `main` con el formato `tipo/A<n>-<número>-descripcion`
   - Ejemplo: `feature/A2-12-seguir-carril`
2. Hacer commits con el ticket en cada mensaje: `A2-12: descripción`
3. Abrir PR hacia `main` completando el template (incluye el link al ticket de Jira)
4. El PR dispara dos checks automáticos en GitHub Actions:
   - **ROS2 Tests**: compila y corre los tests
   - **Jira Conventions**: verifica la rama y los commits (los commits de merge se ignoran)
5. Esperar al menos una aprobación de una Code Owner
6. Mergear a `main`

No se puede pushear directo a `main` (requiere PR aprobado).

## Configurar protección de `main` (paso manual)

Ir a **Settings → Rules → Rulesets** en GitHub y crear una regla sobre la rama `main`:

- [x] Require a pull request before merging
  - Required approvals: **1**
  - [x] Require review from Code Owners
- [x] Require status checks to pass
  - Agregar: `test` (workflow `ROS2 Tests`)
  - Agregar: `check` (workflow `Jira Conventions`)
- [x] Block force pushes
- [x] Restrict deletions

> En planes gratuitos esto solo funciona si el repositorio es **público**.

## Hardware

- Jetson Orin Nano (Ubuntu 22.04, ROS2 Humble)
- ESP32 (control de motores y sensor ultrasónico)
- Puente H
- Sensor ultrasónico HC-SR04
- Cámara de profundidad Intel RealSense

## Equipo

Equipo AURA — Universidad Austral

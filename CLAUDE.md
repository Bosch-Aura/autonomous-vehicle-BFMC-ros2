# AURA — Autonomous Vehicle BFMC ROS2

Vehículo autónomo a escala desarrollado por el equipo **AURA** (Universidad Austral) para la competencia **Bosch Future Mobility Challenge (BFMC) 2025-2026**.

Hardware principal: Jetson Orin Nano (Ubuntu 22.04 / ROS2 Humble) + ESP32 (control de bajo nivel).

## Estructura del repositorio

```
autonomous-vehicle-BFMC-ros2/
├── ros2_ws/
│   └── src/                  # Paquetes ROS2 (uno por funcionalidad)
│       └── bfmc_example/     # Plantilla de paquete
├── firmware/
│   └── esp32/                # Firmware Arduino (motores, sensores)
├── frontend/                 # Dashboard (stack por definir)
├── docs/                     # Documentación del proyecto
├── .github/
│   ├── workflows/            # GitHub Actions (CI, checks de convenciones)
│   ├── CODEOWNERS            # Revisores automáticos
│   └── pull_request_template.md
├── CLAUDE.md                 # Este archivo
└── README.md
```

Cada carpeta de primer nivel es independiente. La comunicación entre frontend y ROS2 es solo por mensajes.

## Convenciones de ramas

```
tipo/A<n>-<número>-descripcion
```

Donde `tipo` es uno de: `feature`, `fix`, `chore`, `docs`, `test`, `refactor`.

Ejemplo: `feature/A2-12-seguir-carril`

## Convenciones de commits

Cada mensaje de commit debe incluir el ticket de Jira:

```
A<n>-<número>: descripción corta en imperativo
```

Ejemplo: `A2-12: agregar nodo de seguimiento de carril`

## Cómo compilar y testear

```bash
cd ros2_ws
source /opt/ros/humble/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
colcon test
colcon test-result --verbose
```

## Checks automáticos (GitHub Actions)

Cada PR hacia `main` corre dos checks:

- **ROS2 Tests**: compila el workspace y corre los tests con `colcon`. Falla si algún test está roto.
- **Jira Conventions**: verifica que la rama y todos los commits (salvo los de merge) incluyan el ticket de Jira. Falla con un mensaje que explica el formato correcto.

## Versión de ROS2

**Humble** (Ubuntu 22.04). Confirmar en la Jetson con `cat /etc/os-release` — si dice 24.04, usar Jazzy.

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
│       ├── flash.sh          # Compila y sube sketches con arduino-cli
│       ├── main/             # Firmware principal
│       └── tests_componentes/  # Sketches de prueba por componente
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

## Arquitectura

La fuente única es [`docs/ARQUITECTURA.md`](docs/ARQUITECTURA.md) (capas, paquetes `bfmc_*`, topics y prioridades). Las decisiones puntuales están en `docs/adr/`. Leelos antes de agregar o cambiar un nodo.

Reglas que respeta todo nodo:

- Un nodo, una responsabilidad. Los paquetes se comunican solo por mensajes y nunca importan código de otro paquete.
- Percepción y seguridad publican hechos; solo `bfmc_decision` decide y es el único que publica `/cmd/drive`.
- Solo `bfmc_hardware_bridge` toca el puerto serial. El protocolo con el ESP32 está en `docs/adr/0001-hardware-de-control-y-protocolo-serial.md`.
- Unidades SI dentro de ROS (m/s, rad, m). La conversión al formato del ESP32 ocurre únicamente en el bridge.
- Nada hardcodeado: umbrales, ganancias, tiempos y nombres de topics van en `config/params.yaml`.
- La lógica es pura y recibe el tiempo como argumento: sin `time.sleep` ni lectura del reloj dentro de la lógica.
- Cada paquete trae su `README.md` con topics, parámetros y cómo probarlo.

Las skills del proyecto viven en `.claude/skills/`. Usalas al agregar nodos o tests.

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

Con Docker (funciona en cualquier sistema operativo y reproduce el CI):

```bash
docker compose run --rm test
```

Directamente en una máquina con ROS2 Humble:

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

## Cómo subir firmware al ESP32

```bash
cd firmware/esp32
./flash.sh [sketch] [puerto]   # sin argumentos sube main/main.ino
```

Usa el soporte ESP32 versión 3.x de arduino-cli: la API de PWM es `ledcAttach(pin, freq, bits)` y `ledcWrite(pin, duty)` (no `ledcSetup`/`ledcAttachPin`).

## Versión de ROS2

**Humble** (Ubuntu 22.04). Confirmar en la Jetson con `cat /etc/os-release` — si dice 24.04, usar Jazzy.

# ros2_ws

Workspace de ROS2. Contiene todos los paquetes del sistema de navegación autónoma.

## Estructura

```
ros2_ws/
└── src/          # Paquetes ROS2 (uno por funcionalidad)
    └── bfmc_example/   # Paquete de ejemplo / plantilla
```

## Cómo compilar

```bash
cd ros2_ws
source /opt/ros/humble/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
```

## Cómo correr los tests

```bash
cd ros2_ws
source /opt/ros/humble/setup.bash
colcon test
colcon test-result --verbose
```

## Convención de paquetes

Cada paquete tiene una sola responsabilidad (un nodo o un grupo de nodos relacionados). El nombre sigue el formato `bfmc_<funcionalidad>` en snake_case.

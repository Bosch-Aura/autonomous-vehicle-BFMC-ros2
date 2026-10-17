# Entorno de verificación del workspace ROS2: Ubuntu 22.04 + ROS2 Humble.
# Reproduce los pasos del workflow "ROS2 Tests" para que cualquier persona del
# equipo pueda compilar y testear igual que el CI, sin importar su sistema operativo.
# Incluye los mismos complementos de flake8 que el CI: sin ellos, un error de estilo
# (orden de imports, comillas, docstrings) pasa acá y recién falla en GitHub.
FROM ros:humble-ros-base

ENV DEBIAN_FRONTEND=noninteractive \
    PYTHONDONTWRITEBYTECODE=1 \
    PYTEST_ADDOPTS="-p no:cacheprovider"

RUN apt-get update \
    && apt-get install -y --no-install-recommends \
        python3-colcon-common-extensions \
        python3-pytest \
        python3-rosdep \
        python3-flake8-blind-except \
        python3-flake8-builtins \
        python3-flake8-class-newline \
        python3-flake8-comprehensions \
        python3-flake8-deprecated \
        python3-flake8-docstrings \
        python3-flake8-import-order \
        python3-flake8-quotes \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /workspace

# Entorno de verificación del workspace ROS2: Ubuntu 22.04 + ROS2 Humble.
# Reproduce los pasos del workflow "ROS2 Tests" para que cualquier persona del
# equipo pueda compilar y testear igual que el CI, sin importar su sistema operativo.
FROM ros:humble-ros-base

ENV DEBIAN_FRONTEND=noninteractive \
    PYTHONDONTWRITEBYTECODE=1 \
    PYTEST_ADDOPTS="-p no:cacheprovider"

RUN apt-get update \
    && apt-get install -y --no-install-recommends \
        python3-colcon-common-extensions \
        python3-pytest \
        python3-rosdep \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /workspace

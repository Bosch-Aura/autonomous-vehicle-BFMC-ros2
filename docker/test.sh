#!/usr/bin/env bash
# Instala dependencias, compila y testea el workspace.
# Son los mismos pasos que el workflow "ROS2 Tests" (.github/workflows/ros2-tests.yml).
# Sin `-u`: el setup.bash de ROS usa variables sin definir.
set -eo pipefail

source /opt/ros/humble/setup.bash
cd /workspace/ros2_ws

apt-get update -qq
rosdep update
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
colcon test
colcon test-result --verbose

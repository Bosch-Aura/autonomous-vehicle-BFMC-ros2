from glob import glob
import os

from setuptools import find_packages, setup

package_name = 'bfmc_hardware_bridge'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'),
            glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='AURA Team',
    maintainer_email='martina.dejesus@ing.austral.edu.ar',
    description='Puente entre ROS2 y el ESP32: único dueño del puerto serial.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'hardware_bridge = bfmc_hardware_bridge.hardware_bridge_node:main',
        ],
    },
)

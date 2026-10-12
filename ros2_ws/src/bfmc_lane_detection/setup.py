import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'bfmc_lane_detection'

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
    description='Convierte la imagen de la cámara en el estado del carril.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'lane_detection = bfmc_lane_detection.lane_detection_node:main',
        ],
    },
)

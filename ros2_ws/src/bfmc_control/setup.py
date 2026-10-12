import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'bfmc_control'

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
    description='Convierte el estado del carril en un comando de dirección con un PID.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'control = bfmc_control.control_node:main',
        ],
    },
)

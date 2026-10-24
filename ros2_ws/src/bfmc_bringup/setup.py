from glob import glob
import os

from setuptools import find_packages, setup

package_name = 'bfmc_bringup'

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
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='AURA Team',
    maintainer_email='martina.dejesus@ing.austral.edu.ar',
    description='Arranca todo el sistema AURA con un solo comando.',
    license='MIT',
    tests_require=['pytest'],
)

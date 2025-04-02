import os
from glob import glob
from setuptools import setup
from setuptools import find_packages, setup

package_name = 'auv_drivers'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'),glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='matsya',
    maintainer_email='sparshbadjte27@gmail.com',
    description='TODO: Package description',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'can_main = auv_drivers.can.can_main:main',
            'dvl_publisher = auv_drivers.dvl.dvl_publisher:main',
            'ps_main = auv_drivers.pressure_sensor.ps_main:main',
            'node = auv_drivers.node:main'
        ],
    },
)

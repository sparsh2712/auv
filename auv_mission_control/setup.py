from setuptools import find_packages, setup

package_name = 'auv_mission_control'

setup(
    name=package_name,  
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ubuntu',
    maintainer_email='jashdalal2018@gmail.com',
    description='Mission Control Package',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'main = auv_mission_control.main:main',
            'test= auv_mission_control.test_nav_service:main'  
        ],
    },
)


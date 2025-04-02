from setuptools import find_packages, setup

package_name = 'auv_navigator'

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
    maintainer='akshaj',
    maintainer_email='akshaj0712@gmail.com',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'navigatorClient=auv_navigator.navigator_client:main',
            'navigatorServer=auv_navigator.navigator_server:main'
        ],
    },
)
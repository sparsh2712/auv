#!/usr/bin/python3
import threading
"""
Simple Utility to run auv drivers
---
Steps to install IMU Driver:
1. Fetch repo from: https://github.com/LORD-MicroStrain/microstrain_inertial <- keep in ws src folder
2. Change c++ version from 11 to 14 <- note do this for all cmake files
"""
import serial.tools.list_ports
import os

# Find available serial ports
available_ports = serial.tools.list_ports.comports()

# Look for the desired port
ps, imu, can = "", "", ""
os.system('sudo chmod a+rwx /dev/tty*')
for port in available_ports:
    if 'Arduino' in port.description:
        ps = port.device
    elif 'Lord' in port.description:
        imu = port.device
    elif 'CAN' in port.description:
        can = port.device
print("ps:", ps)
print("imu:", imu)
print("can:", can)
if imu != "":
    os.system('sed -i \'s/port        : "\/dev\/ttyACM."/port        : "\/dev\/ttyACM'+imu[-1]+'"/g\' ~/ros2_ws/src/Robosub_ROS2_2024/src/microstrain_inertial/microstrain_inertial_driver/config/empty.yml')
# os.system('ros2 launch auv_drivers drivers.launch.py can_port:='+can+' ps_port:='+ps+' &')  
# os.system('ros2 launch microstrain_inertial_driver microstrain_launch.py configure:=true activate:=true &')
def imu_func():
    os.system('ros2 launch microstrain_inertial_driver microstrain_launch.py params_file:=/home/matsya/ros2_ws/src/Robosub_ROS2_2024/src/microstrain_inertial/microstrain_inertial_driver/config/empty.yml configure:=true activate:=true ')
def ps_dvl_func():
    os.system('ros2 launch auv_drivers drivers.launch.py can_port:='+can+' ps_port:='+ps)
imu_thread = threading.Thread(target=imu_func)
ps_dvl_thread = threading.Thread(target=ps_dvl_func)

imu_thread.start()
ps_dvl_thread.start()

# try:
#     while True:
#         pass
# except KeyboardInterrupt:
#     os.system('ros2 lifecycle.set /microstrain')

import os 
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()
    



    node1=Node(
        package = "auv_controller",
        executable = "controller",
        name = "controller",
        parameters = []
    )

    ld.add_action(node1)

    return ld

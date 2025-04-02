import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()
    config1 = os.path.join(
        get_package_share_directory('auv_localization'),
        'config',
        'localization_constants.yaml'
        )


    node1=Node(
        package = "auv_localization",
        executable = "localization",
        name = "localization",
        parameters = [config1]
    )

    ld.add_action(node1)

    return ld


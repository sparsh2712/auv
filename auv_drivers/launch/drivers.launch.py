import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
import launch

def generate_launch_description():
    ld = LaunchDescription()
    config1 = os.path.join(
        get_package_share_directory('auv_drivers'),
        'config',
        'dvl_params.yaml'
        )

    ps_port = launch.actions.DeclareLaunchArgument(
        'ps_port',
        default_value='/dev/ttyACM0',
        description='Port for PS (Pressure Sensor)'
    )

    can_port = launch.actions.DeclareLaunchArgument(
        'can_port',
        default_value='/dev/ttyUSB0',
        description='Port for CAN'
    )

    ld.add_action(ps_port)
    ld.add_action(can_port)

    node1=Node(
        package = "auv_drivers",
        executable = "dvl_publisher",
        name = "dvl_publisher",
        parameters = [config1]
    )
    ld.add_action(node1)

    node2=Node(
        package = "auv_drivers",
        executable = "node",
        name = "node",
        arguments = [launch.substitutions.LaunchConfiguration('can_port')]
    )
    ld.add_action(node2)
    
    return ld
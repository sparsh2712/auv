from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()

    node1 = Node(
        package="auv_vision",
        executable="scam_vision",
        name="auv_vision",
        output="screen",
        emulate_tty=True
    )
    
    ld.add_action(node1)

    return ld


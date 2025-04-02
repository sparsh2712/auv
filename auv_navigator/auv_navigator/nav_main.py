import rclpy
from navigator import Navigator

def main():
    rclpy.init()  # Initialize the ROS 2 communication
    navigator = Navigator()  # Create the Navigator instance

    try:
        while rclpy.ok():  # Check if ROS 2 is running
            navigator.run()  # Run the navigator logic
            rclpy.spin_once()  # Allow ROS 2 to handle callbacks
    except KeyboardInterrupt:
        pass
    finally:
        navigator.destroy_node()  # Clean up the node
        rclpy.shutdown()  # Shut down the ROS 2 system

if __name__ == "__main__":
    main()


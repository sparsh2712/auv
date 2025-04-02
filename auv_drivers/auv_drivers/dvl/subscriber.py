#!/usr/bin/env python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from auv_msgs.msg import DVL
from auv_msgs.msg import DVLBeam

class DVLSubscriber(Node):
    def __init__(self):
        super().__init__('dvl_sub')
        self.dvl_sub_raw = self.create_subscription(String, "dvl/json_data", self.callbackRAW)
        self.dvl_sub = self.create_subscription(DVL, "dvl/data", self.callback)

    def callbackRAW(self, data):
        self.get_logger().info("Data received: %s", data.data)
    
    def callback(self, data):
        self.get_logger().info( "Time received: %s",data.time)

def main(args=None):
    rclpy.init(args=args)
    dvl_subscriber = DVLSubscriber()
    rclpy.spin(dvl_subscriber)
    dvl_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

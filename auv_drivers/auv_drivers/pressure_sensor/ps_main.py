#!/usr/bin/env python3
import sys
import rclpy
from rclpy.node import Node
from auv_msgs.msg import PsData
from .ps_driver import PsDriver

class PsMain(Node):
    def __init__(self,port_name):
        super().__init__("ps_main")
        self.ps_pub = self.create_publisher(PsData, "/ps/data", 1)
        self.ps_driver=PsDriver(port=port_name)
        self.timer=self.create_timer(0.1, self.run)
    
    def read(self):
        self.ps_driver.read_data()

    def publish(self):
        ps_msg=PsData()
        ps_msg.depth=self.ps_driver.get_depth()
        self.ps_pub.publish(ps_msg)
    
    def run(self):
        self.read()
        self.publish()

def main(args=None):
    port=sys.argv[1]
    rclpy.init(args=args)
    ps_main=PsMain(port)
    rclpy.spin(ps_main)
    ps_main.destroy_node()
    rclpy.shutdown()

if __name__=="__main__":
    main()
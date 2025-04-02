import copy
import numpy as np
import rclpy
from rclpy.node import Node
from scipy.spatial.transform import Rotation
import asyncio
import time
from threading import Thread

from std_srvs.srv import Empty
from auv_msgs.msg import AuvState, Pose, VisionSetpoints, NavStatus
from auv_msgs.srv import SetpointService
from auv_msgs.srv import AlignService
from auv_msgs.srv import ScanService
from auv_msgs.srv import TorpedoState
from auv_msgs.srv import GripperState
from auv_msgs.srv import MarkerDropperState
from auv_msgs.srv import PingerAcq
from auv_msgs.srv import SetDetector
import os

class NavSerTest(Node):#(Node):
    def __init__(self):
        super().__init__("mission_control_test_with_nav")
        # self.node = node
        # self.logger = node.get_logger()

        # self.logger.info("Initializing Matsya...")

        # Initialize services
        self.timer_period = 0.5
        self.timer_looper = self.create_timer(self.timer_period, self.print)
        self.ctr = 1
        
        self.nav_setpoint_proxy = self.create_client(SetpointService, "/navigator/setpoint_service")

    def print(self):
        print("BLAHHHH", self.ctr)
        self.ctr += 1
    
    def run(self):
        if self.ctr%10 != 0:
            return
        print("RUN called")
        # rclpy.spin_once(self, timeout_sec=2)
        nav_req = SetpointService.Request()
        task_location = [100.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        nav_req.pose.position.x = float(task_location[0])
        nav_req.pose.position.y = float(task_location[1])
        nav_req.pose.position.z = float(task_location[2])
        nav_req.pose.orientation.roll = float(task_location[3])
        nav_req.pose.orientation.pitch = float(task_location[4])
        nav_req.pose.orientation.yaw = float(task_location[5])
        nav_req.pos_tolerance = float(10)
        nav_req.angle_tolerance = float(5)
        self.in_transit = True
        print("sleep start")
        time.sleep(3)
        print("sleep end")
        while not self.nav_setpoint_proxy.wait_for_service(timeout_sec=1.0):
            print("Service unavailable")
        nav_fut = self.nav_setpoint_proxy.call_async(nav_req)
        print("waiting")
        # return nav_fut
        rclpy.spin_until_future_complete(self, nav_fut)
        # while rclpy.ok():
        #     print("####")
        #     rclpy.spin_once(self)
        #     print("$$$$$$$$")
        #     if nav_fut.done():
        #         break
        #     else:
        #         print("Future not complete")

        nav_res = nav_fut.result()
        print("Response recieved", nav_res)
        self.current_nav_service_id = nav_res.service_id
        print("service id", self.current_nav_service_id)

def main(args=None):
    folder_path = '~/Desktop/ros2_ws/src/Robosub_ROS2_2024/auv_mission_control/config'

    rclpy.init(args=args)
    #params = load_yaml_files(folder_path)
    mission_control=NavSerTest()
    print("Calling RUN in main")

    while True:
        mission_control.run()
        rclpy.spin_once(mission_control, timeout_sec=1)
        # time.sleep(0.1)
    # print("waiting..")
    # rclpy.spin_until_future_complete(mission_control, fut)
    # print("Response: ", fut.result())

    # rclpy.spin(mission_control)

    mission_control.destroy_node()
    rclpy.shutdown()
    
    # time.sleep(2)
    # mission_control.run()

if __name__ == "__main__":
    main()
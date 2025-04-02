#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from .perform_task import PerformTask
from .constants import ConfigLoader
from .map import Map
from .scam_matsya import Matsya
from std_msgs.msg import Bool
import numpy as np
import time
from auv_msgs.msg import AuvState

class MissionControl(Node):
    def __init__(self):
        super().__init__("mission_control")
        self.config = ConfigLoader()
        self.map = self.config.get_map_constants()
        self.mission_plan= self.config.get_mission_constants()
        self.task_list = self.config.get_task_list()
        self.pose_is_valid = False
        self.matsya = Matsya(self)
        self.map = None #change to map object after modifying code
        print("hello world")
        self.timer_period = 1
        self.task_index = 0
        self.using_backup = False
        self.task_details = None

        #check for data validity ideally using a timer in matsya and destructing it if loaclisation data valid

        #objective: to iterate through mission plan and call perform task while taking inputs from matsya 
        
    def run(self):
        print("welvome mission control")
        #print(f"mission plan is {self.task_list}")
        #print(self.mission_plan[self.task_index]["main"])
        self.task_name = (self.mission_plan[self.task_index]["main"])
        if self.task_index < len(self.task_list):
            if not self.using_backup:
                self.get_logger().info("Not using backup main")
                print(self.task_name)
                self.task_details = self.task_list[self.task_name]
                #print(task_details)

            actions = self.task_details["actions"]
            timeout = self.task_details["task_timeout"]

            print("performing the task main")
            task_performer = PerformTask(
                self.matsya,
                self.map,
                self.task_name,
                actions,
                timeout
            )
            task_achieved = task_performer.run()
            print(f"task performer output is = {task_achieved}")

            if task_achieved is True:
                print("Completed task main", self.task_index)
                self.using_backup=False
                self.task_index+=1
            # if not task_achieved and not self.using_backup and self.backup_list[self.task_index]!="":
            #     print("Failed task main", self.task_index)
            #     self.using_backup=True
            #     self.task_name=self.backup_list[self.task_index]
            # else:

                
        

        
    # def run(self):
    #     task_index = 0
    #     print("1")
    #     using_backup = False
    #     self.get_logger().info(f"Number of tasks main: {len(self.task_list)}")
    #     print("2")
    #     while task_index < len(self.task_list):
    #         print("3")
            
            
    #         print("5")
            
    #         self.get_logger().info("Getting the task params main")
    #         #(actions, task_timeout) = get_task_params(task_name)
    #         #actions = self.get_parameter("/mission_control/tasks/{task}/actions".format(task=task_name)).get_parameter_value().string_array_value
    #         #task_timeout = self.get_parameter("/mission_control/tasks/{task}/task_timeout".format(task=task_name)).value
    #         tasks = self.params["rs24_mission.yaml"]["mission_control"]["tasks"]
    #         actions = []
    #         task_timeout = []
    #         print("6")
            
    #         for task_name in tasks:
    #             #print(f"Task Name: {task_name}")
    #             actions.append(tasks[task_name]["actions"])
    #             #print(f"Task Details: {task_details}")
    #             task_timeout.append(tasks[task_name]["task_timeout"])
    #             print("7")
            
    #         print("8")

    #         self.get_logger().info("Performing the task main")
    #         task_afg = Bool()
    #         task_afg.data = False
    #         task_performer = PerformTask(
    #             self.matsya,
    #             self.map,
    #             task_name,
    #             actions,
    #             task_timeout
    #         )
    #         print("9")


    #         #await task_performer.run()
    #         task_achieved = task_performer.run()
    #         print(f"taskachieved = {task_achieved}")
    #         print("10")

    #         if not task_achieved and not using_backup:# and self.backup_tasks[task_index] != "":
    #             self.get_logger().info(f"Failed task main {task_index}")
    #             #using_backup = True
    #             task_name = self.backup_tasks[task_index]
    #             #time.sleep(1)
    #         else:
    #             print(f"backup_list has {self.backup_tasks[task_index]}")
    #             self.get_logger().info(f"Completed task main {task_index}")
    #             using_backup = False
    #             task_index += 1



def main(args=None):
    folder_path = '~/Desktop/ros2_ws/src/Robosub_ROS2_2024/auv_mission_control/config'

    rclpy.init(args=args)
    #params = load_yaml_files(folder_path)
    mission_control=MissionControl()
    print("started")
    while True:
        for i in range(9):
        # if mission_control.matsya.pose_is_valid:
            rclpy.spin_once(mission_control)
            time.sleep(0.05)
            print("running node")
        print("mission control run")
        if mission_control.matsya.pose_is_valid:
            rclpy.spin_once(mission_control)
            mission_control.run()
        print("mission control run")

        # else:
        #     rclpy.spin_once(mission_control)
        #     print("localisation is invalid")
        #     time.sleep(0.1)
            
    
    mission_control.destroy_node()
    
    # time.sleep(2)
    # mission_control.run()

if __name__ == "__main__":
    main()

# class MissionControl(Node):
#     def __init__(self):
#         super().__init__('mission_control')  # Initializing the ROS 2 node
#         self.params = load_yaml_files('~/Desktop/ros2_ws/src/Robosub_ROS2_2024/auv_mission_control/config')
#         self.get_logger().info("1")

#         self.pose_is_valid = False

#         num_tasks = 5  # self.params["rs24_mission.yaml"]["mission_control"]["plan"]["num_tasks"]
#         self.task_list = self.params["rs24_mission.yaml"]["mission_control"]["plan"]["mission"]
#         self.main_tasks = []
#         self.backup_tasks = []

#         for task in self.task_list:
#             self.main_tasks.append(task["main"])
#             self.backup_tasks.append(task["backup"])

#         self.get_logger().info("2")

#         tasks = self.params["map.yaml"]["mission_control"]["map"]
#         self.map_task_list = []
#         self.map_pose_list = []
#         #self.localization_sub = self.create_subscription(AuvState, "/localization/pose", self.cb_pose, 10)

#         for i in range(len(tasks)):
#             self.map_task_list.append(tasks[i]["task"])
#             pose = tasks[i]["pose"]
#             self.map_pose_list.append([float(pose["x"]), float(pose["y"]), float(pose["z"]), float(pose["roll"]), float(pose["pitch"]), float(pose["yaw"])])
#         #node = Node
#         self.get_logger().info("3")
#         self.matsya = Matsya(self)
#         self.get_logger().info("4")
#         self.map = Map(self.map_task_list, self.map_pose_list)

#         # Wait for pose_is_valid to be True using ROS spin_once
#         while not self.matsya.pose_is_valid:
#             self.get_logger().info("Waiting for localization to be valid.")
#             rclpy.spin_once(self)  # process one callback in the ROS event loop

#         self.get_logger().info("Localization is valid.")

#         timer_period = 4
#         self.timer = self.create_timer(timer_period, self.run)
#         self.using_backup = False
#         self.task_index = 0

#     # def cb_pose(self, msg: AuvState):
#     #     self.pose = msg
#     #     self.pose_is_valid = True
#     #     self.get_logger().info(f"Updated Matsya pose: {msg}")
    
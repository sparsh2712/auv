import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from auv_msgs.msg import AuvState
import numpy as np
import asyncio
import time
from .constants import ConfigLoader
#why perform x is a node?
class PerformTask:
    def __init__(self, matsya, map, task_name, task_actions, task_timeout):
        #super().__init__('perform_task_node')
        self.matsya = matsya
        self.map = map
        self.task_name = task_name
        self.task_actions = task_actions
        self.task_timeout = task_timeout
        self.pose=np.zeros(6, dtype=float)
        self.tolerance_x = 10
        self.tolerance_y = 10
        self.tolerance_z = 10
        self.tolerance_roll = 5
        self.tolerance_pitch = 5
        self.tolerance_yaw = 5
        self.config = ConfigLoader()
        self.map = self.config.get_map_constants()
        self.map = {entry["task"]: entry["pose"] for entry in self.map}

    def pose_list_from_dict(self, dict_obj):
        list_obj = []
        list_obj.append(dict_obj["x"])
        list_obj.append(dict_obj["y"])
        list_obj.append(dict_obj["z"])
        list_obj.append(dict_obj["roll"])
        list_obj.append(dict_obj["pitch"])
        list_obj.append(dict_obj["yaw"])
        return list_obj
    
    def cbPose(self, msg):
        self.pose[0]=msg.position.x
        self.pose[1]=msg.position.y
        self.pose[2]=msg.position.z
        self.pose[3]=msg.orientation.roll
        self.pose[4]=msg.orientation.pitch
        self.pose[5]=msg.orientation.yaw
        self.pose_is_valid = True

    def run(self):
        action_count = 0
        task_location = None
        # print(f"started action : {action_count}")
        # print(self.task_actions)
        for action in self.task_actions:
            action_count += 1
            #print(self.map)
            task_location = self.map.get(self.task_name)
            # print(task_location)
            # print(f"action is {action}")

            # if task_location is not None:
            #     print(task_location, " = task_location")
            #     print(f"the poser ofndskjnvsdkndsdsv {self.matsya.pose}")
            #     #time.sleep(1)
            #     if np.abs(self.matsya.pose.position.x-task_location['x']) < self.tolerance_x and \
            #        np.abs(self.matsya.pose.position.y-task_location['y']) < self.tolerance_y and \
            #        np.abs(self.matsya.pose.position.z-task_location['z']) < self.tolerance_z and \
            #        np.abs(self.matsya.pose.orientation.roll-task_location['roll']) < self.tolerance_roll and \
            #        np.abs(self.matsya.pose.orientation.pitch-task_location['pitch']) < self.tolerance_pitch and \
            #        np.abs(self.matsya.pose.orientation.yaw-task_location['yaw']) < self.tolerance_yaw:
            #        print("something is done")
            #        return True
                    # return True
                # else:
                #     return False
            
            if action["name"] == "reach":
                print("Reaching started")

                reach_pos_tolerance = action["reach_pos_tolerance"]
                reach_angle_tolerance = action["reach_angle_tolerance"]
                reach_timeout = action["reach_timeout"]
                #task_location = self.map.get_task_pose(self.task_name)
                if task_location is None:
                    # print(f"Task location for {self.task_name} is None.")
                    return False


                task_reached = self.matsya.reach_task(task_location,
                                                      reach_pos_tolerance, 
                                                      reach_angle_tolerance, 
                                                      reach_timeout)
                # task_reached =True
                if task_reached:
                    print("heyyyyyyyyyyyyy task achieved")
                if not task_reached:
                    return False

            elif action["name"] == "navigate":
                rel_position = self.pose_list_from_dict(action["rel_position"])
                navigate_pos_tolerance = action["navigate_pos_tolerance"]
                navigate_angle_tolerance = action["navigate_angle_tolerance"]

                #self.get_logger().info("Navigating")

                setpoint_navigated = self.matsya.navigate(rel_position,
                                                          navigate_pos_tolerance,
                                                          navigate_angle_tolerance)

                if not setpoint_navigated:
                    return False

            elif action["name"] == "raw_navigate":
                # print("$$$$$$$$ raw navigate $$$$$$$")
                rel_position = self.pose_list_from_dict(action["rel_position"])
                navigate_pos_tolerance = action["navigate_pos_tolerance"]
                navigate_angle_tolerance = action["navigate_angle_tolerance"]
                print(rel_position, "= relative position")
                # self.get_logger().info("Raw Navigating")

                setpoint_navigated = self.matsya.raw_navigate(rel_position,
                                                             navigate_pos_tolerance,
                                                             navigate_angle_tolerance)

                if setpoint_navigated:
                    print("heyyyyyyyyyyyyy setpoint navigated")
                if not setpoint_navigated:
                   print("Not navigated!")
                   return False

            elif action["name"] == "align_continuous":
                align_xy_tolerance = action["align_xy_tolerance"]
                align_z_tolerance = action["align_z_tolerance"]
                align_angle_tolerance = action["align_angle_tolerance"]
                align_hard_z_value = action["hard_z_value"]
                align_displacement = self.pose_list_from_dict(action["align_displacement"])
                scan_type = action["scan_type"]
                timeout = action["timeout"]

                #self.get_logger().info("Aligning continuously")
                self.map = [200,300,40,0,0,20]
                alignment_achieved = self.matsya.scan_align_continuous(self.map, self.task_name,
                                                                  align_xy_tolerance, align_z_tolerance, 
                                                                  align_angle_tolerance,  
                                                                  align_displacement, align_hard_z_value,
                                                                  scan_type, timeout)

                if not alignment_achieved:
                    #self.get_logger().info("Did not align!")
                    return False
                    continue
                    ###   def scan_align_continuous(self, map, task_name, xy_tolerance, z_tolerance, angle_tolerance,
                        #  displacement, hard_z_value, scan_type="vertical_spiral",
                        #  timeout=12, align_wait_time=2):
            elif action["name"] == "shoot_torpedo":
                torpedo_position = action[0]["torpedo_position"]
                self.matsya.shoot_torpedo(torpedo_position)

            elif action["name"] == "drop_marker":
                self.matsya.drop_marker(0)

            elif action["name"] == "go_to_pinger":
                pinger_frequency = action[0]["pinger_frequency"]
                timeout = action[0]["pinger_timeout"]
                pos_tolerance = action[0]["pos_tolerance"]
                angle_tolerance = action[0]["angle_tolerance"]

                data_valid = False
                while not data_valid:
                    data_valid, pinger_pose = self.matsya.get_pinger_pose(pinger_frequency)
                    if not data_valid:
                        continue
                    self.matsya.navigate(pinger_pose)
            
            # elif task_location is not None:
            #     print(task_location, " = task_location")
            #     #time.sleep(1)
            #     if np.abs(self.pose[0]-task_location['x']) < self.tolerance_x and \
            #        np.abs(self.pose[1]-task_location['y']) < self.tolerance_y and \
            #        np.abs(self.pose[2]-task_location['z']) < self.tolerance_z and \
            #        np.abs(self.pose[3]-task_location['roll']) < self.tolerance_roll and \
            #        np.abs(self.pose[4]-task_location['pitch']) < self.tolerance_pitch and \
            #        np.abs(self.pose[5]-task_location['yaw']) < self.tolerance_yaw:
            #         return True
            #     else:
            #         print(f'Achieved action: {action_count}')
            #         return False
            
            else:
                print("Task location is not available, cannot compare.")
                return False


        return True


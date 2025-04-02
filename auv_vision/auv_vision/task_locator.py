#!/usr/bin/python3

import rclpy
import numpy as np
from scipy.spatial.transform import Rotation
from .map_load import load_map
from .map import Map
from auv_msgs.msg import AuvState, VisionSetpoints, ObjectDetected, TaskDetected

class TaskLocator:
    def __init__(self, task_details, objects, dvl_to_camera_vector, camera):
        self.objects = objects
        self.camera = camera
        self.task_details = task_details
        self.dvl_to_camera_vector = dvl_to_camera_vector
        (self.map_task_list, self.map_pose_list) = load_map()
        self.map = Map(self.map_task_list, self.map_pose_list)
        self.cur_pose = AuvState()
        self.fr_frame = 0
        self.bt_frame = 0
   
    def get_object_midpoints(self, bboxes, objects, rel_pose_list):
        test_height = 0
        test_width = 0
        pose_list_counter = 0
        print(bboxes, rel_pose_list)
        for bbox in bboxes:
            for object in objects:
                if bbox.class_id == object.object_id:
                    rel_midpoint, test_height, test_width = rel_pose_list[pose_list_counter]
                    pose_list_counter += 1
                    object.rel_midpoint = np.array(rel_midpoint) + np.array(self.dvl_to_camera_vector)
                    if self.camera == "bottom":
                        pitch_90_rot = Rotation.from_euler("ZYX", (0.0, -90.0, 0.0), degrees=True)
                        object.rel_midpoint = np.matmul(pitch_90_rot.as_matrix(), object.rel_midpoint)

                    object.object_detected = True

                    rot = Rotation.from_euler("ZYX", (self.cur_pose.orientation.yaw,
                        self.cur_pose.orientation.pitch, self.cur_pose.orientation.roll),
                        degrees=True)
                    print('object', object)
                    object.glob_midpoint = np.matmul(rot.as_matrix(),
                        np.array(object.rel_midpoint)) + np.array([self.cur_pose.position.x,
                        self.cur_pose.position.y, self.cur_pose.position.z])

        return objects

    def get_task_location(self, task_name, cur_pose, objects):
        task_location = None
        for task in self.task_details:
            if task['task_name'] == task_name:
                task_objects_list = task['objects']

        # task_objects_list should be in a priority order with the first object trained the best
        ######### Reordering to account for priority in task detection ##########
        #objects = self.prioritize(task_objects_list, objects)
        #######################
        
        mean_location = np.zeros(3)
        counter = 0
        for vision_object in objects:
            for task_object in task_objects_list:
                if vision_object.object_name == task_object['name'] and vision_object.object_detected:
                    rel_vector_from_task = task_object['rel_vector_from_task']
                    rel_vector_to_task = [-rel_vector_from_task["x"],
                                          -rel_vector_from_task["y"],
                                          -rel_vector_from_task["z"]]
                    # Using yaw from map.yaml
                    task_map_location=self.map.get_task_pose(task_name)
                    print(task_name)
                    #####
                    rot = Rotation.from_euler("ZYX", (float(task_map_location[0]),
                                                      0,
                                                      0), degrees=True)
                    global_vector_to_task = np.matmul(rot.as_matrix(), rel_vector_to_task)
                    task_location = vision_object.glob_midpoint + global_vector_to_task

                    if task_name != "Octagon":
                        print("Taking ", vision_object.object_name)
                        print("Seedpoint: ",task_location)
                        return task_location
                    else:
                        mean_location +=task_location
                        counter+=1
        return task_location
   
    def detect_tasks(self, objects):
        task_locations=[]
        for task in self.task_details:
            task_name = task["task_name"]
            task_objects = task["objects"]
            for object in objects:
                if object.object_name in [item['name'] for item in task_objects] and object.object_detected:
                    task_location = self.get_task_location(task_name, self.cur_pose, objects)
                    task_locations.append(
                    {"task_name": task_name, "task_location": task_location}
                )
        return task_locations
    
    def locate_tasks(self, bboxes, rel_pose_list, cur_pose):
        self.cur_pose = cur_pose
        self.objects = self.get_object_midpoints(bboxes, self.objects, rel_pose_list)

        vision_setpoints = VisionSetpoints()

        for object in self.objects:
            if object.object_detected:
                cur_obj = ObjectDetected()
                cur_obj.name = object.object_name
                cur_obj.position.x = object.glob_midpoint[0]
                cur_obj.position.y = object.glob_midpoint[1]
                cur_obj.position.z = object.glob_midpoint[2]

                obj_plot = {}
                obj_plot['x'] = cur_obj.position.x
                obj_plot['y'] = cur_obj.position.y
                obj_plot['z'] = cur_obj.position.z
                obj_plot['frame_num'] = self.fr_frame

                vision_setpoints.objects.append(cur_obj)

        task_locations = self.detect_tasks(self.objects)

        for task in task_locations:
            if task["task_location"] is None:
                continue

            cur_task = TaskDetected()
            cur_task.name = task["task_name"]
            cur_task.position.x = task["task_location"][0]
            cur_task.position.y = task["task_location"][1]
            cur_task.position.z = task["task_location"][2]
            vision_setpoints.tasks.append(cur_task)

        return vision_setpoints


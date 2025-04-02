import numpy as np
import copy
import rclpy
from rclpy.node import Node

class Map:
    def __init__(self, task_list, pose_list):
        self.task_list = task_list
        self.pose_list = pose_list
        self.global_pose_dict = {}
        list_length_error = "Number of tasks doesn't match number of poses provided to map"
        assert len(task_list) == len(pose_list), list_length_error

        # Store tasks and poses in dictionary
        for (i, task) in enumerate(self.task_list):
            self.global_pose_dict[task] = np.array(pose_list[i])

    def add_task(self, new_task, new_pose):
        if new_task not in self.task_list:
            self.task_list.append(new_task)
            self.global_pose_dict[new_task] = np.array(new_pose)

    def update_task(self, updated_task, updated_task_position):
        old_global_pose_dict = copy.deepcopy(self.global_pose_dict)
        updated_task_position = np.array(updated_task_position)

        # Update global poses for all tasks based on the updated task
        for task in self.task_list:
            self.global_pose_dict[task][:3] = updated_task_position + \
                                              old_global_pose_dict[task][:3] - \
                                              old_global_pose_dict[updated_task][:3]

    def get_task_pose(self, requested_task):
        task_pose = self.global_pose_dict.get(requested_task, None)
        return task_pose

if __name__=="__main__":
    tasks = ["gate", "buoy", "torpedo", "octagon"]
    poses = [
        [1.0, 1.0, 0.0, 0.0, 0.0, 45.0],
        [2.0, 2.0, 0.0, 0.0, 0.0, 135.0],
        [3.0, 3.0, 0.0, 0.0, 0.0, -90.0],
        [4.0, 4.0, 0.0, 0.0, 0.0, 180.0]
    ]

    tasks, poses = load_map()

    map = Map(tasks, poses)

    for task in tasks:
        print(task, "is at :", map.get_task_pose(task))

    map.update_task("buoy", [700.0, 800.0, 300.0])

    for task in tasks:
        print(task, "is at :", map.get_task_pose(task))


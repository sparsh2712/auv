import rclpy
import numpy as np

def load_map():
    tasks = [
        {"task": "Gate", "pose": {"x": 250, "y": -40, "z": 100, "roll": 0, "pitch": 0, "yaw": 0}},
        {"task": "Bin", "pose": {"x": 1420, "y": 370, "z": 100, "roll": 0, "pitch": 0, "yaw": 180}},
        {"task": "Buoy", "pose": {"x": 660, "y": -300, "z": 100, "roll": 0, "pitch": 0, "yaw": 0}},
        {"task": "TempBuoy", "pose": {"x": 0, "y": 0, "z": 70, "roll": 0, "pitch": 0, "yaw": 0}},
        {"task": "TorpedoOld", "pose": {"x": 0, "y": 0, "z": 0, "roll": 0, "pitch": 0, "yaw": 0}},
        {"task": "Torpedo", "pose": {"x": 950, "y": 400, "z": 100, "roll": 0, "pitch": 0, "yaw": -7}},
        {"task": "TorpedoSafe", "pose": {"x": 950, "y": 400, "z": 100, "roll": 0, "pitch": 0, "yaw": -7}},
        {"task": "TorpedoFlex", "pose": {"x": 1180, "y": -280, "z": 100, "roll": 0, "pitch": 0, "yaw": -7}},
        {"task": "Octagon", "pose": {"x": 1720, "y": -200, "z": 100, "roll": 0, "pitch": 0, "yaw": 0}}
    ]
    #tasks = rospy.get_param("/mission_control/map")
    task_list = []
    pose_list = []
    if tasks == None:
        return (task_list, pose_list)
    for i in range(len(tasks)):
        task_list.append(tasks[i]["task"])
        pose = tasks[i]["pose"]
        pose_list.append([float(pose["x"]), float(pose["y"]), float(pose["z"]),
                          float(pose["roll"]), float(pose["pitch"]), float(pose["yaw"])])
    return (task_list, pose_list)

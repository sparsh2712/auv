import json 
import os

class ConfigLoader():
    def __init__(self):
        dir_path = os.path.dirname(os.path.abspath(__file__))
        os.chdir("/home/xyz/Desktop/ros2_ws/src/Robosub_ROS2_2024/auv_mission_control/config")
        self.config_dir = os.getcwd() #os.path.join(dir_path, "../../../../../src/Robosub_ROS2_2024/auv_mission_control/", "config")
        self.setup_details = {}
        self.load_constants()
    
    def load_constants(self):
        print(self.config_dir)
        for file in os.listdir(self.config_dir):
            if file.endswith(".json"):
                file_path = os.path.join(self.config_dir, file)
                print(file)
                with open(file_path, 'r') as f:
                    key = file.split(".")[0]
                    self.setup_details[key] = json.load(f)
    
    def get_map_constants(self):
        #print(self.setup_details["map"]["map"])
        return self.setup_details["map"]["map"]

    def get_mission_constants(self):
        #print(self.setup_details["mission"]["plan"])
        return self.setup_details["mission"]["plan"]
    
    def get_task_list(self):
        #print(self.setup_details["tasks"]["tasks"])
        return self.setup_details["tasks"]["tasks"]


if __name__ == "__main__":
    obj = ConfigLoader()
    print(obj.get_map_constants())

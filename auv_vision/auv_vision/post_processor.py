import numpy as np
import matplotlib.pyplot as plt
import os
import statistics
from scipy.optimize import curve_fit
from pathlib import Path
from ament_index_python.packages import get_package_share_directory

workspace_path = '~/ros2_ws'
workspace_path = os.path.expanduser(workspace_path)
package_name = 'auv_vision'
src_path = os.path.join(workspace_path, 'src', 'Robosub_ROS2_2024', package_name, package_name)

PATH = get_package_share_directory('auv_vision') + '/src'
#PATH = os.getcwd()

window = 5
majority_required = 3
frame_gap = 15

def linear_func(x, m, c):
    return m*x + c

class PostProcessor():
    def __init__(self, obj_name, obj_agreement_threshold):
        self.detections = []
        self.obj_name = str(obj_name)
        self.obj_agreement_threshold = obj_agreement_threshold
        self.window_detections = []
        os.listdir(src_path)
        os.chdir(src_path + '/detections')

    def add_detection(self, detection):
        # detection = {}
        data = {}
        data['x'] = detection['x']
        data['y'] = detection['y']
        data['z'] = detection['z']
        data['distance'] = np.sqrt(pow(detection['x'], 2) + pow(detection['y'], 2) + pow(detection['z'], 2))
        data['frame_num'] = detection['frame_num']
        self.detections.append(data)

    def sliding_window_detect(self, detection):
        self.add_detection(detection)
        print("\n\n")
        print("Current detection:", detection['x'], detection['y'], detection['z'])
        
        total_num_agreed = 0
        for i in range(len(self.detections) - window - 1, len(self.detections) - 1):
            if i < 0:
                continue
            
            dist = np.sqrt((self.detections[i]['x'] - detection['x'])**2 +
                           (self.detections[i]['y'] - detection['y'])**2 +
                           (self.detections[i]['z'] - detection['z'])**2)
            
            if dist < self.obj_agreement_threshold:
                print("Agreement threshold reached!")
                total_num_agreed += 1

            print(dist)

        print("Total num agreed:", total_num_agreed)
        if total_num_agreed < majority_required:
            return []
        
        return [detection["x"], detection["y"], detection["z"]]


def main():
    test = PostProcessor("hello")
    for i in range(10):
        detec = {}
        detec['x'] = np.random.randint(low=0, high=100)
        detec['y'] = np.random.randint(low=0, high=100)
        detec['z'] = np.random.randint(low=0, high=100)
        detec['frame_num'] = i+1
        pred = test.sliding_window_detect()
        print(pred)
    

if __name__=="__main__":
    main()

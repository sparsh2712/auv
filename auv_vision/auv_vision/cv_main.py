import cv2 
import os
from .bin_detection import Bin
from .face_detection import Face

workspace_path = '~/ros2_ws'
workspace_path = os.path.expanduser(workspace_path)
package_name = 'auv_vision'
PATH = os.path.join(workspace_path, 'src', 'Robosub_ROS2_2024', package_name, package_name)

class CV:
    def __init__(self):
        self.bin = Face()
        self.buoy = Bin() #same CV detector for both Bin and Buoy

    def predict(self,image,model_name="bin", frame_num=0):
        """
        image: opencv mat
        boxes has format
        x1 (pixels)  y1 (pixels)  x2 (pixels)  y2 (pixels)   confidence  class
        Ex: [0, 0, 10, 10, 1, 1]
        """
        if model_name=="bin":
            results = self.bin.detect(image, frame_num)
        elif model_name=="buoy":
            results = self.bin.detect(image, frame_num)
            
            #if len(results) > 0:
            #    results[0, 5] = 2 #assigning id as object id of buoy
        # print("cv_main raw: ", results)
        results = self.bin.detect(image, frame_num)
        return results

    def debug_draw(self, image):
        """
        @Todo: Implement Method To Return Rendered Image With BBox
        """
        self.cv_gate.debug_draw(image, save=True, image_name=PATH+"/test_output/image_"+str(int(round(datetime.now().timestamp())))+".jpg")
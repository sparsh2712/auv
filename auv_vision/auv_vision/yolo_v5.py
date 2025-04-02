"""
Wrapper for handling YoloV5
"""
from ament_index_python.packages import get_package_share_directory
import torch
from pathlib import Path
from datetime import datetime
from ultralytics import YOLO

package_path = get_package_share_directory('auv_vision')
PATH = package_path + '/src'

import os
workspace_path = '~/ros2_ws'
workspace_path = os.path.expanduser(workspace_path)
package_name = 'auv_vision'
src_path = os.path.join(workspace_path, 'src', 'Robosub_ROS2_2024', package_name, package_name)

class YoloV5:
    """
    Wrapper for calling YoloV5 inference on opencv image
    """
    def __init__(self):
        """
        Add a path to best pt(path='/best/pt'), and change yolov5s
        to custom to load a custom model
        @Todo: Replace with custom weights
        """
        '''
        self.model_octagon = torch.hub.load(PATH + '/yolov5','custom', path = PATH + '/yolov5_models/octagon.pt', source = 'local', device="cpu")
        self.model_buoy = torch.hub.load(PATH + '/yolov5','custom', path = PATH + '/yolov5_models/buoy.pt', source = 'local', device="cpu")
        self.model_torpedo = torch.hub.load(PATH + '/yolov5','custom', path = PATH + '/yolov5_models/torpedo.pt', source = 'local', device="cpu")
        self.model_bin = torch.hub.load(PATH + '/yolov5','custom',path = PATH + '/yolov5_models/bin.pt', source = 'local', device='cpu')
        self.model_octagon.eval()
        self.model_buoy.eval()
        self.model_torpedo.eval()
        self.model_bin.eval()
        '''
        # self.model_gate = torch.hub.load(PATH + '/yolov5','custom', path = PATH + '/yolov8_models/gate_sauvc.pt', source = 'local', device="cpu")
        #self.model_bin = torch.hub.load(PATH + '/yolov5','custom', path = PATH + '/yolov8_models/bins_sauvc.pt', source = 'local', device="cpu")
        #self.model_flare = torch.hub.load(PATH + '/yolov5','custom', path = PATH + '/yolov5_models/flare_sauvc.pt', source = 'local', device="cpu")
        # self.model_flare = YOLO(PATH + '/yolov8_models/flare_yolov8.pt')
        self.model_torpedo = YOLO(PATH + '/yolov8_models/yolov8_torpedo_single_new.pt')
        self.model_torpedo_flex = YOLO(PATH + '/yolov8_models/yolov8_torpedo_flex.pt')
        self.model_buoy = YOLO(PATH + '/yolov8_models/yolov8_buoy.pt')
        self.model_bin = YOLO(PATH + '/yolov8_models/yolov8_bin_rs.pt')
        self.model_gate = YOLO(PATH + '/yolov8_models/yolov8_gate_new.pt')


        #self.model_gate.eval()
        #self.model_bin.eval() #no need as yolov8
        

    def predict(self,image,model_name="buoy"):
        """
        image: opencv mat
        boxes has format
        x1 (pixels)  y1 (pixels)  x2 (pixels)  y2 (pixels)   confidence  class
        Ex: [0, 0, 10, 10, 1, 1]
        """
        print(model_name)
        use_yolov8 = True if model_name == "flare" or model_name == "torpedo" or model_name == "torpedo_flex" or model_name == "buoy" or model_name == "bin" or model_name=="gate" else False

        if model_name == "gate":
            results = self.model_gate(image)
        if model_name=="buoy":
            results = self.model_buoy(image)
        if model_name=="torpedo":
            results = self.model_torpedo(image)
        if model_name=="torpedo_flex":
            results = self.model_torpedo_flex(image)
        if model_name=="octagon":
            pass
            #results = self.model_octagon(image)
        if model_name=="bin":
            results = self.model_bin(image)

        if use_yolov8:
            boxes = results[0].boxes.data.detach().cpu().numpy()
        else:
            boxes = results.xyxy[0].detach().cpu().numpy()

        if model_name=="octagon":
            pass
            #boxes[:,-1] += 22
        elif model_name == "bin":
            boxes[:, -1] += 10
        elif model_name == "torpedo":
            boxes[:, -1] += 3
        elif model_name == "torpedo_flex":
            boxes[:, -1] = 7
        elif model_name == "buoy":
            boxes[:, -1] = 2
        elif model_name == "gate":
            for i in range(len(boxes)):
                boxes[i][-1] += 8


        return boxes

    def debug_draw(self, image):
        """
        @Todo: Implement Method To Return Rendered Image With BBox
        """
        results=self.model_buoy(image)
        results.display(save=True, save_dir=Path(src_path+"/test_output"), image_name="image_"+str(int(round(datetime.now().timestamp())))+".jpg")
        #results.save(save_dir=PATH+"/test_output/photo_rand_"+str(random.randint(0, 100000)))


#!/usr/bin/python3

import rclpy
import numpy as np
from scipy.spatial.transform import Rotation
from auv_msgs.msg import AuvState, BoundingBox, VisionSetpoints, ObjectDetected
from math import sqrt
from .yolo_v5 import YoloV5
from .cv_main import CV
import os
from ament_index_python.packages import get_package_share_directory
import cv2
from .vision_constants import get_models, get_task_details


package_path = get_package_share_directory('auv_vision')
PATH = package_path + '/src'

workspace_path = '~/ros2_ws'
workspace_path = os.path.expanduser(workspace_path)
package_name = 'auv_vision'
src_path = os.path.join(workspace_path, 'src', 'Robosub_ROS2_2024', package_name, package_name)


class MainDetector:
    def __init__(self, task_details, camera="front",
        config={"img_height": 20, "img_width": 18}, zfactor=3000.0 * 1.8, xyfactor=1, objects = None): #750, #0.25 for torpedo
        print('Inside Main Detector')
        self.vision_mode = 'cv'
        self.vision_yolo = YoloV5()
        self.vision_cv = CV()
        
        self.objects = objects
        self.task_details = task_details
        self.camera = camera
        self.config = config
        self.zfactor = zfactor
        self.xyfactor = xyfactor

        self.cur_pose = AuvState()
        
        self.counter = 0

    def object_id_to_name(self, object_id):
        for object in self.objects:
            if object.object_id==object_id:
                return object.object_name

    def object_name_to_min_confidence(self, object_name):
        for object in self.objects:
            if object.object_name==object_name:
                return object.min_confidence
        return 0

    def get_vision_model(self, task):
        models = get_models()
        return models[task]

    def filter_bounding_boxes(self, bounding_boxes):
        """
        Returns a dict of best BoundingBox for each class detected
        """
        best_bboxes = {}
        for bbox in bounding_boxes:
            if not bbox.class_id in best_bboxes:
                best_bboxes[bbox.class_id] = bbox
                continue
            # Taking highest confidence for all objects except torpedo
            # hole (torpedo_1) where we went to get a specific side
            if True:
                if best_bboxes[bbox.class_id].confidence < bbox.confidence:
                    best_bboxes[bbox.class_id] = bbox
            elif bbox.class_id == 3:
                if self.torpedo_side == 'bottom':
                    if best_bboxes[bbox.class_id].ymax < bbox.ymax:
                        best_bboxes[bbox.class_id] = bbox
                if self.torpedo_side == 'top':
                    if best_bboxes[bbox.class_id].ymin > bbox.ymin:
                        best_bboxes[bbox.class_id] = bbox
                if self.torpedo_side == 'right':
                    if best_bboxes[bbox.class_id].xmax < bbox.xmax:
                        best_bboxes[bbox.class_id] = bbox
                if self.torpedo_side == 'left':
                    if best_bboxes[bbox.class_id].xmin > bbox.xmin:
                        best_bboxes[bbox.class_id] = bbox
                if self.torpedo_side == 'biggest':
                    if ((best_bboxes[bbox.class_id].xmax - best_bboxes[bbox.class_id].xmin) * (best_bboxes[bbox.class_id].ymax - best_bboxes[bbox.class_id].ymin)) < \
                            ((bbox.xmax - bbox.xmin) * (bbox.ymax - bbox.ymin)):
                        best_bboxes[bbox.class_id] = bbox

        return best_bboxes

    def get_bounding_boxes(self, img_msg):
        # Raw Image -> CV Image
        img_buffer = (np.frombuffer(img_msg.data, dtype=np.uint8, count=-1))
        img = np.resize(img_buffer,(img_msg.height,img_msg.width,3))
        img2 = np.resize(img_buffer,(img_msg.height,img_msg.width,3))

        # img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
        # img2 = cv2.cvtColor(img2, cv2.COLOR_BGR2RGB)

        # Fish eye calibration attempted
        # img = self.fish_eye_calibration(img)

        # cv2.imwrite("/home/auv/catkin_ws/src/matsya/auv_vision/scripts/basler_images" + "image_"+str(int(round(datetime.now().timestamp())))+".jpg", img)

        # img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
        if self.counter % 10 == 0 or True:
            os.chdir(src_path+'/raw_imgs')
            cv2.imwrite("raw_img_"+str(int(self.counter/10))+".jpg", img)
            os.chdir(src_path)
        #img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)

        self.counter += 1

        if self.get_vision_model(self.task_name) == 'CV':
            raw_bboxes = self.vision_cv.predict(img,self.task_name, self.counter)
        else:
            raw_bboxes = self.vision_yolo.predict(img,self.task_name)
        
        ######### To print live z values #########

        ##### Save Raw detections ######
        if len(raw_bboxes) > 0:
            # if self.counter % 35 == 1:
            #     self.vision_model.debug_draw(img)
            #     pass
            raw_img = None
            for raw_box in raw_bboxes:
                raw_img = cv2.rectangle(img2, (int(raw_box[0]), int(raw_box[1])), (int(raw_box[2]), int(raw_box[3])), (0,0,255), thickness=3, lineType = cv2.LINE_8)
                raw_img = cv2.putText(raw_img, str(self.object_id_to_name(int(raw_box[5])))+": "+str(round(raw_box[4], 2)), (int(raw_box[0]), int(raw_box[1])),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0,0,0), 1)

            # mycwd = os.getcwd()
            os.chdir(src_path +'/detections')
            cv2.imwrite('raw_bbox_'+str(self.counter)+'.jpg', raw_img)#cv2.cvtColor(raw_img, cv2.COLOR_RGB2BGR))
            os.chdir(src_path)
        #############################


        # Conversion To Standard Format
        bboxes = []
        for raw_box in raw_bboxes:
            bbox = BoundingBox()
            bbox.xmin = int(raw_box[0])
            bbox.ymin = int(raw_box[1])
            bbox.xmax = int(raw_box[2])
            bbox.ymax = int(raw_box[3])
            bbox.confidence = float(raw_box[4])
            bbox.class_id = int(raw_box[5])
            bbox.class_name = self.object_id_to_name(bbox.class_id)
            bboxes.append(bbox)
        filtered_bboxes = self.filter_bounding_boxes(bboxes)
        # filtered_bboxes is a dict

        final_bboxes = []
        for bbox_key in filtered_bboxes.keys():
            bbox = filtered_bboxes[bbox_key]
            if bbox.confidence > self.object_name_to_min_confidence(bbox.class_name):
                final_bboxes.append(bbox)


        if len(final_bboxes) > 0:
            final_img = None
            for bbox in final_bboxes:
                final_img = cv2.rectangle(img, (int(bbox.xmin), int(bbox.ymin)), (int(bbox.xmax), int(bbox.ymax)), (36,255,12), thickness=2, lineType = cv2.LINE_8)
                final_img = cv2.putText(final_img, str(bbox.class_name)+": "+str(round(bbox.confidence, 2)), (int(bbox.xmin), int(bbox.ymin)-10), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (36,255,12), 2)
            ####### Save detections ######
            # mycwd = os.getcwd()
            os.chdir(src_path+'/detections')
            cv2.imwrite('final_bbox_'+str(self.counter)+'.jpg', final_img) #cv2.cvtColor(final_img, cv2.COLOR_RGB2BGR))
            os.chdir(src_path)
            ###############################
            # self.bbox_image_pub.publish(self.br.cv2_to_imgmsg(final_img, encoding='bgr8'))
        else:
            # self.bbox_image_pub.publish(self.br.cv2_to_imgmsg(img, encoding='bgr8'))
            pass


        return final_bboxes

    def compute_midpoint(self, bounding_box, object):
        actual_object_length = object.object_length
        actual_object_height = object.object_height
        img_height = self.config["img_height"]
        img_width = self.config["img_width"]

        image_object_height = bounding_box.ymax-bounding_box.ymin
        image_object_width = bounding_box.xmax-bounding_box.xmin
        image_object_cent_x = (bounding_box.xmin+bounding_box.xmax)/2
        image_object_cent_y = (bounding_box.ymin+bounding_box.ymax)/2

        image_object_area = image_object_height * image_object_width
        image_object_radius = sqrt(image_object_area)/2

        actual_object_size = sqrt(actual_object_length*actual_object_height)/2

        # Can compute actual object size from length and width
        height_scale_factor = actual_object_height/image_object_height
        area_scale_factor = actual_object_size/image_object_radius

        scale_factor = min(height_scale_factor, area_scale_factor)

        midpoint_z = self.zfactor * scale_factor
        midpoint_x = self.xyfactor * scale_factor * (image_object_cent_x - img_width/2)
        midpoint_y = self.xyfactor * scale_factor * (image_object_cent_y - img_height/2)

        # camera computes the midpoint in a different axis
        return [midpoint_z, midpoint_x, midpoint_y], image_object_height, image_object_width


    def run_detection(self, img_msg, task_name):
        self.task_name = task_name
        bboxes = self.get_bounding_boxes(img_msg)

        # Print z values in live feed with bounding boxes
        img_buffer = (np.frombuffer(img_msg.data, dtype=np.uint8, count=-1))
        img = np.resize(img_buffer,(img_msg.height,img_msg.width,3))
        img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
        if len(bboxes) > 0:
            final_img = None
            for bbox in bboxes: 
                rel_pose = [[-1,-1,-1],-1,-1]
                #rel_pose = -1
                rel_pose_list = []
                for object in self.objects:
                    if object.object_id == bbox.class_id:
                        rel_pose = self.compute_midpoint(bbox, object)
                        rel_pose_list.append(rel_pose)
                final_img = cv2.rectangle(img, (int(bbox.xmin), int(bbox.ymin)), (int(bbox.xmax), int(bbox.ymax)), (36,255,12), thickness=2, lineType = cv2.LINE_8)
                final_img = cv2.putText(final_img, str(bbox.class_name)+": "+str(round(bbox.confidence, 2)) + " pos_value: " +str(rel_pose[0]), (int(bbox.xmin), int(bbox.ymin)-10), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (36,255,12), 2)
                #final_img = cv2.putText(final_img, str(bbox.class_name)+": "+str(round(bbox.confidence, 2)) + " zvalue: " +str(round(rel_pose[0][0], 1)), (int(bbox.xmin), int(bbox.ymin)-10), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (36,255,12), 2)
            
            # Save detections
            # mycwd = os.getcwd()
            # os.chdir(PATH+'/detections')
            # cv2.imwrite('final_bbox_'+str(self.counter)+'.jpg', cv2.cvtColor(final_img, cv2.COLOR_RGB2BGR))
            # os.chdir(mycwd)
            img = final_img
        else:
            print("NO DETECTIONS")
            return
        return final_img, bboxes, rel_pose_list

if __name__ == '__main__':
    main_det = MainDetector(get_task_details)

import os
from .water_removal import remove_water
import cv2
import matplotlib.pyplot as plt 
import numpy as np 
from .constants import morning, color_const, min_area, eve_color_const, eve_min_area

if not morning:
    color_const = eve_color_const
    min_area = eve_min_area

class Buoy:
    def __init__(self, color='buoy_red'):
        self.l_bound, self.u_bound = color_const[color]
        self.min_area = min_area
        self.i=0
        self.frame_num=0

    def get_hue_mask(self, img):
        hue_mask = cv2.inRange(img, np.array(self.l_bound), np.array(self.u_bound))
        return hue_mask

    def post_color(self, masked_image):
        gray_img = cv2.cvtColor(masked_image, cv2.COLOR_BGR2GRAY)
        kernel = np.ones((4,4), np.uint8) #6, 6
        # closed_img = cv2.morphologyEx(gray_img, cv2.MORPH_CLOSE, kernel)
        eroded_img = cv2.erode(gray_img, kernel, iterations=2) #5
        blur_image = cv2.GaussianBlur(eroded_img, (3,3), 0)
        return blur_image

    def get_bbox(self,img, hsv_img):
        mask_hue = self.get_hue_mask(hsv_img)
        masked_image = cv2.bitwise_and(img, img, mask=mask_hue)
        blur_image = self.post_color(masked_image)
        contours,_ = cv2.findContours(blur_image,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
        # detected = False
        contour_img = None

        big = 0

        for c in contours:
            x,y,w,h = cv2.boundingRect(c)
            
            if cv2.contourArea(c) > self.min_area and 0.8 < w/h < 1.25:
                big = max(big, w*h)  #To take only the biggest detection

        for contour in contours:
           x,y,w,h = cv2.boundingRect(contour)
           if cv2.contourArea(contour) > self.min_area and 0.8 < w/h < 1.25 and w*h == big: #0.8-1.2(w/h)
                raw_bbox = np.array([np.array([x, y, x+w, y+h, 1, 2])])
                contour_img = cv2.rectangle(img,(x,y),(x+w,y+h),(0,255,0),2)
                if self.frame_num%1 == 0:
                    cur_dir = os.getcwd()
                    img_dir = os.path.join(os.getcwd(), 'bbox')
                    os.chdir(img_dir)
                    #print("WRITING TO BBOX")
                    cv2.imwrite('bbox_'+str(int(self.frame_num/1))+'.jpg', contour_img)
                    os.chdir(cur_dir)
                
                return raw_bbox
           
            # detected=True
        
        return np.array([])

    def detect(self,img, frame_num=0):
        #cv2.imwrite(f'/home/auv/catkin_ws/src/matsya/auv_vision/scripts/cv/{}.jpg', img)
        self.frame_num = frame_num
        p_img = remove_water(img, frame_num)
        hsv_img = cv2.cvtColor(p_img, cv2.COLOR_BGR2HSV)
        raw_bbox = self.get_bbox(img, hsv_img)
        return raw_bbox

if __name__ == '__main__':
    buoy = Buoy()
    img_path = '/home/auv/catkin_ws/src/matsya/auv_vision/scripts/cv_test_images'
    image_list = os.listdir(img_path)
    for image in image_list:
        img = cv2.imread(os.path.join(img_path, image), cv2.IMREAD_COLOR)
        var = buoy.detect(img)
        print(var)


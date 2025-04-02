import cv2 
import numpy as np 
from .constants import water_removal_constants
import matplotlib.pyplot as plt 
import os

#(75,99),(78, 102)

def remove_water(input_img, frame_num=0):
    """
    REMOVES WATER BY REMOVING CERTAIN HUE VALUES BASED ON THE HISTOGRAM
    """
    #defining constants
    hmask_sat_l , hmask_sat_u = water_removal_constants["hue_mask_constants"]["saturation_range"]
    hmask_val_l, hmask_val_u = water_removal_constants["hue_mask_constants"]["value_range"]
    smask_hue_l, smask_hue_u = water_removal_constants["saturation_mask_constants"]["hue_range"]
    smask_sat_l,smask_sat_u = water_removal_constants["saturation_mask_constants"]["saturation_range"]
    smask_val_l,smask_val_u = water_removal_constants["saturation_mask_constants"]["value_range"]

    #converting to hsv and blurring 
    hsv_img = cv2.cvtColor(input_img, cv2.COLOR_BGR2HSV)
    gblur_img = cv2.GaussianBlur(hsv_img, (1,5), 0) 
    gblur_original = cv2.GaussianBlur(input_img, (1,9), 0)
    
    #finding hue with max frequency 
    h_hist = cv2.calcHist(gblur_img[0], [1], None, [256], [0,256], True, False)
    _, maxima, _, max_loc = cv2.minMaxLoc(h_hist)

    #draw hue graph if needed while debugging
    # plt.bar(range(256), h_hist.flatten(), width=4)
    # plt.show()

    # removing water using range of hue_values and tuneable saturation and values 
    mask_hue = cv2.inRange(gblur_img, np.array([75 , hmask_sat_l, hmask_val_l]), np.array([105, hmask_sat_u, hmask_val_u]))

    # removing the light reflection on pool's floor based on saturation values 
    mask_saturation = cv2.inRange(gblur_img, np.array([smask_hue_l, smask_sat_l, smask_val_l]), np.array([smask_hue_u, smask_sat_u, smask_val_u]))

    #taking the intersection of two masks and inverting them 
    final_mask = cv2.bitwise_or(mask_hue, mask_saturation)
    inverted_mask = cv2.bitwise_not(final_mask)

    #morphing images to reduce noise
    #kernel sizes can be tuned as neccesary 
    kernel = np.ones((8,4), np.uint8)
    kernel_wide = np.ones((4,8), np.uint8)
    morphed_mask = cv2.morphologyEx(inverted_mask, cv2.MORPH_OPEN, kernel)
    closed_mask = cv2.morphologyEx(morphed_mask, cv2.MORPH_CLOSE, kernel_wide)

    #applying the inverted mask on the original image to get image only with objects of intrest 
    masked_img = cv2.bitwise_and(gblur_original, gblur_original, mask=closed_mask)

    #########
    #if frame_num%10 == 0:
    #    cur_dir = os.getcwd()
    #    img_dir = os.path.join(os.getcwd(), 'water_removed')
    #    os.chdir(img_dir)
    #    cv2.imwrite('wr_'+str(int(frame_num/10))+'.jpg', masked_img)
    #    os.chdir(cur_dir)
    #########
    return masked_img


if __name__ == "__main__" :
    img = cv2.imread('/Users/sparsh/Desktop/auv_vision/final_pipeline/actual_gate_left.jpg', cv2.IMREAD_COLOR)
    processed_img = remove_water(img)
    
    # showing pre and post process image together 
    plt.figure(figsize=(10, 5))
    plt.subplot(1, 2, 1)
    plt.imshow(cv2.cvtColor(img, cv2.COLOR_BGR2RGB))
    plt.title('pre processing')

    plt.subplot(1, 2, 2)
    plt.imshow(cv2.cvtColor(processed_img, cv2.COLOR_BGR2RGB))
    # plt.imshow(processed_img)
    plt.title('post processing')

    plt.show()




import os
import cv2
import matplotlib.pyplot as plt 
import numpy as np 

class Face:
    def __init__(self, color='bin_red'):
        pass

    def detect(self,img, frame_num=0):
        import cv2

        # Load the Haar Cascade Classifier
        face_cascade = cv2.CascadeClassifier(cv2.data.haarcascades + 'haarcascade_frontalface_default.xml')

        # Read the image (change the path to your image file)

        # Convert the image to grayscale (required for Haar cascades)
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

        # Detect faces
        faces = face_cascade.detectMultiScale(gray, scaleFactor=1.1, minNeighbors=5, minSize=(30, 30))

        # Draw rectangles around detected faces
        if len(faces) == 0:
            return [np.array([0, 0, 1280, 720, 1, 0])]
        bboxes = []
        for (x, y, w, h) in faces:
            bboxes.append(np.array([x, y, x + w, y + h, 1, 0]))
        if len(bboxes) > 0:
            return bboxes
        
        
        #return [np.array([0.3 * 1280, 0.3 *720, 0.7 * 1280, 0.7 * 720, 1, 0])]

if __name__ == '__main__':
    bin = Face()
    img_path = '/home/auv/catkin_ws/src/matsya/auv_vision/scripts/cv_test_images'
    image_list = os.listdir(img_path)
    for image in image_list:
        img = cv2.imread(os.path.join(img_path, image), cv2.IMREAD_COLOR)
        var = bin.detect(img)
        print(var)
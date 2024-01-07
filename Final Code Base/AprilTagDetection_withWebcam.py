"""
AprilTagDetection_withWebcam script is prepared for testing the detecting functions and algorithms without using the robot. 
The way it handles the AprilTag following process is identical with AprilTagFollower_withCNNsupport.py script. 

The script takes a picture with the webcam and gives a decision for the Robot. If regular AprilTag detection function fails to detect the AprilTag,
it initiates the CNN model to make a decision. The sequence is decided in this order due to the fact that AprilTag detection function is more
accurate and fast compared to CNN model prediction. 
"""

import os
import cv2
import time
from apriltag import apriltag
import yaml
import numpy as np
from keras.models import load_model
import tensorflow as tf

#Initiating the webcam and CNN model.
#If there's an external camera connected, or another camera is intented to use, please adjust the index in paranthesis. 0 is the default camera.
cap = cv2.VideoCapture(0)
model = load_model('modelCNN.h5')

folder = os.path.dirname(os.path.abspath(__file__))

# Reading the calibration results from a yaml file. Camera calibration is handled with another script and the results are saved.
with open(r'calibrationResults.yaml') as file:
    data = yaml.full_load(file)
    cameraMatrix = data['camMatrix']
    dist = data["dist"]
 

# tag36h11 is a standart AprilTag family. 
detector = apriltag("tag36h11")
cameraMatrix = np.array(cameraMatrix)
dist = np.array(dist)

#tag_size variable is needed to be adjusted for different tags.
tag_size = 0.1
ob_pt1 = [-tag_size/2, -tag_size/2, 0.0]
ob_pt2 = [ tag_size/2, -tag_size/2, 0.0]
ob_pt3 = [ tag_size/2,  tag_size/2, 0.0]
ob_pt4 = [-tag_size/2,  tag_size/2, 0.0]
ob_pts = ob_pt1 + ob_pt2 + ob_pt3 + ob_pt4

object_pts = np.array(ob_pts).reshape(4,3)
objPointsArr = []
objPointsArr.append(object_pts)
objPointsArr = np.array(objPointsArr)

object_points = objPointsArr

while True:
    ret, frame = cap.read()

    cv2.imshow("Webcam", frame)
    cv2.waitKey()
    
    cv2.imwrite(folder + "/image.jpg", frame)
    
    #Before this part, image is captured and saved. 
    #After this part is reading the image and deciding the direction.
    #The script prepared this way, so that a pre recorded image can also be used with commenting out the cv2.imshow-cv2.imwrite.
    
    img = cv2.imread(folder + "/image.jpg")
    gray = cv2.cvtColor(img, cv2.COLOR_RGB2GRAY)
    
    img_cx = img.shape[1] / 2
    
    try: 
        detections = detector.detect(gray)
        if len(detections) != 0:
            for item in detections:

                corners = np.array([item['lb-rb-rt-lt'] for item in detections])

                #Uncomment the below part for putting a triangle in the detected AprilTag

                # x1 = int(corners[0][0][0])
                # y1 = int(corners[0][0][1])
                # x2 = int(corners[0][2][0])
                # y2 = int(corners[0][2][1])
                
                # cv2.rectangle(img, (x1, y1), (x2, y2), (0,0,255), 5)
                # cv2.imshow('Apriltags', img)
                # cv2.waitKey()  
                cv2.imwrite(folder + "/image1.jpg", frame)
                                                                                                      

                success, rotation_vector, translation_vector = cv2.solvePnP(object_points, corners, cameraMatrix ,dist,cv2.SOLVEPNP_IPPE_SQUARE)
                x = translation_vector[0]
                z = translation_vector[2]

                # The thresholds for direction decisions are determined with trial-error methodology.
                if z <= 0.4:
                    print('Reached the April Tag')    

                elif z >= 0.4:
                    if (x >= 0.08):
                        print("Turning Right")

                    elif x <= -0.08:
                        print("Turning Left") 
                                                    
                    else:
                        print('Going Forward')

    except: 
        
        print('Initiating CNN Model.')

        img1 = img
        img_array = tf.keras.utils.img_to_array(img1)
        img_array = tf.expand_dims(img_array, 0) # Create a batch
        img_array = tf.image.resize(img_array, [256, 256], method='nearest')


        x = model.predict(img_array)
        img_cx = 128 # center x pixel of a 256x256 image.

        #Direction intervals is decided with trial-error methodology.
        if x/img_cx > 1.1:
            print("Turning Right")
        elif x/img_cx < 0.9:
            print("Turning Left")
        else:
            print('Going Forward')



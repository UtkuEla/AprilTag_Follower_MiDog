#!usr/bin/python
import threading
import os, sys
import cv2
import time
from apriltag import apriltag
import yaml
sys.path.append("/home/pi/OpenCat/serialMaster") # set the path to where Opencat is located on your Pi
from ardSerial import *
import Camera
import numpy as np


class AprilTag:

    def __init__(self, frame=None):
        self.frame = frame
        self.stopped = False
        self.command = ""  # can contain gaits and skills
        self.key = 255  # the last key that was pressed for remote control
        self.goodPorts = {}
        connectPort(self.goodPorts) # for connecting to the NyBoard
        self.video_getter = Camera.VideoGet(0)
        self.video_getter.start()
        self.tag_size = 0.1
            
    def getPicture(self):

        #video_getter = Camera.VideoGet(0)
        #video_getter.start()
        image = self.video_getter.frame

        return image


    def start(self):
        threading.Thread(target=self.localize, args=()).start()
        return self

    def leftOrRight(self):
        
        model = tag.loadCNNModel()
        detector = apriltag("tag36h11")
        cameraMatrix ,dist = tag.calibrationResults()
        cameraMatrix = np.array(cameraMatrix)
        dist = np.array(dist)
        object_points = tag.objectPoints()
        #print(cameraMatrix)

        while True:
            img = self.getPicture()
            gray = cv2.cvtColor(img, cv2.COLOR_RGB2GRAY)
            
            try: 
                detections = detector.detect(gray)
                #print(detections[0])
                if len(detections) != 0:
                    for item in detections:

                        corners = np.array([item['lb-rb-rt-lt'] for item in detections])
                        success, rotation_vector, translation_vector = cv2.solvePnP(object_points, corners, cameraMatrix ,dist,cv2.SOLVEPNP_IPPE_SQUARE)
                        x = translation_vector[0]
                        z = translation_vector[2]
                        #print(translation_vector)
                        if z <= 0.4:
                            print('Reached the April Tag')    
                            self.command = 'kbalance'
                            send(self.goodPorts, [self.command, "\n"])
                        elif z >= 0.4:
                            if (x >= 0.08):
                                print("Turning Right") #add turn comment, can be also used with return flag = 'right'
                                self.command = 'kcrR'
                                send(self.goodPorts, [self.command, "\n"])

                            elif x <= -0.08:
                                print("Turning Left") #add turn comment, can be also used with return flag = 'left'
                                self.command = 'kcrL'
                                send(self.goodPorts, [self.command, "\n"])
                                                            
                            else:
                                print('Going Forward')
                                self.command = 'kcrF'
                                send(self.goodPorts, [self.command, "\n"])

                else:
                    print('April Tag not found.')
                    self.command = 'kbalance'
                    send(self.goodPorts, [self.command, "\n"]) 
                                       
            except: 
                print('Initiating CNN Model.')
                img_array = img.img_to_array(img)

                input_data = np.expand_dims(img_array, axis=0)

                x = model.predict(input_data)
                print(x)

                img_cx = img.shape[1] / 2

                if x/img_cx > 6/5:
                    self.command = 'kcrR'
                    send(self.goodPorts, [self.command, "\n"])
                elif x/img_cx < 4/5:
                    self.command = 'kcrR'
                    send(self.goodPorts, [self.command, "\n"])
                else:
                    print('Going Forward')
                    self.command = 'kcrF'
                    send(self.goodPorts, [self.command, "\n"])


    def poseEstimation(self,img,cameraMatrix):
        
        # options = apriltag.DetectorOptions(
        #     border=1,
        #     nthreads=4,
        #     quad_decimate=1.0,
        #     quad_blur=0.0,
        #     refine_edges=True,
        #     refine_decode=False,
        #     refine_pose=True,
        #     debug=False,
        #     quad_contours=True)

        detector = apriltag.Detector()
        gray = cv2.cvtColor(img, cv2.COLOR_RGB2GRAY)
        tag = detector.detect(gray)
        tag_size = 1 #dummy
        cameraParams = [cameraMatrix[0][0],cameraMatrix[1][1],cameraMatrix[0][2],cameraMatrix[1][2]] # fx, fy, cx, cy
        #print(cameraParams)
        pose = detector.detection_pose(tag[0], cameraParams, tag_size)
        print(pose)
        return pose

    def calibrationResults(self):

        with open(r'calibrationResults.yaml') as file:
            data = yaml.full_load(file)
            cameraMatrix = data['camMatrix']
            dist = data["dist"]
            return(cameraMatrix,dist)

    def objectPoints(self):
        ob_pt1 = [-self.tag_size/2, -self.tag_size/2, 0.0]
        ob_pt2 = [ self.tag_size/2, -self.tag_size/2, 0.0]
        ob_pt3 = [ self.tag_size/2,  self.tag_size/2, 0.0]
        ob_pt4 = [-self.tag_size/2,  self.tag_size/2, 0.0]
        ob_pts = ob_pt1 + ob_pt2 + ob_pt3 + ob_pt4
        object_pts = np.array(ob_pts).reshape(4,3)
        objPointsArr = []
        objPointsArr.append(object_pts)
        objPointsArr = np.array(objPointsArr)
        return objPointsArr

    def loadCNNModel(self):
        from keras.models import load_model

        model = load_model('path/to/saved_model.h5')

        return model
    
    def stop(self):
        self.stopped = True



#imagePath = 'image4.jpg'
#img = cv2.imread(imagePath)

tag = AprilTag()


tag.leftOrRight()

#tag.poseEstimation(img, cameraMatrix)




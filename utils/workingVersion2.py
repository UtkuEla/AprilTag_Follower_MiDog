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
        self.objPoints = np.array([[[-0.05, -0.05, 0.0],
                              [0.05, -0.05, 0.0],
                              [0.05, 0.05, 0.0],
                              [-0.05, 0.05, 0.0]]],
                             dtype=np.float32)
            
    def getPicture(self):

        #video_getter = Camera.VideoGet(0)
        #video_getter.start()
        image = self.video_getter.frame

        return image


    def start(self):
        threading.Thread(target=self.localize, args=()).start()
        return self

    def leftOrRight(self):

    
        detector = apriltag("tag36h11")
        cameraMatrix ,dist = tag.calibrationResults()
        cameraMatrix = np.array(cameraMatrix)
        dist = np.array(dist)
        #print(cameraMatrix)

        while True:
            img = self.getPicture()
            cv2.imshow("mat",img)
            cv2.waitKey()
            #print(img)
            gray = cv2.cvtColor(img, cv2.COLOR_RGB2GRAY)
            #img_cx = img.shape[1] / 2
            
            #try:

            detections = detector.detect(gray)
            #print(detections[0])
            if len(detections) != 0:
                for d in detections:
                    #(ptA, ptB, ptC, ptD) = d.corner
                    corners = np.array([d['lb-rb-rt-lt'] for d in detections])
                    success, rotation_vector, translation_vector = cv2.solvePnP(self.objPoints, corners, cameraMatrix ,dist,cv2.SOLVEPNP_IPPE_SQUARE)
                    #print(translation_vector)
                    x = translation_vector[0]
                    z = translation_vector[2]
                    print(translation_vector)
                    
                if (x >= 0.1):
                    print("Turning Right") #add turn comment, can be also used with return flag = 'right'
                    self.command = 'kcrR'
                    send(self.goodPorts, [self.command, "\n"])
                    #time.sleep(0.2)
                    self.command = 'kcrF'
                    send(self.goodPorts, [self.command, "\n"])
                    #send(self.goodPorts, [self.command, "\n"])
                elif x <= -0.1:
                    print("Turning Left") #add turn comment, can be also used with return flag = 'left'
                    self.command = 'kcrL'
                    send(self.goodPorts, [self.command, "\n"])
                    #time.sleep(0.2)
                    self.command = 'kcrF'
                    send(self.goodPorts, [self.command, "\n"])
                    #send(self.goodPorts, [self.command, "\n"])
                elif z >= 0.1: 
                    print('Going Straight')    
                    self.command = 'kcrF'
                    send(self.goodPorts, [self.command, "\n"])

                else:
                    print('blind')    
                    self.command = 'kbalance'
                    send(self.goodPorts, [self.command, "\n"])
                    #send(self.goodPorts, [self.command, "\n"])
            

            #except: 
                #print('Apriltag not found.')
               
                #self.command = 'kbalance'
                #send(self.goodPorts, [self.command, "\n"])                

            time.sleep(2)

        

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
    
    def stop(self):
        self.stopped = True






# class movement():
#      def __init__(self):
#         self.stopped = False
#         self.command = ""  # can contain gaits and skills
#         self.key = 255  # the last key that was pressed for remote control
#         self.goodPorts = {}
#         connectPort(self.goodPorts) # for connecting to the NyBoard





#imagePath = 'image4.jpg'
#img = cv2.imread(imagePath)

tag = AprilTag()


tag.leftOrRight()

#tag.poseEstimation(img, cameraMatrix)




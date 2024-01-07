"""
AprilTagDetection_withCNN script takes pictures continuously and gives instantaneous direction decisions for the Robot to follow the AprilTag.
It is written in a Object Oriented fashion to have a better control over the attributes and methods. 

There are different decions the robot can make, which are 
1-> Going straight to follow the AprilTag 
2-> Turning left or right to follow the AprilTag
3-> Stopping when the AprilTag is reached.
4-> If an April Tag is not found 10 consecutive times, it stops and waits until an AprilTag is detected.  

"""

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
        """
        This function initiates the camera and returns to a picture.
        """
        image = self.video_getter.frame
        return image


    def start(self):
        threading.Thread(target=self.localize, args=()).start()
        return self

    def leftOrRight(self):
        """
        This function is the main method of the class which gives the direction decisions.
        """
        detector = apriltag("tag36h11")
        cameraMatrix ,dist = tag.calibrationResults()
        cameraMatrix = np.array(cameraMatrix)
        dist = np.array(dist)
        object_points = tag.objectPoints()

        while True:
            img = self.getPicture()
            gray = cv2.cvtColor(img, cv2.COLOR_RGB2GRAY)
            
            try: 
                detections = detector.detect(gray)

                if len(detections) != 0:
                    for item in detections:

                        corners = np.array([item['lb-rb-rt-lt'] for item in detections])
                        success, rotation_vector, translation_vector = cv2.solvePnP(object_points, corners, cameraMatrix ,dist,cv2.SOLVEPNP_IPPE_SQUARE)
                        x = translation_vector[0]
                        z = translation_vector[2]

                        if z <= 0.4:
                            print('Reached the April Tag')    
                            self.command = 'kbalance'
                            send(self.goodPorts, [self.command, "\n"])
                        elif z >= 0.4:
                            if (x >= 0.08):
                                print("Turning Right") 
                                self.command = 'kcrR'
                                send(self.goodPorts, [self.command, "\n"])

                            elif x <= -0.08:
                                print("Turning Left") 
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
                print('Apriltag not found.')
                self.command = 'kbalance'
                send(self.goodPorts, [self.command, "\n"])                

    
    def calibrationResults(self):
        """
        This function reads the calibration results from a yaml file. Camera calibration is handled with another script and the results are saved.
        """

        with open(r'calibrationResults.yaml') as file:
            data = yaml.full_load(file)
            cameraMatrix = data['camMatrix']
            dist = data["dist"]
            return(cameraMatrix,dist)

    def objectPoints(self):
        """
        This function calculates the object points for different tag sizes.
        """
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

    def stop(self):
        self.stopped = True


tag = AprilTag()
tag.leftOrRight()


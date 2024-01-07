
import cv2
import numpy as np  
import apriltag
from picamera import PiCamera
from picamera.array import PiRGBArray
from time import sleep
import time


class ApriltagDetector:
    def __init__(self,K,apriltag_family="tag36h11",TAG_SIZE = .1):
        options = apriltag.DetectorOptions(families=apriltag_family,nthreads=4)
        self.detector = apriltag.Detector(options=options)
        self.camera_params = [K[0,0],K[1,1],K[0,2],K[1,2]] #elements from the K matrix
        self.TAG_SIZE = TAG_SIZE #Tag size from Step 1 in meters 

    def detect_tags(self,
                image,
                detector,
                camera_params,
                tag_size=0.0762,
               ):

        gray = image
        detections, dimg = detector.detect(gray, return_image=True)
        overlay = gray // 2 + dimg // 2
        num_detections = len(detections)
        result = []

        for i, detection in enumerate(detections):
            pose, e0, e1 = detector.detection_pose(detection, camera_params, tag_size)
            #result.extend([detection, pose, e0, e1])
            result.append(pose)
            
        return result 
    

    def distance_to_camera(self):
        camera = PiCamera()
        camera.resolution = (640, 480)
        camera.framerate = 32
        rawCapture = PiRGBArray(camera, size=(640, 480))

        # allow the camera to warmup
        time.sleep(0.1)
        
        results = []

        # capture frames from the camera
        for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):
            image = frame.array
            
            gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)  
            results = np.array(self.detect_tags(image=gray,detector=self.detector,camera_params=self.camera_params))
            # show the frame
            # cv2.imshow("Frame", gray)
            
            try:
                dist_to_cam = results[0][2][-1]
                print(f"dist= {dist_to_cam}")
            except:
                pass
                # print("no apriltag detected")
            
            
            key = cv2.waitKey(1) & 0xFF

            # clear the stream in preparation for the next frame
            rawCapture.truncate(0)

            # if the `q` key was pressed, break from the loop
            if key == ord("q"):
                break



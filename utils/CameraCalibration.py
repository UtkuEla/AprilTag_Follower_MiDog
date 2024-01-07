import numpy as np
import cv2
import glob
import apriltag
import json

def calibration():
    """
    Function for camera calibration.
    """
    
    CHECKERBOARD = (6,8) #checkerboard dimension, different for each setting.
    path = './imagess/*.png' #setup this to path for images.


    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
    
    objPoints = []  #3D coordinate frame, vectors of vectors

    imgPoints = []  #2D image frame, vectors of vectors
    

    objp = np.zeros((1, CHECKERBOARD[0] * CHECKERBOARD[1], 3), np.float32)

    objp[0,:,:2] = np.mgrid[0:CHECKERBOARD[0], 0:CHECKERBOARD[1]].T.reshape(-1, 2)
    
    images = glob.glob(path)

    #print(images)

    for image in images:
        #print(image)
        img = cv2.imread(image) #read the images
        gray = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY) # making them grayscale
        
        bool, corners = cv2.findChessboardCorners(gray, CHECKERBOARD, cv2.CALIB_CB_ADAPTIVE_THRESH + cv2.CALIB_CB_FAST_CHECK + cv2.CALIB_CB_NORMALIZE_IMAGE)
        #print(bool)
        if bool == True:
            objPoints.append(objp)
            
            # refining coordinates for given 2d points, not necessary but recommended. 
            cornersRefined = cv2.cornerSubPix(gray, corners, (11,11),(-1,-1), criteria)
            
            imgPoints.append(cornersRefined)
    
            # To display the checkerboard cornerns, can be commented out.
            img = cv2.drawChessboardCorners(img, CHECKERBOARD, cornersRefined, bool)
        else:
            raise('corners are not found in:' + str(image))
        
        
        cv2.imshow('img',img) # to display the calibrated images, can be commented out.
        #cv2.waitKey(0) to inspect the images one by one, can be commented out.
    
    cv2.destroyAllWindows()
    
    bool, camMatrix, dist, rvecs, tvecs = cv2.calibrateCamera(objPoints, imgPoints, gray.shape[::-1], None, None)
        

    return camMatrix, dist;



def poseEstimate(camK, dist):
    tag_sizes = 0.5
    mtx = camK

    image = cv2.imread("image1.png")
    gray = cv2.cvtColor(image,cv2.COLOR_BGR2GRAY)


    # AprilTag detector options
    options = apriltag.DetectorOptions(families='tag36h11',
                                    border=1,
                                    nthreads=4,
                                    quad_decimate=1.0,
                                    quad_blur=0.0,
                                    refine_edges=True,
                                    refine_decode=False,
                                    refine_pose=True,
                                    debug=False,
                                    quad_contours=True)

    detector = apriltag.Detector(options)

    # Detect the apriltags in the image
    detection_results, dimg = detector.detect(gray, return_image=True)

    # Amount of april tags detected
    num_detections = len(detection_results)
    print('Detected {} tags.\n'.format(num_detections))

    imgPointsArr = []
    objPointsArr = []
    opointsArr = []

    if num_detections > 0:
        for i, detection in enumerate(detection_results):

            print('Detection {} of {}:'.format(i + 1, num_detections))
            print()
            print(detection.tostring(indent=2))

            if mtx is not None:
                imagePoints = detection.corners.reshape(1,4,2) 

                tag_size = tag_sizes

                ob_pt1 = [-tag_size/2, -tag_size/2, 0.0]
                ob_pt2 = [ tag_size/2, -tag_size/2, 0.0]
                ob_pt3 = [ tag_size/2,  tag_size/2, 0.0]
                ob_pt4 = [-tag_size/2,  tag_size/2, 0.0]
                ob_pts = ob_pt1 + ob_pt2 + ob_pt3 + ob_pt4
                object_pts = np.array(ob_pts).reshape(4,3)

                opoints = np.array([
                    -1, -1, 0,
                    1, -1, 0,
                    1,  1, 0,
                    -1,  1, 0,
                    -1, -1, -2*1,
                    1, -1, -2*1,
                    1,  1, -2*1,
                    -1,  1, -2*1,
                ]).reshape(-1, 1, 3) * 0.5*tag_size
                    
                imgPointsArr.append(imagePoints)
                objPointsArr.append(object_pts)
                opointsArr.append(opoints)
                dcoeffs = dist
                # mtx - the camera calibration's intrinsics
                good, prvecs, ptvecs = cv2.solvePnP(object_pts, imagePoints, mtx, dcoeffs, flags=cv2.SOLVEPNP_ITERATIVE)
                imgpts, jac = cv2.projectPoints(opoints, prvecs, ptvecs, mtx, dcoeffs)

                # Draws the edges of the pose onto the image
                #draw_boxes(img, imgpts, edges)

    return 0




    #tvecs = tvecs.tolist()
    #rvecs = rvecs.tolist()

camK, dist = calibration()

camMatrix = camK.tolist()
distMatrix = dist.tolist()
calibrationData = {'camMatrix' : camMatrix, 'dist' : distMatrix}

import yaml
    #result = yaml.dump(calibrationData)
    
with open(r'calibrationResults.yaml', 'w') as file:
    documents = yaml.dump(calibrationData, file)


#poseEstimate(camK, dist)
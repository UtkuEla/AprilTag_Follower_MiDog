import numpy as np
import cv2
import glob
import apriltag
import json
import yaml

def calibration():
    """
    The function for camera calibration.
    """
    
    CHECKERBOARD = (6,8) #checkerboard dimension, different for each setting.
    path = './imagess/*.png' #setup this to path for images.

    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
    
    objPoints = []  #3D coordinate frame, vectors of vectors

    imgPoints = []  #2D image frame, vectors of vectors
    

    objp = np.zeros((1, CHECKERBOARD[0] * CHECKERBOARD[1], 3), np.float32)

    objp[0,:,:2] = np.mgrid[0:CHECKERBOARD[0], 0:CHECKERBOARD[1]].T.reshape(-1, 2)
    
    images = glob.glob(path)

    for image in images:
        #print(image)
        img = cv2.imread(image) #read the images
        gray = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY) # making them grayscale
        
        bool, corners = cv2.findChessboardCorners(gray, CHECKERBOARD, cv2.CALIB_CB_ADAPTIVE_THRESH + cv2.CALIB_CB_FAST_CHECK + cv2.CALIB_CB_NORMALIZE_IMAGE)

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
        cv2.waitKey(0) #to inspect the images one by one, can be commented out.
    
    cv2.destroyAllWindows()
    
    bool, camMatrix, dist, rvecs, tvecs = cv2.calibrateCamera(objPoints, imgPoints, gray.shape[::-1], None, None)
        

    return camMatrix, dist;


camK, dist = calibration()

camMatrix = camK.tolist()
distMatrix = dist.tolist()
calibrationData = {'camMatrix' : camMatrix, 'dist' : distMatrix}
    
with open(r'calibrationResults.yaml', 'w') as file:
    documents = yaml.dump(calibrationData, file)

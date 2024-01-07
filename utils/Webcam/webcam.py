import os
import cv2
import time
import apriltag

cap = cv2.VideoCapture(0)
detector = apriltag.Detector()

folder = os.path.dirname(os.path.abspath(__file__))

while True:
    ret, frame = cap.read()

    cv2.imshow("Webcam", frame)

    cv2.imwrite(folder + "/image.jpg", frame)
    
    #Before this part, image is captured and saved. After this part is 
    #reading and deciding the direction.
    
    img = cv2.imread(folder + "/image.jpg")
    gray = cv2.cvtColor(img, cv2.COLOR_RGB2GRAY)
    
    img_cx = img.shape[1] / 2
    
    try:
        tags = detector.detect(gray)

        x = int(tags[0].center[0])

        if x/img_cx > 6/5:
            print("Turning Right")
        elif x/img_cx < 4/5:
            print("Turning Left")
        else:
            print('Going Straight')
    except: 
        print('Apriltag not found.')

    time.sleep(2)


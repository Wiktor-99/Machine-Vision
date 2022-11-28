import cv2
import numpy as np
import time
from picamera2 import Picamera2

# Grab images as numpy arrays and leave everything else to OpenCV.

cv2.startWindowThread()

picam2 = Picamera2()
picam2.configure(picam2.create_preview_configuration(main={"format": 'RGB888', "size": (720, 480)}))
picam2.start()

lower_blue = np.array([90, 120, 185])
upper_blue = np.array([120, 255, 255])

lower_green = np.array([40, 120, 150])
upper_green = np.array([89, 255, 255])

lower_red = np.array([160,100,20])
upper_red = np.array([179,255,255])


kernel = np.ones((3,3),np.uint8)

def undistort(img):
    DIM=(720, 480)
    K=np.array([[302.4177433437569, 0.0, 399.0254022462873], [0.0, 301.3798248377745, 247.11851371392422], [0.0, 0.0, 1.0]])
    D=np.array([[-0.0889478582038578], [0.3952180611257378], [-1.0015149600635633], [0.8761272461627602]])
    map1, map2 = cv2.fisheye.initUndistortRectifyMap(K, D, np.eye(3), K, DIM, cv2.CV_16SC2)
    return cv2.remap(img, map1, map2, interpolation=cv2.INTER_LINEAR, borderMode=cv2.BORDER_CONSTANT)

while True:
    im = picam2.capture_array()
    im2 = undistort(im)
    hsv = cv2.cvtColor(im2, cv2.COLOR_BGR2HSV)
    mask_red = cv2.inRange(hsv, lower_red, upper_red)
    mask_blue = cv2.inRange(hsv, lower_blue, upper_blue)
    mask_green = cv2.inRange(hsv, lower_green, upper_green)

    mask_green = cv2.dilate(mask_green, kernel,iterations = 1)
    mask_red = cv2.dilate(mask_red, kernel,iterations = 1)
    mask_blue = cv2.dilate(mask_blue, kernel,iterations = 1)

    res_red = cv2.bitwise_and(im2,im2, mask= mask_red)
    res_blue = cv2.bitwise_and(im2,im2, mask= mask_blue)
    res_green = cv2.bitwise_and(im2,im2, mask= mask_green)


    res = res_red + res_blue + res_green


    cv2.imshow("Camera 2", im2)
    cv2.imshow("Camera 3", res_red)
    cv2.imshow("Camera 4", res_blue)
    cv2.imshow("Camera 5", res_green)
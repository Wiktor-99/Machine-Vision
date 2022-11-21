import os
import cv2 as cv2
import numpy as np
from picamera2 import Picamera2


def find_circles(picam):
    lower_blue = np.array([100,50,50])
    upper_blue = np.array([150,255,255])

    frame = picam.capture_array()
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, lower_blue, upper_blue)
    res = cv2.bitwise_and(frame,frame, mask= mask)
    _, _, v = cv2.split(res)
    v = cv2.medianBlur(v, 5)
    original_image = frame.copy()

    circles = cv2.HoughCircles(v, cv2.HOUGH_GRADIENT, 1, 20, param1=50, param2=30, minRadius=30, maxRadius=50)

    return circles, original_image

def draw_circles(circles, original_image):
    for c in circles[0,:]:
        cv2.circle(original_image, (c[0], c[1]), c[2], (0 , 255, 0) ,2)
        cv2.circle(original_image, (c[0], c[1]), 2, (0, 0, 255), 3)

    cv2.imwrite('detected.png', original_image)


def write_all_circles_to_pipe(circles, pipe):
    for c in circles[0,:]:
        msg = str(c[0]) + ' ' + str(c[1]) + ' ' + 'b'
        if len(msg) < 20:
            padding = 20 - len(msg)
            msg = msg + padding*' '

        pipe.write(msg)


def main():
    path = "pipe"
    mode = 0o600

    if not os.path.exists(path):
        os.mkfifo(path, mode)
    cv2.startWindowThread()

    picam2 = Picamera2()
    picam2.configure(picam2.create_preview_configuration(main={"format": 'RGB888', "size":  (720, 480)}))
    picam2.start()
    write_pipe = open(path, 'w')

    while(1):
        sign = input("Type f to find circles, type e to close app")
        circles = None

        if sign == 'f':
            if write_pipe.closed:
                write_pipe = open(path, 'w')

            while (circles is None):
                circles,  original_image = find_circles(picam2)
                if circles is not None:
                    circles = np.uint16(np.around(circles))
                    draw_circles(circles, original_image)
                    write_all_circles_to_pipe(circles, write_pipe)
                    write_pipe.close()

        elif sign == 'e':
            break


main()

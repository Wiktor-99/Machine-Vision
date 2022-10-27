import os
import cv2 as cv2
import numpy as np

def find_circles(cap):
    lower_blue = np.array([100,50,50])
    upper_blue = np.array([150,255,255])

    _, frame = cap.read()
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
    str = ""
    for c in circles[0,:]:
        msg = str(c[0]) + ' ' + str(c[1]) + ' ' + 'b'
        if len(msg) < 20:
            padding = 20 - len(msg)
            msg = msg + padding*' '

        os.write(pipe, msg.encode())


def main():
    path= "pipe"
    write_pipe = os.open(path, os.O_WRONLY | os.O_NONBLOCK)
    cap = cv2.VideoCapture(0)

    while(1):
        sign = input("Type f to find circles, type e to close app")
        circles = None

        if sign == 'f':
            while (circles is None):
                circles,  original_image = find_circles(cap)
                if circles is not None:
                    circles = np.uint16(np.around(circles))
                    draw_circles(circles, original_image)
                    write_all_circles_to_pipe(circles, write_pipe)
        elif sign == 'e':
            break

    cap.release()



main()

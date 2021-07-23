#!/usr/bin/env python

import cv2 as cv
import rospy
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

fx = 595.8788545
fy = 595.8788545
cx = 500.5
cy = 400.5


class bcolors:
    HEADER = '\033[95m'
    OKBLUE = '\033[94m'
    OKCYAN = '\033[96m'
    OKGREEN = '\033[92m'
    WARNING = '\033[93m'
    FAIL = '\033[91m'
    ENDC = '\033[0m'
    BOLD = '\033[1m'
    UNDERLINE = '\033[4m'


def detect_potholes(cur_frame):
    img = cv.cvtColor(cur_frame, cv.COLOR_BGR2GRAY)
    edged = cv.Canny(img, 30, 200)
    contours, _ = cv.findContours(
        edged, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)

    cnt = 0
    center = []
    dim = []
    for contour in contours:
        (x, y, w, h) = cv.boundingRect(contour)
        if cv.contourArea(contour) < 60000 and cv.contourArea(contour) > 100:
            cnt = cnt + 1
            cv.rectangle(cur_frame, (x, y), (x+w, y+h), (255, 0, 0), 1)
            center.append((x + w/2, y + h/2))
            dim.append((w, h))

    if(len(contours) > 0):
        print(f"\n{bcolors.OKGREEN} Potholes Detected !")
        print(f"    {bcolors.OKCYAN}Count: {int(cnt)}{bcolors.ENDC}")
        for i in range(cnt):
            print(f"    {i+1}-")
            print(f"      Center: {center[i]}")
            # x_w = (center[i][0]-cx)*0.54 / fx
            # y_w = (center[i][1]-cy)*0.54 / f
            # print(f"     WC: ({x_w},{y_w})")

            print(f"      W,H: {dim[i]} ")

    cv.imshow("World", cur_frame)
    cv.waitKey(500)


def callback(data):
    br = CvBridge()
    cur_frame = br.imgmsg_to_cv2(data)
    detect_potholes(cur_frame)


def recieve_image():
    rospy.init_node('Pothole_detector', anonymous=True)
    rospy.Subscriber('/viratbot/camera1/image_raw', Image, callback)
    rate = rospy.Rate(2)

    rospy.spin()

    cv.destroyAllWindows()


if __name__ == '__main__':
    recieve_image()

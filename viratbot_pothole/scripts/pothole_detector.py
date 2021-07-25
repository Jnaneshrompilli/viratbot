#!/usr/bin/env python

import cv2 as cv
import rospy
import tf
import tf2_ros
import tf2_geometry_msgs
from tf.transformations import euler_from_quaternion, quaternion_matrix, rotation_from_matrix
from tf import TransformerROS
import numpy as np
from sensor_msgs.msg import Image
from geometry_msgs.msg import Pose
from nav_msgs.msg import Odometry
from cv_bridge import CvBridge

hgph = []

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


def convert_frame(input_pose):
    tf_buffer = tf2_ros.Buffer()
    tf_listener = tf2_ros.TransformListener(tf_buffer)

    pose_stamped = tf2_geometry_msgs.PoseStamped()

    pose_stamped.pose = input_pose
    pose_stamped.header.frame_id = 'camera_base'
    pose_stamped.header.stamp = rospy.Time(0)

    try:
        # ** It is important to wait for the listener to start listening. Hence the rospy.Duration(1)
        output_pose_stamped = tf_buffer.transform(
            pose_stamped, 'odom', rospy.Duration(1))
        return output_pose_stamped.pose

    except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
        raise


def detect_potholes(cur_frame):
    global hgph

    img = cv.cvtColor(cur_frame, cv.COLOR_BGR2GRAY)
    edged = cv.Canny(img, 30, 200)
    contours, _ = cv.findContours(
        edged, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)

    cnt = 0
    center = []
    dim = []
    for contour in contours:
        (x, y, w, h) = cv.boundingRect(contour)
        if cv.contourArea(contour) < 60000 and cv.contourArea(contour) > 10:
            cnt = cnt + 1
            cv.rectangle(cur_frame, (x, y), (x+w, y+h), (255, 0, 0), 1)
            center.append((x + w/2, y + h/2))
            dim.append((w, h))

    if(len(contours) > 0):
        print(f"\n{bcolors.OKGREEN} Potholes Detected !")
        print(f"    {bcolors.OKCYAN}Count: {int(cnt)}{bcolors.ENDC}")
        for i in range(cnt):
            print(f"    {i+1}:")
            print(f"      Pixel Corord: {center[i]}")

            b = np.array([[center[i][0]], [center[i][1]], [1]])
            x = np.dot(hgph, b)
            x = x/x[2][0]
            i_pose = Pose()
            i_pose.position.x = x[0][0]
            i_pose.position.y = x[1][0]
            res = convert_frame(i_pose)

            print(f"      (W Coord:  {res.position.x},{res.position.y})")

    cv.imshow("World", cur_frame)
    cv.waitKey(500)


def update_image(data):
    br = CvBridge()
    cur_frame = br.imgmsg_to_cv2(data)
    detect_potholes(cur_frame)

def map_coord():
    global hgph
    pts_src = np.array([
        [1, 800],
        [1000, 800],
        [1000, 43],
        [1, 43],
        [500, 203],
        [500, 389.5],
        [501, 105.5],
        [686.5, 105.5],
        [306.5, 105]])
    pts_dst = np.array([
        [0.1987, 0.3946],
        [0.1987, -0.3946],
        [5.9334, -4.2794],
        [5.9334, 4.2794],
        [1.6845, -0.001],
        [0.769, -0.001],
        [3.1148, -0.001],
        [3.1336, -0.8928],
        [3.1336, 0.9235]])

    hgph, status = cv.findHomography(pts_src, pts_dst)



def recieve_image():
    rospy.init_node('Pothole_detector', anonymous=True)
    map_coord()

    rospy.Subscriber('/viratbot/camera1/image_raw', Image, update_image)

    rospy.spin()

    cv.destroyAllWindows()


if __name__ == '__main__':
    recieve_image()

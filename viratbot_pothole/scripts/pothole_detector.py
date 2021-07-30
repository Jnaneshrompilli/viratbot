#!/usr/bin/env python

import cv2 as cv
import rospy
import struct
import math
from tf import TransformListener, TransformerROS
import tf2_ros
import tf2_geometry_msgs
import numpy as np
from sensor_msgs.msg import Image, PointCloud2, PointField
from sensor_msgs import point_cloud2
from geometry_msgs.msg import Pose, Point, Twist
from tf2_ros import transform_listener
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import Header
from cv_bridge import CvBridge

hgph_matrix = []

pothole_centers = []
pothole_radii = []
pothole_cnt = 0

lin_vel = 0
ang_vel = 0

turn = 1


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


def update_vel(data):
    global lin_vel
    global ang_vel

    lin_vel = data.linear.x
    ang_vel = data.angular.z


def map_coord():
    global hgph_matrix
    pts_src = np.array([
        [1, 800],
        [1000, 800],
        [1000, 43],
        [1, 43],
        [500, 203],
        [500, 389.5],
        [501, 105.5],
        [686.5, 105.5],
        [306.5, 105],
        [198, 223],
        [820, 223],
        [805, 466.5],
        [195, 466.5],
        [807.5, 71],
        [192.5, 71],
        [902, 71.5],
        [98, 71.5]])
    pts_dst = np.array([
        [0.1987, 0.3946],
        [0.1987, -0.3946],
        [5.9334, -4.2794],
        [5.9334, 4.2794],
        [1.6845, -0.001],
        [0.769, -0.001],
        [3.1148, -0.001],
        [3.1336, -0.8928],
        [3.1336, 0.9235],
        [1.52, 0.784],
        [1.52, -0.83],
        [0.589, -0.407],
        [0.589, 0.407],
        [4.161, -1.89],
        [4.161, 1.89],
        [4.161, -2.47],
        [4.161, 2.47]])

    hgph_matrix, status = cv.findHomography(pts_src, pts_dst)


def convert_frame(input_pose, time_stamp):
    tf_buffer = tf2_ros.Buffer()
    tf_listener = tf2_ros.TransformListener(tf_buffer)

    pose_stamped = tf2_geometry_msgs.PoseStamped()

    pose_stamped.pose = input_pose
    pose_stamped.header.frame_id = 'camera_base'

    pose_stamped.header.stamp = rospy.Time(0)

    try:
        output_pose_stamped = tf_buffer.transform(
            pose_stamped, 'odom', rospy.Duration(1))
        return output_pose_stamped.pose

    except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
        raise


def cam_to_odom():
    frame_listener = TransformListener()
    frame_listener.waitForTransform(
        "/odom", "/camera_base", rospy.Time(), rospy.Duration(4.0))

    t = frame_listener.getLatestCommonTime("/odom", "/camera_base")
    (t_pos, r_quat) = frame_listener.lookupTransform("/odom", "/camera_base", t)

    transform = TransformerROS()
    p_matrix = transform.fromTranslationRotation(t_pos, r_quat)

    return p_matrix


def markpoints(centers_wcoord, radii):

    global pothole_centers
    global pothole_radii
    global pothole_cnt

    for i in range(len(radii)):
        present = False
        for j in range(len(pothole_centers)):
            distn = math.dist(centers_wcoord[i], pothole_centers[j])
            if(distn < radii[i]+pothole_radii[j]):
                present = True
        if not present:
            pothole_centers.append(centers_wcoord[i])
            pothole_radii.append(radii[i])

    markerarray = MarkerArray()

    marker = Marker()
    marker.header.frame_id = 'odom'
    marker.lifetime = rospy.Duration()
    marker.type = 2
    marker.action = marker.ADD
    marker.pose.position.z = 0
    marker.pose.orientation.x = 0
    marker.pose.orientation.y = 0
    marker.pose.orientation.z = 0
    marker.pose.orientation.w = 1
    marker.scale.z = 0.01
    marker.color.a = 1
    marker.color.r = 1
    marker.color.b = 1
    marker.color.g = 1

    #print()
    #print(np.array(pothole_centers))

    tmp_size = len(pothole_centers)

    while(pothole_cnt < tmp_size):
        i = pothole_cnt
        marker.pose.position.x = pothole_centers[i][0]
        marker.pose.position.y = pothole_centers[i][1]
        marker.scale.x = pothole_radii[i]
        marker.scale.y = pothole_radii[i]
        marker.id = i
        pothole_cnt += 1

        markerarray.markers.append(marker)

    markerpub.publish(markerarray)


def pointcld_publish(pt_cld):

    fields = [PointField('x', 0, PointField.FLOAT32, 1),
              PointField('y', 4, PointField.FLOAT32, 1),
              PointField('z', 8, PointField.FLOAT32, 1),
              PointField('rgba', 12, PointField.UINT32, 1),
              ]
    header = Header()
    header.frame_id = 'odom'
    pc2 = point_cloud2.create_cloud(header, fields, pt_cld)
    pc2.header.stamp = rospy.Time.now()
    pointcld_pub.publish(pc2)


def detect_potholes(cur_frame, time_stamp):
    global hgph_matrix
    global turn

    proj_matrix = cam_to_odom()

    image = cv.cvtColor(cur_frame, cv.COLOR_BGR2GRAY)
    edged = cv.Canny(image, 30, 200)
    contours, _ = cv.findContours(
        edged, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)

    cnt = 0
    center = []
    pixel_coords = []
    for contour in contours:
        (x, y, w, h) = cv.boundingRect(contour)
        if cv.contourArea(contour) < 60000 and cv.contourArea(contour) > 100:
            pixel_coords.append(contour)
            cv.rectangle(cur_frame, (x, y), (x+w, y+h), (255, 0, 0), 1)
            center.append((x + w/2, y + h/2))
            cnt += 1

    cv.imshow("World", cur_frame)
    cv.waitKey(5)

    centers_wcoord = []

    if(len(contours) > 0):

        for i in range(len(center)):
            xyz = np.array([[center[i][0]], [center[i][1]], [1]])
            x = np.dot(hgph_matrix, xyz)
            x = x/x[2][0]
            i_pose = Pose()
            i_pose.position.x = x[0][0]
            i_pose.position.y = x[1][0]
            ans = np.dot(proj_matrix, np.array(
                [[x[0][0]], [x[1][0]], [0], [1]]))

            centers_wcoord.append([ans[0][0], ans[1][0]])

        radii = []
        pt_cloud = []

        i = 0
        for coord in pixel_coords:

            radius = 0

            coord = np.delete(coord, range(1, coord.shape[0], 2), axis=0)
            num_rows = np.shape(coord)[0]
            if(num_rows >= 10):
                coord = np.delete(coord, range(1, coord.shape[0], 2), axis=0)

            cnt2 = 0

            for pt in coord:

                xyz = np.array([[pt[0][0]], [pt[0][1]], [1]])
                result = np.dot(hgph_matrix, xyz)
                result = result/result[2][0]
                i_pose = Pose()
                i_pose.position.x = result[0][0]
                i_pose.position.y = result[1][0]
                #res = convert_frame(i_pose, time_stamp)

                ans = np.dot(proj_matrix, np.array(
                    [[result[0][0]], [result[1][0]], [0], [1]]))


                x1 = ans[0][0]
                y1 = ans[1][0]
                rgb = 000000
                pt_cloud.append([x1, y1,0,rgb])

                radius += math.dist(centers_wcoord[i], [x1, y1])

                cnt2 += 1

            radii.append(radius/cnt2)
            i += 1

        #print(np.array(pothole_centers))

        if(turn==5):
            print(f"\n\n{bcolors.OKGREEN} Potholes Currently Detected !")
            print(f"    {bcolors.OKCYAN}Count: {int(cnt)}{bcolors.ENDC}")

            for i in range(len(radii)):
                print(f"    {i+1}")
                print(f"      Coordinates: {round(centers_wcoord[i][0],3), round(centers_wcoord[i][1],3)}")
                print(f"      Radius: {round(radii[i],3)}")
                print()
                
            
            turn = 1
        else:
            turn = turn + 1

        #markpoints(centers_wcoord, radii)
        pointcld_publish(pt_cloud)


def update_image(data):
    time_stamp = rospy.Time.now()
    bridge = CvBridge()
    cur_frame = bridge.imgmsg_to_cv2(data)
    
    detect_potholes(cur_frame, time_stamp)


def recieve_image():
    global markerpub
    global pointcld_pub

    rospy.init_node('Pothole_detector', anonymous=True)

    map_coord()

    pointcld_pub = rospy.Publisher(
        'pothole_pointcloud', PointCloud2, queue_size=2)
    markerpub = rospy.Publisher("pothole_marker", MarkerArray, queue_size=10)

    rospy.Subscriber('/cmd_vel', Twist, update_vel)
    rospy.Subscriber('/viratbot/camera1/image_raw', Image, update_image)

    rospy.spin()

    cv.destroyAllWindows()


if __name__ == '__main__':
    recieve_image()

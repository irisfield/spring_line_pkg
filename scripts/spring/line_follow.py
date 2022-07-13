#!/usr/bin/env python3

# https://www.youtube.com/watch?v=AbqErp4ZGgU
# https://medium.com/@mrhwick/simple-lane-detection-with-opencv-bfeb6ae54ec0
# https://towardsdatascience.com/finding-driving-lane-line-live-with-opencv-f17c266f15db

import cv2
import math
import rospy
import numpy as np
from std_msgs.msg import Float32
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge, CvBridgeError
from dynamic_reconfigure.server import Server

from typing import Dict, Tuple, List
from numpy import ndarray
from vec import Vec
from lane_centering import center_lane
from lane_detection import compute_lines
from utils import rows, cols
from spring_line_pkg.cfg import BlobConfig

# global variables
yaw_rate = Float32()
debug_publishers: Dict[str, rospy.Publisher] = {}
cvbridge = CvBridge()

################### callback ###################

def dynamic_reconfigure_callback(config, level):
    global RC
    RC = config
    return config

def image_callback(camera_image):

    try:
        # convert camera_image into an opencv-compatible image
        cv_image = cvbridge.imgmsg_to_cv2(camera_image, "bgr8")
    except CvBridgeError:
        print(CvBridgeError)

    # resize the image
    cv_image = cv2.resize(cv_image, None, fx=0.7, fy=0.7, interpolation=cv2.INTER_AREA)

    # get the dimensions of the image
    width = cv_image.shape[1]
    height = cv_image.shape[0]

    debug_image = cv_image.copy()

    filtered_image = apply_filters(cv_image)
    filtered_roi_image = get_region_of_interest(filtered_image)

    # Find the lanes in the image
    lanes_image = compute_lines(filtered_roi_image, RC, debug_image)
    debug_publish('lanes_image', lanes_image)

    #lane centering
    p0 = Vec(cols(lanes_image)/2, rows(lanes_image) - rows(lanes_image)/10)
    p_diff = center_lane(lanes_image, p0, debug_image=debug_image)
    adjust = p_diff.x
    # rospy.loginfo(f'force vector: {p_diff}')
    debug_publish('debug_final', debug_image)
    make_twist(adjust)

    cv2.imshow("Canny", filtered_roi_image)
    cv2.imshow("Hough Lines", lanes_image)
    cv2.imshow("Springs", debug_image)
    cv2.waitKey(3)
    #rate.sleep()

###############convert to image##############

def debug_publish(name, image: ndarray):
        name = f'/lane_follow_blob_debug/{name}'
        if name not in debug_publishers:
            debug_publishers[name] = rospy.Publisher(name, Image, queue_size=2)
        debug_publishers[name].publish(cvbridge.cv2_to_imgmsg(image))


################### filters ###################

def apply_white_balance(cv_image):

    # convert image to the LAB color space
    lab_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2LAB)

    average_a = np.average(lab_image[:,:,1])
    average_b = np.average(lab_image[:,:,2])

    lab_image[:,:,1] = lab_image[:,:,1] - ((average_a - 128) * (lab_image[:,:,0] / 255.0) * 1.1)
    lab_image[:,:,2] = lab_image[:,:,2] - ((average_b - 128) * (lab_image[:,:,0] / 255.0) * 1.1)

    return cv2.cvtColor(lab_image, cv2.COLOR_LAB2BGR)

def apply_filters(cv_image):

    # helps remove some of the yellow from the sunlight
    balanced_image = apply_white_balance(cv_image)

    # one more time
    balanced_image = apply_white_balance(balanced_image)

    # convert image to the HLS color space
    hls_image = cv2.cvtColor(balanced_image, cv2.COLOR_BGR2HLS)

    # lower and upper bounds for the color white
    lower_bounds = np.uint8([0, RC.light_low, 0])
    upper_bounds = np.uint8([255, 255, 255])
    white_detection_mask = cv2.inRange(hls_image, lower_bounds, upper_bounds)

    # lower and upper bounds for the color yellow
    # lower_bounds = np.uint8([10, 0, 100])
    # upper_bounds = np.uint8([40, 255, 255])
    # yellow_detection_mask = cv2.inRange(hls_image, lower_bounds, upper_bounds)

    # combine the masks
    # white_or_yellow_mask = cv2.bitwise_or(white_detection_mask, yellow_mask)
    balanced_image_with_mask =  cv2.bitwise_and(balanced_image, balanced_image, mask = white_detection_mask)

    # convert image to grayscale
    gray_balanced_image_with_mask = cv2.cvtColor(balanced_image_with_mask, cv2.COLOR_BGR2GRAY)

    # smooth out the image
    kernel = np.ones((5, 5), np.float32) / 25
    smoothed_gray_image = cv2.filter2D(gray_balanced_image_with_mask, -1, kernel)

    # find and return the edges in in smoothed image
    return cv2.Canny(smoothed_gray_image, 200, 255)

def get_region_of_interest(image):

    width = image.shape[1]
    height = image.shape[0]

    width = width / 8
    height = height / 8

    roi = np.array([[

                       [0, height*8],
                       [0, height*5],
                       [width*2, (height*4)-30],
                       [width*5 , (height*4)-30],
                       [width*8, height*7],
                       [width*8, height*8]

                   ]], dtype = np.int32)

    mask = np.zeros_like(image)
    cv2.fillPoly(mask, roi, 255)

    # return the image with the region of interest
    return cv2.bitwise_and(image, mask)


################### algorithms ###################

def make_twist(turn):
    angular_z = - RC.blob_mult * turn
    yaw_rate.data = angular_z
    yaw_rate_pub.publish(yaw_rate)
    return

################### main ###################

if __name__ == "__main__":

    rospy.init_node("follow_line", anonymous=True)

    rospy.Subscriber("/camera/image_raw", Image, image_callback)

    yaw_rate_pub = rospy.Publisher("yaw_rate", Float32, queue_size=1)

    dynamic_reconfigure_server = Server(BlobConfig, dynamic_reconfigure_callback)

    try:
      rospy.spin()
    except rospy.ROSInterruptException:
      pass

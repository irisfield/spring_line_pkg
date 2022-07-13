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

    # filtered_image = apply_filters(cv_image)
    # filtered_roi_image = get_region_of_interest(filtered_image)

    # Find the lanes in the image
    lanes_image = compute_lines(cv_image, RC, debug_image)
    debug_publish('lanes_image', lanes_image)

    #lane centering
    p0 = Vec(cols(lanes_image)/2, rows(lanes_image) - rows(lanes_image)/10)
    p_diff = center_lane(lanes_image, p0, debug_image=debug_image)
    adjust = p_diff.x
    # rospy.loginfo(f'force vector: {p_diff}')
    debug_publish('debug_final', debug_image)
    make_twist(adjust)

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

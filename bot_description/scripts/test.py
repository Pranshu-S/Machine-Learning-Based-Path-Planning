#!/usr/bin/env python

import rospy
import cv2
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import sys
import os
from nav_msgs.msg import Odometry

bridge = CvBridge()

def image_callback2(depth_data):
    try:
        depth_image = bridge.imgmsg_to_cv2(depth_data, "32FC1")
    except CvBridgeError, e:
        print e

    print(depth_image[432][934])


def main(args):
    cv2.destroyAllWindows()
    rospy.init_node('image_listener', anonymous=True)
    # rospy.Subscriber('/odom', Odometry, odom_callback)
    # rospy.Subscriber("/camera/rgb/image_raw",Image, image_callback)
    rospy.Subscriber("/camera/depth/image_raw",Image, image_callback2)
    rospy.spin()
    cv2.destroyAllWindows()



if __name__ == '__main__':
    main(sys.argv)
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

from tf.transformations import euler_from_quaternion

global pose
pose = [0, 0, 0]


def odom_callback(data):
    global pose
    x  = data.pose.pose.orientation.x
    y  = data.pose.pose.orientation.y
    z = data.pose.pose.orientation.z
    w = data.pose.pose.orientation.w
    pose = [data.pose.pose.position.x, data.pose.pose.position.y, euler_from_quaternion([x,y,z,w])[2]]


def image_callback(ros_image):
    print 'here'
    image_name = 'img_(%0.4f)__(%0.4f).jpg' % (pose[0],pose[1]) 
    print 'got an image'

    global bridge
  #convert ros_image into an opencv-compatible image
    try:
        cv_image = bridge.imgmsg_to_cv2(ros_image, "bgr8")
    except CvBridgeError as e:
        print(e)
    cv2.imshow("Image window", cv_image)
    cv2.waitKey(0)
    path = __file__ + '/' + image_name + ".jpg"
    # print path
    x = cv2.imwrite('/home/pranshu/ROS_WORKSPACES/IEEE_P1/src/bot_description/images/' + image_name + ".jpg", cv_image)
    print(x)

def image_callback2(ros_image):

    print 'here'
    image_name = 'depth_img_(%0.4f)__(%0.4f).jpg' % (pose[0],pose[1]) 
    print 'got an image'

    global bridge
  #convert ros_image into an opencv-compatible image
    try:
        cv_image = bridge.imgmsg_to_cv2(ros_image)
    except CvBridgeError as e:
        print(e)
    #from now on, you can work exactly like with opencv
    cv2.imshow("Image window", cv_image)
    cv2.waitKey(0)
    path = __file__ + '/' + image_name + ".jpg"
    # print path
    x = cv2.imwrite('/home/pranshu/ROS_WORKSPACES/IEEE_P1/src/bot_description/images/' + image_name + ".jpg", cv_image)
    print(x)
  
def main(args):
    cv2.destroyAllWindows()
    rospy.init_node('image_listener', anonymous=True)
    rospy.Subscriber('/odom', Odometry, odom_callback)
    rospy.Subscriber("/camera/rgb/image_raw",Image, image_callback)
    rospy.Subscriber("/camera/depth/image_raw",Image, image_callback2)
    rospy.spin()
    cv2.destroyAllWindows()



if __name__ == '__main__':
    main(sys.argv)
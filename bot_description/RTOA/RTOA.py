import cv2
import numpy as np
import tensorflow as tf
from tensorflow import keras
import os
import rospy
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import sys
from nav_msgs.msg import Odometry

bridge = CvBridge()

def image_callback(ros_image):


    global bridge
    #convert ros_image into an opencv-compatible image
    try:
        cv_image = bridge.imgmsg_to_cv2(ros_image, "bgr8")
    except CvBridgeError as e:
        print(e)

    cv2.imshow("Image window", cv_image)
    cv2.waitKey(0)

    bowl_cascade = cv2.CascadeClassifier('bowl/cascade.xml')
    MR_cascade = cv2.CascadeClassifier('Mars Rover/cascade.xml')
    #QC_cascade = cv2.CascadeClassifier('Quadcopter/cascade.xml')
    #wheel_cascade = cv2.CascadeClassifier('bowl/cascade.xml')
    #boxes_cascade = cv2.CascadeClassifier('bowl/cascade.xml')

    # cap=cv2.VideoCapture(0)

    while 1:
        # ret, img = cap.read()
        bowl = bowl_cascade.detectMultiScale(cv_image, 1.3, 5)
        for (x,y,w,h) in bowl:
            cv2.rectangle(cv_image,(x,y),(x+w,y+h),(255,0,255),2)
            cv2.putText(cv_image,'Bowl',(x,y-5),cv2.FONT_HERSHEY_COMPLEX,2,(0,1500),1)
            #roi_gray = gray[y:y+h, x:x+w]
            roi_color = cv_image[y:y+h, x:x+w]
            
        MR = MR_cascade.detectMultiScale(cv_image, 1.3, 5)
        for (x,y,w,h) in MR:
            cv2.rectangle(cv_image,(x,y),(x+w,y+h),(255,0,0),2)
            cv2.putText(cv_image,'MR',(x,y-5),cv2.FONT_HERSHEY_COMPLEX,2,(0,1500),1)
            #roi_gray = gray[y:y+h, x:x+w]
            roi_color = cv_image[y:y+h, x:x+w]

        cv2.imshow('img',cv_image)
        k = cv2.waitKey(30) & 0xff
        if k == 27:
            break

    cap.release()
    cv2.destroyAllWindows()

if '__name__' == '__main__':
    cv2.destroyAllWindows()
    rospy.init_node('image_listener', anonymous=True)
    rospy.Subscriber("/camera/rgb/image_raw",Image, image_callback)
    rospy.spin()
    cv2.destroyAllWindows()
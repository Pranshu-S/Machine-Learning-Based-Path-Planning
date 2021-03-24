#!/usr/bin/env python

import cv2
import numpy as np
from numpy import asarray
import tensorflow as tf
from tensorflow import keras
import os
from matplotlib import pyplot as plt
import rospy
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import sys
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry


global pose
pose = [0, 0, 0]

def odom_callback(data):
    global pose
    x  = data.pose.pose.orientation.x
    y  = data.pose.pose.orientation.y
    z = data.pose.pose.orientation.z
    w = data.pose.pose.orientation.w
    pose = [data.pose.pose.position.x, data.pose.pose.position.y, euler_from_quaternion([x,y,z,w])[2]]


bridge = CvBridge()

global ans
ans = []

#run model
from tensorflow.keras.models import load_model
model= load_model('/home/ros/IEEE/src/bot_description/CNN/CNN.h5')

class_i=-1
class_x=-1
class_y=-1
class_w=-1
class_h=-1

global reached
reached = 0

def prediction_object(image_recieved):
    img_x = image_recieved
    image_recieved = np.array(image_recieved)
    path = '~/IEEE/src/bot_description/cascades/'
    bowl_cascade = cv2.CascadeClassifier('/home/ros/IEEE/src/bot_description/cascades/bowl_cascade.xml')
    MR_cascade = cv2.CascadeClassifier('/home/ros/IEEE/src/bot_description/cascades/MR_cascade.xml')
    QC_cascade = cv2.CascadeClassifier('/home/ros/IEEE/src/bot_description/cascades/QC_cascade.xml')
    wheel_cascade = cv2.CascadeClassifier('/home/ros/IEEE/src/bot_description/cascades/Wheel_cascade.xml')

    img2 = image_recieved
    image_recieved= tf.image.resize(image_recieved,[224,224])
    image_recieved=tf.reshape(image_recieved,[1,224,224,3])
    predictions= model.predict(image_recieved)

    print(predictions)

    #3 = Quadcopter, MARS ROVER
    #1 = bowl
    #cool?

    classNo1=model.predict_classes(image_recieved)
    probabilityValue= np.amax(predictions)
    classNo=classNo1.tolist()

    # rospy.loginfo(classNo)

    class_i=classNo

    image_recieved = np.array(image_recieved)
    # print(image_recieved)
    # print(type(image_recieved))
    # print(image_recieved.shape)

    if classNo[0] is 1:
        bowl = bowl_cascade.detectMultiScale(img2, 1.3, 5)
        for (x,y,w,h) in bowl:
            cv2.rectangle(img_x,(x,y),(x+w,y+h),(255,0,255),2)
            cv2.putText(img_x,'Bowl',(x,y-5),cv2.FONT_HERSHEY_COMPLEX,2,(0,1500),1)
            #roi_gray = gray[y:y+h, x:x+w]
            roi_color = img_x[y:y+h, x:x+w]
            cv2.imshow("img",img_x)
            cv2.waitKey(0)
            # cv2.destroyAllWindows() 
            ans=[classNo[0],x,y,w,h]

    if classNo[0] is 3:
        MR = MR_cascade.detectMultiScale(img2, 1.3, 5)
        for (x,y,w,h) in MR:
            cv2.rectangle(image_recieved,(x,y),(x+w,y+h),(255,0,0),2)
            cv2.putText(image_recieved,'MR',(x,y-5),cv2.FONT_HERSHEY_COMPLEX,2,(0,1500),1)
            #roi_gray = gray[y:y+h, x:x+w]
            roi_color = image_recieved[y:y+h, x:x+w]
            # cv2.imshow(x)
            # cv2.destroyAllWindows()
            ans=[classNo[0],x,y,w,h]

    if classNo[0] is 0:
        QC = QC_cascade.detectMultiScale(img2, 1.3, 5)
        for (x,y,w,h) in QC:
            cv2.rectangle(image_recieved,(x,y),(x+w,y+h),(0,0,255),2)
            cv2.putText(image_recieved,'QC',(x,y-5),cv2.FONT_HERSHEY_COMPLEX,2,(0,1500),1)
            #roi_gray = gray[y:y+h, x:x+w]
            roi_color = image_recieved[y:y+h, x:x+w]
            # cv2.imshow(x)
            # cv2.destroyAllWindows()
            ans=[classNo[0],x,y,w,h]

    if classNo[0] is 2:
        Wheel = wheel_cascade.detectMultiScale(img2, 1.3, 5)
        for (x,y,w,h) in Wheel:
            cv2.rectangle(image_recieved,(x,y),(x+w,y+h),(255,255,0),2)
            cv2.putText(image_recieved,'Wheel',(x,y-5),cv2.FONT_HERSHEY_COMPLEX,2,(0,1500),1)
            #roi_gray = gray[y:y+h, x:x+w]
            roi_color = image_recieved[y:y+h, x:x+w]
            # cv2.imshow(x)
            # cv2.destroyAllWindows()
            ans=[classNo[0],x,y,w,h]

    print(ans)
    global reached
    if not reached and (ans[3]<600 or ans[4]<400):
        velocity_msg.linear.x = 0.2
        velocity_msg.linear.y = 0
        velocity_msg.angular.z = 0
        rospy.loginfo(velocity_msg)
        pub.publish(velocity_msg)
        # rospy.loginfo(pose)
    else:
        reached = 1
        velocity_msg.linear.x = 0
        pub.publish(velocity_msg)

    # if reached:
    #     if (ans[2]>800):
    #         velocity_msg.angular.z = 0.1
    #         pub.publish(velocity_msg)
    #     else:
    #         velocity_msg.angular.z = -0.1
    #         pub.publish(velocity_msg)



global velocity_msg
velocity_msg = Twist()

def image_callback(ros_image):
    global velocity_msg
    global bridge
    global ans
    #convert ros_image into an opencv-compatible image
    try:
        cv_image = bridge.imgmsg_to_cv2(ros_image, "bgr8")
    except CvBridgeError as e:
        print(e)


    cv2.imshow("Image window", cv_image)
    cv2.waitKey(0)

    ans=prediction_object(cv_image)
    rospy.loginfo(ans)


if __name__ == '__main__':
    cv2.destroyAllWindows()
    rospy.loginfo('ehheiasdij')
    rospy.init_node('image_listener', anonymous=True)
    rospy.Subscriber("/camera/rgb/image_raw",Image, image_callback)
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    # rospy.Subscriber('/odom', Odometry, odom_callback)

    rate = rospy.Rate(10) 

    rospy.spin()

    # cv2.destroyAllWindows()
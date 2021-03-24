#!/usr/bin/env python
import rospy
import geometry_msgs.msg
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import math
import numpy as np
from tf import TransformListener
from tf.transformations import euler_from_quaternion
import sys
import tensorflow as tf
from tensorflow.keras.models import load_model
import cv2
import copy
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from std_msgs.msg import Float32MultiArray
# from scipy.optimize import fsolve

# Brings in the SimpleActionClient
import actionlib

# Brings in the .action file and messages used by the move base action
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

# For initialization and storing of Pose
global pose
pose = [0, 0, 0]

# Import Cascades
bowl_cascade = cv2.CascadeClassifier('/home/pranshu/ROS_WORKSPACES/IEEE_P1/src/bot_description/cascades/bowl_cascade.xml')
MR_cascade = cv2.CascadeClassifier('/home/pranshu/ROS_WORKSPACES/IEEE_P1/src/bot_description/cascades/MR_cascade.xml')
QC_cascade = cv2.CascadeClassifier('/home/pranshu/ROS_WORKSPACES/IEEE_P1/src/bot_description/cascades/QC_cascade.xml')
wheel_cascade = cv2.CascadeClassifier('/home/pranshu/ROS_WORKSPACES/IEEE_P1/src/bot_description/cascades/Wheel_cascade.xml')

# Import Model
model= load_model('/home/pranshu/ROS_WORKSPACES/IEEE_P1/src/bot_description/CNN/CNN.h5')

# Create CvBridge
bridge = CvBridge()


#Initialize Values
class_i=-1
class_x=-1
class_y=-1
class_w=-1
class_h=-1

# Fixed Coordinates of Surrounding Features
rover = [0.297462, 5.661100, 0]
qc = [-2.805357, 4.368702, 0]

# Solve Equations
def equations(p):
    global distance_qc
    global distance_rover
    x, y = p
    return ((rover[1] - x)**2 + (rover[0] - y)**2 - distance_rover**2, (qc[1] - x)**2 + (qc[0] - y)**2 - distance_qc**2)

# Callback for odometry readings 
def odom_callback(data):
    global pose
    x = data.pose.pose.orientation.x
    y = data.pose.pose.orientation.y
    z = data.pose.pose.orientation.z
    w = data.pose.pose.orientation.w
    pose = [data.pose.pose.position.x, data.pose.pose.position.y, euler_from_quaternion([x,y,z,w])[2]]

# Rotate bot to a small extent (10 Degrees)
def rotate_bot():

    global pose
    global regions
    
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size = 10)
    rate = rospy.Rate(10)

    velocity_msg = Twist()
    error = 0.01

    z = 1

    # Rotate it 10 Degrees
    target_pose = pose[2] + 0.1745
    if target_pose > 3.14:
        target_pose = target_pose - 2*3.14

    while not (pose[2] < target_pose + error and pose[2] > target_pose - error):
        velocity_msg.linear.x = 0
        velocity_msg.linear.y = 0
        velocity_msg.angular.z = 0.1
        pub.publish(velocity_msg)
        # rospy.loginfo(pose)

    velocity_msg.angular.z = 0
    pub.publish(velocity_msg)

# Detech and measure distance from the object using depth map
def find_obj(RGB_IMG):
    global bridge

    # Convert ros_image into an opencv-compatible image
    try:
        cv_image = bridge.imgmsg_to_cv2(RGB_IMG, "bgr8")
    except CvBridgeError as e:
        print(e)

    # Convert to Numpy Array
    image_recieved0 = np.array(cv_image)
    image_recieved = np.array(cv_image)

    # Resize Image
    image_recieved= tf.image.resize(image_recieved,[224,224])
    image_recieved2=tf.reshape(image_recieved,[1,224,224,3])

    #Predict
    predictions= model.predict(image_recieved2, steps = 1)
    classNo1=model.predict_classes(image_recieved2)
    probabilityValue= np.amax(predictions)
    classNo=classNo1.tolist()

    # Check which object identified
    if classNo[0] is 1:
        bowl = bowl_cascade.detectMultiScale(image_recieved0, 1.3, 5)
        for (x,y,w,h) in bowl:
            cv2.rectangle(image_recieved0,(x,y),(x+w,y+h),(255,0,255),2)
            cv2.putText(image_recieved0,'Bowl',(x,y-5),cv2.FONT_HERSHEY_COMPLEX,2,(0,1500),1)
            #roi_gray = gray[y:y+h, x:x+w]
            roi_color = img_x[y:y+h, x:x+w]
            cv2.imshow("result",image_recieved0)
            # cv2.waitKey(0)
            cv2.destroyAllWindows() 
            ans=[classNo[0],x,y,w,h]

    # If Mars Rover Identified
    if classNo[0] is 3:
        MR = MR_cascade.detectMultiScale(image_recieved0, 1.3, 5)
        for (x,y,w,h) in MR:
            cv2.rectangle(image_recieved0,(x,y),(x+w,y+h),(255,0,0),2)
            cv2.putText(image_recieved0,'MR',(x,y-5),cv2.FONT_HERSHEY_COMPLEX,2,(0,1500),1)
            #roi_gray = gray[y:y+h, x:x+w]
            roi_color = image_recieved[y:y+h, x:x+w]
            cv2.imshow('result',image_recieved0)
            cv2.waitKey(0)
            cv2.destroyAllWindows()
            ans=[classNo[0],x,y,w,h]

    # If QuadCopter identified
    if classNo[0] is 0:
        QC = QC_cascade.detectMultiScale(image_recieved0, 1.3, 5)
        for (x,y,w,h) in QC:
            cv2.rectangle(image_recieved0,(x,y),(x+w,y+h),(0,0,255),2)
            cv2.putText(image_recieved0,'QC',(x,y-5),cv2.FONT_HERSHEY_COMPLEX,2,(0,1500),1)
            #roi_gray = gray[y:y+h, x:x+w]
            roi_color = image_recieved[y:y+h, x:x+w]
            cv2.imshow('result',image_recieved0)
            # cv2.waitKey(0)
            cv2.destroyAllWindows()
            ans=[classNo[0],x,y,w,h]

    # If wheel identified
    if classNo[0] is 2:
        Wheel = wheel_cascade.detectMultiScale(image_recieved0, 1.3, 5)
        for (x,y,w,h) in Wheel:
            cv2.rectangle(image_recieved0,(x,y),(x+w,y+h),(255,255,0),2)
            cv2.putText(image_recieved0,'Wheel',(x,y-5),cv2.FONT_HERSHEY_COMPLEX,2,(0,1500),1)
            #roi_gray = gray[y:y+h, x:x+w]
            roi_color = image_recieved[y:y+h, x:x+w]
            cv2.imshow('result',image_recieved0)
            # cv2.waitKey(0)
            cv2.destroyAllWindows()
            ans=[classNo[0],x,y,w,h]

    # Noting Found
    else:
        ans = [-1, -1, -1, -1, -1]

    # Return Results
    return ans

# Get Distance to the object
def find_distance(DEPTH_IMG, x, y):

    # Get image data
    try:
        depth_image = bridge.imgmsg_to_cv2(DEPTH_IMG, "32FC1")
    except CvBridgeError, e:
        print e

    #Convert centroid positions to integer
    x = int(x)
    y = int(y)

    #Return Distance
    return depth_image[x][y]


def main():

    ditance_to_rover = 100000
    distance_to_qs = 100000

    # Rotate to Detect Nearby Objects
    for i in range(36):
        # rotate_bot()
        # print("Rotated 10 Degrees")
        # rospy.sleep(1)

        #Get RGB and Depth Data
        RGB = rospy.wait_for_message('/camera/rgb/image_raw', Image)

        #Find Objects if Available
        result = find_obj(RGB)

        if result[0] != -1:
            Depth = rospy.wait_for_message('/camera/depth/image_raw', Image)
            result_distance = find_distance(Depth, result[1] + result[3]/2, result[2] + result[4]/2)

        print(find_distance)

# Pose Estimation
    print(angle_bw,distance_qc,distance_rover)

    x, y =  fsolve(equations, (1, 1))
    theta = math.atan2(qc[0] - x, qc[1] - y)
    theta = theta*-1 + math.pi/2
    y = y*-1
    print("Estimated")
    print(y,x, theta)

    print("Actual")
    print(pose)

# Go to Destination
    print("Given current pose, beginnning to path plan and go to target")
     # Create an action client called "move_base" with action definition file "MoveBaseAction"
    client = actionlib.SimpleActionClient('move_base',MoveBaseAction)
   # Waits until the action server has started up and started listening for goals.
    client.wait_for_server()

   # Creates a new goal with the MoveBaseGoal constructor
    goal = MoveBaseGoal()

    goal.target_pose.header.frame_id = "odom"

    goal.target_pose.pose.position.x = 0
    goal.target_pose.pose.position.y = 0
    goal.target_pose.pose.position.z = 0
    # No rotation of the mobile base frame w.r.t. map frame
    goal.target_pose.pose.orientation.x = 0
    goal.target_pose.pose.orientation.y = 0
    goal.target_pose.pose.orientation.z = -0.683466457574
    goal.target_pose.pose.orientation.w = 0.729981918523

    # Sends the goal to the action server.
    client.send_goal(goal)
    client.wait_for_result()

    print("Reached")

if __name__ == '__main__':
    rospy.init_node('Kidnapped_Robot')
    rospy.Subscriber('/odom', Odometry, odom_callback)
    # rate = rospy.spin()
    main()
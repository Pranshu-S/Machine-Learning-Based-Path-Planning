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
from scipy.optimize import fsolve

# Brings in the SimpleActionClient
import actionlib

# Brings in the .action file and messages used by the move base action
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

# For initialization and storing of Pose
global pose
pose = [0, 0, 0]

# Distance inf
global distance_to_rover
global distance_to_qc

distance_to_rover = 100000
distance_to_qc = 100000


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
    global distance_to_qc
    global distance_to_rover
    x, y = p
    return ((rover[1] - x)**2 + (rover[0] - y)**2 - distance_to_rover**2, (qc[1] - x)**2 + (qc[0] - y)**2 - distance_to_qc**2)

# Callback for odometry readings 
def odom_callback(data):
    global pose
    x = data.pose.pose.orientation.x
    y = data.pose.pose.orientation.y
    z = data.pose.pose.orientation.z
    w = data.pose.pose.orientation.w
    pose = [data.pose.pose.position.x, data.pose.pose.position.y, euler_from_quaternion([x,y,z,w])[2]]

global rgb

# Callback for rbg img
def rgb_callback(data):
    global rgb
    rgb = data

global depth

def depth_callback(data):
    global depth
    depth = data

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
def find_obj():

    global bridge
    global rgb

    # Convert ros_image into an opencv-compatible image
    try:
        cv_image = bridge.imgmsg_to_cv2(rgb, "bgr8")
    except CvBridgeError as e:
        print(e)

    # Convert to Numpy Array
    image_recieved0 = np.array(cv_image)
    image_recieved = np.array(cv_image)

    # If Mars Rover Identified
    MR = MR_cascade.detectMultiScale(image_recieved0, 1.3, 5)
    for (x,y,w,h) in MR:
        cv2.rectangle(image_recieved0,(x,y),(x+w,y+h),(255,0,0),2)
        cv2.putText(image_recieved0,'MR',(x,y-5),cv2.FONT_HERSHEY_COMPLEX,2,(0,1500),1)
        #roi_gray = gray[y:y+h, x:x+w]
        roi_color = image_recieved[y:y+h, x:x+w]

        if w>400:
            cv2.imshow('result',image_recieved0)
            cv2.waitKey(500)
            cv2.destroyAllWindows()

        ans=[3,x,y,w,h]
        # print(ans)
        return ans

    # If QuadCopter identified
    QC = QC_cascade.detectMultiScale(image_recieved0, 1.3, 5)
    for (x,y,w,h) in QC:
        cv2.rectangle(image_recieved0,(x,y),(x+w,y+h),(0,0,255),2)
        cv2.putText(image_recieved0,'QC',(x,y-5),cv2.FONT_HERSHEY_COMPLEX,2,(0,1500),1)
        #roi_gray = gray[y:y+h, x:x+w]
        roi_color = image_recieved[y:y+h, x:x+w]
        if w>400:
            cv2.imshow('result',image_recieved0)
            cv2.waitKey(500)
            cv2.destroyAllWindows()
        ans=[4,x,y,w,h]
        # print(ans)
        return ans

    # Noting Found
    
    ans = [-1, -1, -1, -1, -1]

    # Return Results
    return ans

# Get Distance to the object
def find_distance(x, y):

    global depth

    # Get image data
    try:
        depth_image = bridge.imgmsg_to_cv2(depth, "32FC1")
    except CvBridgeError, e:
        print e

    #Convert centroid positions to integer
    x = int(x)
    y = int(y)

    #Return Distance
    return depth_image[y][x]


def main():


    global distance_to_rover
    global distance_to_qc

    no_of_objects_found = 0

    angle_rotated = 0

    final_angle = 0

    # Rotate to Detect Nearby Objects
    for i in range(27):
        rotate_bot()
        print("Rotated 10 Degrees")
        rospy.sleep(1)

        angle_rotated += 10

        #Find Objects if Available
        result = find_obj()
        print(result)

        if result[3] < 400:
            result[0] = -1

        if result[0] != -1:
            if (result[0] == 1):
                print("Bowl Detected")
            elif result[0] == 2:
                print("Wheel Detected")
            elif result[0] == 3:
                print("Mars Rover Detected")
            else:
                print("Quadcopter Detected")
            result_distance = find_distance(result[1] + result[4]/2, result[2] + result[3]/2)
            # print(result_distance)

            if result[0] == 3 and distance_to_rover > result_distance:
                distance_to_rover = result_distance
                angle_rotated = 0
            
            if result[0] == 4 and distance_to_qc > result_distance:
                distance_to_qc = result_distance
                final_angle = angle_rotated

    distance_to_qc=distance_to_qc * 3
    distance_to_rover=distance_to_rover * 3 

# Pose Estimation
    print(final_angle,distance_to_qc,distance_to_rover)

    x, y =  fsolve(equations, (3, 0))
    y = y*-1
    print(x,y)
    theta = math.atan2(qc[0] - x, qc[1] - y)
    theta = theta*-1 + math.pi/2
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
    rospy.Subscriber('/camera/rgb/image_raw', Image, rgb_callback)

    data = rospy.wait_for_message('/odom', Odometry)

    global pose
    x = data.pose.pose.orientation.x
    y = data.pose.pose.orientation.y
    z = data.pose.pose.orientation.z
    w = data.pose.pose.orientation.w
    pose = [data.pose.pose.position.x, data.pose.pose.position.y, euler_from_quaternion([x,y,z,w])[2]]

    rospy.Subscriber('/odom', Odometry, odom_callback)
    rospy.Subscriber('/camera/depth/image_raw', Image,depth_callback)

    # rate = rospy.spin()
    main()
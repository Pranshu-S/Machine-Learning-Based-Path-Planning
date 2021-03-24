#!/usr/bin/env python
import rospy
import geometry_msgs.msg
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
import math
import numpy as np
import moveit_commander
import moveit_msgs.msg
from tf import TransformListener
from tf.transformations import euler_from_quaternion
import sys
import cv2
import copy
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from std_msgs.msg import Float32MultiArray
# from scipy.optimize import fsolve

bridge = CvBridge()

# Brings in the SimpleActionClient
import actionlib
# Brings in the .action file and messages used by the move base action
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

# For initialization and storing of Pose
global pose
pose = [0, 0, 0]

# DataBase of Objects
list_rover = [79, 81]
list_qc = [80, 82]

rover = [0.297462, 5.661100, 0]
qc = [-2.805357, 4.368702, 0]

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

# Rotate bot to a small extent
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


def get_transform(object_id):

    t = TransformListener()
    object_name = ('object_' + '%d') % object_id

    rospy.loginfo(t.frameExists("base_link"))
    rospy.loginfo(object_name)
    rospy.loginfo(t.frameExists(object_name))

    rospy.sleep(1)

    if t.frameExists(object_name):
        print("HERE")
        (translation,rotation) = t.lookupTransform("base_link", object_name, rospy.Time())
        print(translation)
        return translation
    else:
        pass

    # print(translation)


def main():
    global distance_qc
    global distance_rover
    #Conditional and logical Parameters
    objects_found = 0
    location_qc = 0
    location_rover = 0
    angle_bw = 0

    distance_qc = 0
    distance_rover = 0

    first_obj = 'none'


    # Rotate to Detect Nearby Objects
    for i in range(36):
        rotate_bot()
        print("Rotated 10 Degrees")
        rospy.sleep(1)

        data = rospy.wait_for_message('/objects', Float32MultiArray)
        data.data = list(data.data)

        #Initializing Empty lists for Storing Detected objects
        object_rover = []
        object_qc = []

        #Checking for detected objects
        i = 0
        while i < len(data.data):
            if data.data[i] in list_qc: object_qc.append(data.data[i])
            elif data.data[i] in list_rover: object_rover.append(data.data[i])
            i = i + 12

        # QuadCopter Detected
        if len(object_qc) != 0:
            print("Detected QuadCopter!")
            location_qc = get_transform(object_qc[0])
            print(type(location_qc))
            print(location_qc)
            if location_qc:
                if location_qc[1] > -0.4 and location_qc[1] < 0.4:
                    distance_qc = location_qc[0]
                    if objects_found == 0:
                        first_obj = 'qc'
                        objects_found = 1
                    if first_obj == 'rover' and objects_found == 1:
                        objects_found = 2
                        break
        
        #Mars Rover Detected
        elif len(object_rover) != 0:
            print("Detected Rover!")
            location_rover = get_transform(object_rover[0])
            print(location_rover)
            print(type(location_rover))
            if location_rover:
                if location_rover[1] > -0.4 and location_rover[1] < 0.4:
                    distance_rover = location_rover[0]
                    if objects_found == 0:
                        first_obj = 'rover'
                        objects_found = 1
                    if first_obj == 'qc' and objects_found == 1:
                        objects_found = 2
                        break

        print(objects_found)
        if objects_found == 1:
            angle_bw = angle_bw + 10

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
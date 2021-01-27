#!/usr/bin/env python
# license removed for brevity

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
import math

from tf.transformations import euler_from_quaternion

# Brings in the SimpleActionClient
import actionlib
# Brings in the .action file and messages used by the move base action
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

global pose
pose = [0, 0, 0]

def odom_callback(data):
    global pose
    x  = data.pose.pose.orientation.x
    y  = data.pose.pose.orientation.y
    z = data.pose.pose.orientation.z
    w = data.pose.pose.orientation.w
    pose = [data.pose.pose.position.x, data.pose.pose.position.y, euler_from_quaternion([x,y,z,w])[2]]


def movebase_client():

   # Create an action client called "move_base" with action definition file "MoveBaseAction"
    client = actionlib.SimpleActionClient('move_base',MoveBaseAction)
   # Waits until the action server has started up and started listening for goals.
    client.wait_for_server()

   # Creates a new goal with the MoveBaseGoal constructor
    goal = MoveBaseGoal()
   # Move 0.5 meters forward along the x axis of the "map" coordinate frame 

    goal_x = [-9.2, 10.7, 12.6, 18.2, -2]
    goal_y = [-1.2, 10.5, -1.9, -1.4, 4.0]

    goal.target_pose.header.frame_id = "map"


    #------------Goal 1-----------------
    goal.target_pose.pose.position.x = goal_x[0]
    goal.target_pose.pose.position.y = goal_y[0]
    # No rotation of the mobile base frame w.r.t. map frame

    target_theta = math.atan2((goal_y[1] - goal_y[0]),(goal_x[1] - goal_x[0]))
    goal.target_pose.pose.orientation.w = target_theta


    inf = 10000
    error_x = inf
    error_y = inf


    # Sends the goal to the action server.
    client.send_goal(goal)
    x = 1

    while error_x > 0.2 or error_y>0.2:
        error_x = abs(goal_x[0] - pose[0])
        error_y = abs(goal_y[0] - pose[1])

    # while x: 
    #     if(error_x < 0.1 and error_y<0.1):
    #         rospy.loginfo("Oreintation = %s" % abs(pose[2]-target_theta))
    #         if(abs(pose[2]-target_theta)<0.5):
    #             x = 0
    #             break
    #     error_x = abs(goal_x[0] - pose[1])
    #     error_y = abs(goal_y[0] - pose[1])
        # rospy.loginfo("Oreintation = %s" % abs(pose[2]-target_theta))
    
    #------------Goal 2--------------------
    goal.target_pose.pose.position.x = goal_x[1]
    goal.target_pose.pose.position.y = goal_y[1]
    # No rotation of the mobile base frame w.r.t. map frame
    target_theta = math.atan2((goal_y[2] - goal_y[1]),(goal_x[2] - goal_x[1]))
    goal.target_pose.pose.orientation.w = target_theta


    inf = 10000
    error_x = inf
    error_y = inf


    # Sends the goal to the action server.
    client.send_goal(goal)

    # while x: 
    #     if(error_x < 0.1 and error_y<0.1):
    #         if(abs(pose[2]-target_theta)<0.5):
    #             break
    #     error_x = abs(goal_x[1] - pose[2])
    #     error_y = abs(goal_y[1] - pose[2])

    while error_x > 0.2 or error_y>0.2:
        error_x = abs(goal_x[1] - pose[0])
        error_y = abs(goal_y[1] - pose[1])

    #------------Goal 3-----------------
    goal.target_pose.pose.position.x = goal_x[2]
    goal.target_pose.pose.position.y = goal_y[2]
    # No rotation of the mobile base frame w.r.t. map frame
    target_theta = math.atan2((goal_y[3] - goal_y[2]),(goal_x[3] - goal_x[2]))
    goal.target_pose.pose.orientation.w = target_theta


    inf = 10000
    error_x = inf
    error_y = inf


    # Sends the goal to the action server.
    client.send_goal(goal)

    while error_x > 0.4 or error_y>0.4:
        error_x = abs(goal_x[2] - pose[0])
        error_y = abs(goal_y[2] - pose[1])

    #------------Goal 4--------------------
    goal.target_pose.pose.position.x = goal_x[3]
    goal.target_pose.pose.position.y = goal_y[3]
    # No rotation of the mobile base frame w.r.t. map frame
    goal.target_pose.pose.orientation.w =1


    inf = 10000
    error_x = inf
    error_y = inf


    # Sends the goal to the action server.
    client.send_goal(goal)

    # while x: 
    #     if(error_x < 0.1 and error_y<0.1):
    #         if(abs(pose[2]-target_theta)<0.5):
    #             break
    #     error_x = abs(goal_x[3] - pose[0])
    #     error_y = abs(goal_y[3] - pose[1])

    while error_x > 0.2 or error_y>0.2:
        error_x = abs(goal_x[3] - pose[0])
        error_y = abs(goal_y[3] - pose[1])

    #------------Goal 5--------------------
    goal.target_pose.pose.position.x = goal_x[4]
    goal.target_pose.pose.position.y = goal_y[4]
    # No rotation of the mobile base frame w.r.t. map frame
    
    goal.target_pose.pose.orientation.w =1


    inf = 10000
    error_x = inf
    error_y = inf


    # Sends the goal to the action server.
    client.send_goal(goal)

    while error_x > 0.2 or error_y>0.2:

        error_x = abs(goal_x[4] - pose[0])
        error_y = abs(goal_y[4] - pose[1])


# If the python node is executed as main process (sourced directly)
if __name__ == '__main__':
    try:
       # Initializes a rospy node to let the SimpleActionClient publish and subscribe
        rospy.init_node('movebase_client_py')
        rospy.Subscriber('/odom', Odometry, odom_callback)
        movebase_client()
    except rospy.ROSInterruptException:
        rospy.loginfo("Navigation test finished.")

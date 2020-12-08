#! /usr/bin/env python
import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist, Point, Pose, PoseStamped
from nav_msgs.msg import Odometry
from gazebo_msgs.msg import LinkState
from tf import transformations
import math
import random
import actionlib
import actionlib.msg
import exp_assignment2.msg
import rospy
import time
#from __future__ import print_function


def human_client():
    # Creates the SimpleActionClient, passing the type of the action
    # (FibonacciAction) to the constructor.
    client = actionlib.SimpleActionClient(
        '/reaching_goal', exp_assignment2.msg.PlanningAction)

    # Waits until the action server has started up and started
    # listening for goals.
    client.wait_for_server()

    # Creates a goal to send to the action server.
    goal = exp_assignment2.msg.PlanningGoal()
    goal.target_pose.pose.position.x = 1  # random.randrange(0, 9)
    goal.target_pose.pose.position.y = 1  # random.randrange(0, 9)
    goal.target_pose.pose.position.z = 0  # -1

    # Sends the goal to the action server.
    client.send_goal(goal)

    # Waits for the server to finish performing the action.
    client.wait_for_result()

    return client.get_result()


def main():

    rospy.init_node('human_client')

    while True:
        human_client()
        time.sleep(10)


if __name__ == '__main__':
    main()

#!/usr/bin/env python
import sys

import rospy
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
import math
import time
from std_srvs.srv import Empty
# Brings in the SimpleActionClient
import actionlib

# Brings in the messages used by the fibonacci action, including the
# goal message and the result message.
import search_and_rescue.msg

def main():

    rospy.init_node('search_and_rescue', anonymous=True)
    client = actionlib.SimpleActionClient('search_and_rescue', search_and_rescue.msg.target_statusAction)
    # Waits until the action server has started up and started
    # listening for goals.
    client.wait_for_server()
    # Creates a goal to send to the action server.
    goal = search_and_rescue.msg.target_statusGoal(x_coordinates=4, y_coordinates=3)

    # Sends the goal to the action server.
    client.send_goal(goal)
    # Waits for the server to finish performing the action.
    client.wait_for_result()

    # Prints out the result of executing the action
    print(client.get_result())  # A FibonacciResult


main()
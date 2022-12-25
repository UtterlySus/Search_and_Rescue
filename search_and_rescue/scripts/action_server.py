#! /usr/bin/env python

import rospy
import actionlib
from search_and_rescue.msg import target_statusAction, target_statusFeedback, target_statusResult
import os


class ActionServer():

    def __init__(self):
        self.a_server = actionlib.SimpleActionServer(
            "search_and_rescue", target_statusAction, execute_cb=self.execute_cb, auto_start=False)
        self.a_server.start()

    def execute_cb(self, goal):

        success = True
        target_status = 'Target Found'
        feedback = target_statusFeedback()
        result = target_statusResult()
        rate = rospy.Rate(1)

        feedback.target_status = target_status
        

        result.x_coordinates = goal.x_coordinates
        result.y_coordinates = goal.y_coordinates

        self.a_server.publish_feedback(feedback)
        rate.sleep()

        if success:
            self.a_server.set_succeeded(result)
            #run UGV mission
            os.system('python /home/utterlysus/catkin_ws/src/search_and_rescue/src/ugv_mission.py '+str(result.x_coordinates)+' '+str(result.y_coordinates))




if __name__ == "__main__":
    rospy.init_node("search_and_rescue")
    s = ActionServer()
    rospy.spin()

#!/usr/bin/env python

import rospy 
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import Point, Twist 
import math
import time
from math import atan2 
from sensor_msgs.msg import LaserScan # LaserScan type message is defined in sensor_msgs
from sensor_msgs.msg import Image
# ROS Image message -> OpenCV2 image converter
from cv_bridge import CvBridge, CvBridgeError
# OpenCV2 for saving an image
import cv2

import sys


# Instantiate CvBridge
bridge = CvBridge()

x = 0.0
y = 0.0
theta = 0.0

def showImage(title, img):
    cv2.imshow(title, img)
    cv2.waitKey(1)


def newOdom (msg):
    global x
    global y
    global theta 
    x = msg.pose.pose.position.x
    y = msg.pose.pose.position.y
    
    rot_q = msg.pose.pose.orientation 
    (roll, pitch, theta) = euler_from_quaternion ([rot_q.x, rot_q.y, rot_q.z, rot_q.w])

def laserScan (msg):
    global laser_scanner
    laser_scanner=msg
    
def UGV_rescu_callback(msg):

    try:

        feeded_img_ugv = bridge.imgmsg_to_cv2(msg, "bgr8")
        #print(feeded_img_ugv.shape)
        showImage("UGV Feed", feeded_img_ugv)

    except Exception as err:
            
        #print("error pass 1 ", err)
        pass    


# def Lasercallback(data):

#     print(data.ranges[0])

#     threshold = 0.8 # Laser scan range threshold
#     if data.ranges[0]<threshold: # Checks if there are obstacles in front and

#         avoid.linear.x = 0 # go forward (linear velocity)
#         avoid.angular.z = 1 # do not rotate (angular velocity)


#     else:
#         avoid.linear.x = 0.0 # stop
#         avoid.angular.z = 0.5 # rotate counter-clockwise
#         if dt.ranges[0]>thr1 and dt.ranges[15]>thr2 and dt.ranges[345]>thr2:
#             avoid.linear.x = 0.5
#             avoid.angular.z = 0.0
#     velocity_publisher.publish(avoid) # publish the move object


def go_to_goal(velocity_publisher, x_goal, y_goal):
    global x
    global y, theta

    velocity_message = Twist()

    thr1 = 1 # Laser scan range threshold



    while (True):
        # print(laser_scanner.ranges[0])
        if laser_scanner.ranges[0]<thr1 :
            print("obstecle ahead detected")
            velocity_message.linear.x = 0 #stop
            velocity_message.angular.z = 2 # do not rotate (angular velocity
            velocity_publisher.publish(velocity_message)
            rospy.sleep(1)
            velocity_message.angular.z = 0 # do not rotate (angular velocity
            velocity_publisher.publish(velocity_message)
            if laser_scanner.ranges[0]>thr1:
                velocity_message.linear.x = 1 #move
            else:
                velocity_message.angular.z = 2 # do not rotate (angular velocity
                velocity_publisher.publish(velocity_message)
                rospy.sleep(1) 
                velocity_message.angular.z = 0 # do not rotate (angular velocity
                velocity_publisher.publish(velocity_message)
                continue
        

        K_linear = 100
        distance = abs(math.sqrt(((x_goal-x) ** 2) + ((y_goal-y) ** 2)))

        linear_speed = distance * K_linear

        K_angular = 2.0
        desired_angle_goal = math.atan2(y_goal-y, x_goal-x)
        angular_speed = (desired_angle_goal-theta)*K_angular

        velocity_message.linear.x = linear_speed
        velocity_message.angular.z = angular_speed

        velocity_publisher.publish(velocity_message)
        rospy.sleep(0.5)
        #print ('x=', x, ', y=',y, ', distance to goal: ', distance)

        if (distance <0.1):
            break



def stopUGV():
    vel_msg = Twist()
    vel_msg.linear.x = 0
    vel_msg.linear.y = 0
    vel_msg.linear.z = 0
    vel_msg.angular.z = 0
    vel_msg.angular.x = float(0.0)
    vel_msg.angular.y = float(0.0)
    velocity_publisher.publish(vel_msg)



if __name__ == '__main__':

    x_coord = int(sys.argv[1])
    y_coord = int(sys.argv[2])


    try:

        rospy.init_node('UGV_Rescue', anonymous=True)

        avoid = Twist() # Creates a Twist message type object

        #declare velocity publisher
        
        velocity_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        UGV_cam = rospy.Subscriber('/camera/image_raw', Image, UGV_rescu_callback)
        
        
        pose_subscriber = rospy.Subscriber('/odometry/filtered', Odometry, newOdom)

        sub_laser = rospy.Subscriber("/scan", LaserScan, laserScan)  # Subscriber object which will listen

        time.sleep(2)

        
        go_to_goal(velocity_publisher, x_coord, y_coord)
        #go_to_goal(velocity_publisher, -15, 0)
        stopUGV()
        rospy.spin()

        
    except rospy.ROSInterruptException:
        rospy.loginfo("node terminated.")
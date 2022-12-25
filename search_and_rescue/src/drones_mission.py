#!/usr/bin/env python3
#  -*- coding: utf-8 -*-
from std_msgs.msg import Empty
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Odometry
import tkinter as tk
import math
import rospy
import numpy as np
from datetime import datetime
import os

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


from sensor_msgs.msg import Image
# ROS Image message -> OpenCV2 image converter
from cv_bridge import CvBridge, CvBridgeError
# OpenCV2 for saving an image
import cv2



def quaterionToRads(data):
	x = data.pose.orientation.x
	y = data.pose.orientation.y
	z = data.pose.orientation.z
	w = data.pose.orientation.w

	t3 = 2.0 * (w * z + x * y)
	t4 = 1.0 - 2.0 * (y * y + z * z)
	yawZActual = math.atan2(t3, t4)
	if yawZActual < 0:
		yawZActual = 2*math.pi + yawZActual

	return yawZActual

def pose_callback_uav1(data):
    global x_p_u1
    global y_p_u1
    global z_p_u1

    x_p_u1 = data.pose.pose.position.x
    y_p_u1 = data.pose.pose.position.y
    z_p_u1 = data.pose.pose.position.z

def rot_callback_uav1(data):
    global z_o_u1

    z_o_u1 = math.degrees(quaterionToRads(data))

def pose_callback_uav2(data):
    global x_p_u2
    global y_p_u2
    global z_p_u2

    x_p_u2 = data.pose.pose.position.x
    y_p_u2 = data.pose.pose.position.y
    z_p_u2 = data.pose.pose.position.z

def rot_callback_uav2(data):
    
    global z_o_u2

    rospy.sleep(1)

    z_o_u2 = math.degrees(quaterionToRads(data))

# Instantiate CvBridge
bridge = CvBridge()
bridge2 = CvBridge()

def showImage(title, img):
    cv2.imshow(title, img)
    cv2.waitKey(1)



def uav1_image_callback(msg):
    
    try:

        car_cascade = cv2.CascadeClassifier('/home/utterlysus/catkin_ws/src/search_and_rescue/src/cars.xml')

        feeded_img = bridge.imgmsg_to_cv2(msg, "bgr8")

        # Altering properties of image with cv2  
        imaging_gray = cv2.cvtColor(feeded_img, cv2.COLOR_BGR2GRAY)  

        cars = car_cascade.detectMultiScale(imaging_gray, 1.1, 5)

        # Don't do anything if there's 
        # no sign
        amount_found = len(cars)
            
            
        if amount_found != 0:
            global target_detected
            global search_started
                

            for (x, y, w, h) in cars:
                    

                cv2.rectangle(feeded_img,(x,y),(x+w,y+h),(0,0,255),2)
            if search_started == True:

                target_detected = True
                
                
                # Using cv2.imwrite() method
                # Saving the image
                try:
                    # now = datetime.now()
                    # print(now)
                    # directory = r'/home/utterlysus/Downloads'
                    # os.chdir(directory)
                    # Filename
                    # dt_string = now.strftime("%d/%m/%Y %H:%M:%S")
                    # print(dt_string)
                    
                    filename = '/home/utterlysus/catkin_ws/src/search_and_rescue/images/object_detected.jpg'
                    cv2.imwrite(filename, feeded_img)
                    showImage("UAV1 Target Found", feeded_img)
                    distance_of_object()
                    global drone_to_target_x_coordinates
                    global drone_to_target_y_coordinates

                    drone_to_target_x_coordinates = x_p_u1
                    drone_to_target_y_coordinates = y_p_u1

                except Exception as err:
                   # print("error pass 2")
                    pass
                #distance_of_object(feeded_img)
               
        showImage("UAV1 Feed", feeded_img)

    except Exception as err:
        
        #print("error pass 1 ", err)
        pass


def uav2_image_callback(msg):

    counter = 0

    if(counter < 1):
        rospy.sleep(3)
        counter =counter +1

    try:


        feeded_img_uav2 = bridge2.imgmsg_to_cv2(msg, "bgr8")
        showImage("UAV2 Feed", feeded_img_uav2)

    except Exception as err:
        
        #print("error pass 1 ", err)
        pass   

# def uav2_image_callback(msg):

#     print("Received an image!")

#     feeded_img = bridge.imgmsg_to_cv2(msg, "bgr8")
#     drawImg = feeded_img




rospy.init_node('HectorQ_GUI_Lider', anonymous=False)
#Subscribers
posicionLider_sub_uav1 = rospy.Subscriber("/uav1/ground_truth/state", Odometry , pose_callback_uav1)
orientaLider_sub_uav1 = rospy.Subscriber("/uav1/ground_truth_to_tf/pose", PoseStamped , rot_callback_uav1)

posicionLider_sub_uav2 = rospy.Subscriber("/uav2/ground_truth/state", Odometry , pose_callback_uav2)
orientaLider_sub_uav2 = rospy.Subscriber("/uav2/ground_truth_to_tf/pose", PoseStamped , rot_callback_uav2)



cam_sub_uav1 = rospy.Subscriber("/uav1/front_cam/camera/image", Image , uav1_image_callback)

cam_sub_uav2 = rospy.Subscriber("/uav2/front_cam/camera/image", Image , uav2_image_callback)



#Publishers

vel_pub_uav1 = rospy.Publisher('/uav1/cmd_vel', Twist, queue_size=1)
vel_pub_uav2 = rospy.Publisher('/uav2/cmd_vel', Twist, queue_size=1)



def stop(vx,vy,vz,vaz, uav):
    vel_msg = Twist()
    vel_msg.linear.x = float(vx)
    vel_msg.linear.y = float(vy)
    vel_msg.linear.z = float(vz)
    vel_msg.angular.z = float(vaz)
    vel_msg.angular.x = float(0.0)
    vel_msg.angular.y = float(0.0)
    uav.publish(vel_msg)

def hover_pub(uav):
    stop(0.0,0.0,0.0,0.0, uav)


def up_fun(uav):
    vel_msg = Twist()
    vel_msg.linear.z = float(1.0)
    uav.publish(vel_msg)

def down_fun(uav):
    vel_msg = Twist()
    vel_msg.linear.z = float(-1.0)
    uav.publish(vel_msg)

def forward_fun(uav):
    vel_msg = Twist()
    vel_msg.linear.x = float(1.0)
    uav.publish(vel_msg)

def backward_fun(uav):
    vel_msg = Twist()
    vel_msg.linear.x = float(-1.0)
    uav.publish(vel_msg)

def right_fun(uav):
    vel_msg = Twist()
    vel_msg.linear.y = float(-1.0)
    uav.publish(vel_msg)

def left_fun(uav):
    vel_msg = Twist()
    vel_msg.linear.y = float(1.0)
    uav.publish(vel_msg)

def back_right_with_a_tilt(uav):
    vel_msg = Twist()
    vel_msg.linear.x = float(-0.25)
    vel_msg.linear.y = float(-1.0)

    vel_msg.linear.z = float(-0.25)
    uav.publish(vel_msg)

def UAV1adjust_front_parallel_to_base():
    vel_msg = Twist()

    while z_o_u1 > 1:
        vel_msg.angular.z = float(0.25)
        vel_pub_uav1.publish(vel_msg)
        rospy.sleep(0.5)

def UAV2adjust_front_parallel_to_base():
    vel_msg = Twist()

    while z_o_u2 > 1:
        print("adjusting uav2 "+str(z_o_u2))
        vel_msg.angular.z = float(-0.25)
        vel_pub_uav2.publish(vel_msg)
        rospy.sleep(0.5)
        
# def cw_fun():
#     vel_msg = Twist()
#     vel_msg.angular.z = float(-1.0)
#     vel_pub.publish(vel_msg)

# def ccw_fun():
#     vel_msg = Twist()
#     vel_msg.angular.z = float(1.0)
#     vel_pub.publish(vel_msg)

# def spiralSwipeUAV1():
#     vel_msg = Twist()
#     loop_rate = rospy.Rate(1)

#     wk = 1
#     rk = 0
    
#     while True:
#         rk=rk+0.5
#         vel_msg.linear.x =rk
#         vel_msg.linear.y =0
#         vel_msg.linear.z =0
#         vel_msg.angular.x = 0
#         vel_msg.angular.y = 0
#         vel_msg.angular.z =wk
#         vel_pub_uav1.publish(vel_msg)
#         loop_rate.sleep()


 
#     vel_msg.linear.x = 0
#     vel_msg.angular.z = 0
#     vel_pub_uav1.publish(vel_msg)

def find_marker(image):
	# convert the image to grayscale, blur it, and detect edges
	gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
	gray = cv2.GaussianBlur(gray, (5, 5), 0)
	edged = cv2.Canny(gray, 35, 125)

	# find the contours in the edged image and keep the largest one;
	# we'll assume that this is our piece of paper in the image
	(cnts, _) = cv2.findContours(edged.copy(), cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
	c = max(cnts, key = cv2.contourArea)

	# compute the bounding box of the of the paper region and return it
	return cv2.minAreaRect(c)

def distance_to_camera(knownWidth, focalLength, perWidth):
	# compute and return the distance from the maker to the camera
	return (knownWidth * focalLength) / perWidth

def distance_of_object():

    # initialize the known distance from the camera to the object, which
    # in this case is 24 inches
    KNOWN_DISTANCE = 50

    # initialize the known object width, which in this case, the piece of
    # paper is 12 inches wide
    KNOWN_WIDTH = 5.0

    # initialize the list of images that we'll be using
    IMAGE_PATH = "/home/utterlysus/catkin_ws/src/search_and_rescue/images/object_detected.jpg"

    # load the furst image that contains an object that is KNOWN TO BE 2 feet
    # from our camera, then find the paper marker in the image, and initialize
    # the focal length
    image = cv2.imread(IMAGE_PATH)
    marker = find_marker(image)
    focalLength = (marker[1][0] * KNOWN_DISTANCE) / KNOWN_WIDTH

	# load the image, find the marker in the image, then compute the
	# distance to the marker from the camera
	#image = cv2.imread(imagePath)
	#marker = find_marker(image)
    inches = distance_to_camera(KNOWN_WIDTH, focalLength, marker[1][0])
    global meters
    meters = inches*0.3048 #convert to meters

	# draw a bounding box around the image and display it
    box = np.int0(cv2.boxPoints(marker))
    cv2.drawContours(image, [box], -1, (0, 255, 0), 2)
    cv2.putText(image, "%.2fmt" % meters,
		(image.shape[1] - 400, image.shape[0] - 20), cv2.FONT_HERSHEY_SIMPLEX,
		2.0, (0, 255, 0), 3)
    filename = '/home/utterlysus/catkin_ws/src/search_and_rescue/images/calculated_distance.jpg'
    cv2.imwrite(filename, image)
    showImage("UAV1 Processed image", image)

def go_to_goal(velocity_publisher, x_goal, y_goal):
    global x_p_u1
    global y_p_u1
    global z_p_u1

    velocity_message = Twist()

    while (True):
        K_linear = 0.5 
        distance = abs(math.sqrt(((x_goal-x_p_u1) ** 2) + ((y_goal-y_p_u1) ** 2)))

        linear_speed = distance * K_linear

        K_angular = 1
        desired_angle_goal = math.atan2(y_goal-y_p_u1, x_goal-x_p_u1)
        angular_speed = (desired_angle_goal-z_p_u1)*K_angular

        velocity_message.linear.x = linear_speed
        velocity_message.angular.z = angular_speed

        velocity_publisher.publish(velocity_message)
        #print ('x=', x_p_u1, ', y=',y_p_u1, ', distance to goal: ', distance)

        if (distance <0.01):
            break

def UAV1ReturntoBase():
    global target_detected
    global search_started

    target_detected = False
    search_started = False

    while y_p_u1 < -1:
        # print("uav1 y-axis ",y_p_u1)
        left_fun(vel_pub_uav1)
        rospy.sleep(0.5)
    
    if (x_p_u1 > 1):

        while x_p_u1 > 1:

            # print("uav1 if x-axis ",x_p_u1)
            backward_fun(vel_pub_uav1)
            rospy.sleep(0.5)
    else:
        while x_p_u1 < -1:
            # print("uav1 else x-axis ",x_p_u1)
            forward_fun(vel_pub_uav1)
            rospy.sleep(0.5)

    while z_p_u1 > 1:
        down_fun(vel_pub_uav1)
        rospy.sleep(0.5)

def UAV2ReturntoBase():


    while y_p_u2 > 1:
        # print("uav2 y-axis ",y_p_u2)
        left_fun(vel_pub_uav2)
        rospy.sleep(0.5)
    
    if (x_p_u2 > 1):

        while x_p_u2 > 1:

            # print("uav2 if x-axis ",x_p_u2)
            backward_fun(vel_pub_uav2)
            rospy.sleep(0.5)
    else:
        while x_p_u2 < -1:
            # print("uav2 else x-axis ",x_p_u2)
            forward_fun(vel_pub_uav2)
            rospy.sleep(0.5)

    while z_p_u2 > 1:
        down_fun(vel_pub_uav2)
        rospy.sleep(0.5)

def spiralSwipeUAVs():

    global target_detected
    global search_started

    target_detected = False
    search_started = True

    vel_msg = Twist()
    #loop_rate = rospy.Rate(1)


    wk = 1
    rk = 0.5

    while target_detected != True:

        
        rk=rk+0.5
        vel_msg.linear.x =rk
        vel_msg.linear.y =0
        vel_msg.linear.z =0
        vel_msg.angular.x = 0
        vel_msg.angular.y = 0
        vel_msg.angular.z =wk
        vel_pub_uav1.publish(vel_msg)
        vel_pub_uav2.publish(vel_msg)
        #loop_rate.sleep()
        rospy.sleep(0.5)

        if(y_p_u2 > 25 or y_p_u2 < 0):
            break
        if(x_p_u2 > 20 or x_p_u2 < -20):
            break

        if(y_p_u1 < -25 or y_p_u1 > 0):
            break
        if(x_p_u1 > 20 or x_p_u1 < -20):
            break
 
    vel_msg.linear.x = 0
    vel_msg.angular.z = 0

    vel_pub_uav1.publish(vel_msg)
    vel_pub_uav2.publish(vel_msg)

def main():

    
    while True:

        up_fun(vel_pub_uav1)
        up_fun(vel_pub_uav2)

        rospy.sleep(0.5)
        #print(z_o_u1)
        if(z_p_u1 > 25 and z_p_u2 > 25):
            break


        
    hover_pub(vel_pub_uav1)
    hover_pub(vel_pub_uav2)

    
    while True:
        right_fun(vel_pub_uav1)
        left_fun(vel_pub_uav2)
        rospy.sleep(0.5)

        if (y_p_u1 < -12.5 and y_p_u2 > 12.5):
            break
    

    hover_pub(vel_pub_uav1)
    hover_pub(vel_pub_uav2)


    spiralSwipeUAVs()

    hover_pub(vel_pub_uav1)
    hover_pub(vel_pub_uav2)
    rospy.sleep(3)

    # inform the action server

    client = actionlib.SimpleActionClient('search_and_rescue', search_and_rescue.msg.target_statusAction)
    # Waits until the action server has started up and started
    # listening for goals.
    client.wait_for_server()
    # Creates a goal to send to the action server.
    global meters
    # print(meters)
    goal = search_and_rescue.msg.target_statusGoal(x_coordinates=-int(drone_to_target_x_coordinates), y_coordinates=int(drone_to_target_y_coordinates - meters*2))
    # print(int(drone_to_target_x_coordinates))
    # print(int(drone_to_target_y_coordinates + meters))
    # Sends the goal to the action server.
    client.send_goal(goal)
    # Waits for the server to finish performing the action.
    client.wait_for_result()

    # Prints out the result of executing the action
    print(client.get_result())  


    UAV1adjust_front_parallel_to_base()
    hover_pub(vel_pub_uav1)

    UAV1ReturntoBase()
    hover_pub(vel_pub_uav1)

    UAV2adjust_front_parallel_to_base()
    hover_pub(vel_pub_uav2)

    UAV2ReturntoBase()
    hover_pub(vel_pub_uav2)


    rospy.spin()

    # go_to_goal(vel_pub_uav1, 0, 1)

    # rospy.sleep(10)

    # hover_pub(vel_pub_uav1)



main()


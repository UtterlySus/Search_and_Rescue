# Search_and_Rescue

----------------------
Welcome to our project!
----------------------

This work is an outcome of the course project of COE 510: Programming Methods for Robotics at KFUPM as part of master program in Robotics and Autonomous Intelligent Systems.


Firstly, please refer the IEEE paper to gain the understanding and knowledge behind the project and the different details of the final results.

then, follow the below steps to execute the simulation correctly:

1- unzip the compressed file in https://files.fm/u/8qwbk26ss

1.1- copy all the folders inside ROS_Files to cd ~/catkin_ws/src

2- now go back to root folder by "cd ~/catkin_ws/" in terminal

3- compile and build the packeges "catkin_make" in terminal

if you see 100% completed then everything is good!

4- now we need to source catkin_ws by typing "source ~/catkin_ws/devel/setup.bash" in terminal

5- go to line 96, 133, 340 and 363 in drones_mission file under search_and_rescue/src and adjust the absolute path for your device

6- do the same in line 36 of search_and_rescue/scripts/actions_server.py

7- open a new terminal and execute "roslaunch hector_quadrotor_demo two_drones_desert.launch"

the simulation will start after this, thanks!



# MROBOT 
SLAM using 2D lidar 

Usage

1. Matlab:

	Step1:setenv('ROS_MASTER_URI','http://UBUNTU-IP:11311');
	
	      setenv('ROS_IP','PC-IP');
	      
	Step2:rosinit

2. Ubuntu16.04(kinetic):

	Step1:roscore
	
	Step2:roslauch mrobot_laser_nav_gazebo.launch
	
	note:you need set ROS_MASTER_URI & ROS_HOSTNAME  both of MATLAB and UBUNTU

3. ROS_CONTROL_APP

![MATLAB_ROS_APP](https://github.com/lei01cao/MROBOT/blob/master/img/matlab-ros.png)

To Do

1. Mapping

	![MAPPING]()
2. AMCL_Localize

	![AMCL]()
3. Path_follow

	![PRM]()
4. ObstacleAvoidance

	![VFH]()

References 

1.ROS机器人开发实践 胡春旭 

2.Robotic System Toolbox Matlab2018b

![MATLAB_ROS](https://github.com/lei01cao/MROBOT/blob/master/img/matlab-ros.png)

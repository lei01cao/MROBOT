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

![MATLAB_ROS_APP](https://github.com/lei01cao/MROBOT/blob/master/img/MATLAB_ROS_APP.png)

To Do

1. Mapping

	![MAPPING](https://github.com/lei01cao/MROBOT/blob/master/img/mapping.png)
2. AMCL_Localize

	![AMCL](https://github.com/lei01cao/MROBOT/blob/master/img/AMCL.png)
3. Path_follow

	<center class="half">
    <img src="https://github.com/lei01cao/MROBOT/blob/master/img/prm-500.png" title="PRM-500" height="350" width="400">
    <img src="https://github.com/lei01cao/MROBOT/blob/master/img/prm-5000.png" title="PRM-5000" height="350" width="400">
	</center>
	<!-- ![PRM](https://github.com/lei01cao/MROBOT/blob/master/img/prm.png) -->
4. ObstacleAvoidance

	![VFH](https://github.com/lei01cao/MROBOT/blob/master/img/vfh.png)

References 

1.ROS WIKI 

2.Robotic System Toolbox Matlab2018b

![MATLAB_ROS](https://github.com/lei01cao/MROBOT/blob/master/img/matlab-ros.png)

clc; clear all;
%rosinit
% path = [0  0;
%         3  0; 
%         4 -3;
%         -4 -4];
% path = [0  0;
%         3  0;
%         5  -2;
%         5  -5;
%         -5 -5;
%         -5 0;
%         0 -1];  

% findpath
startLocation = [0 0]; 
endLocation = [5 -5];
prmSimple = robotics.PRM(myOccMap,50)
path = findpath(prm, startLocation, endLocation);
global laserSub; 
global odomSub;
global amcl;

%reset gazebo
client = rossvcclient('/gazebo/reset_world');
call(client);
%���빹����ɵ�ȫ��դ���ͼ
load map\map_mrobot_gazebo_laser_nav.mat;

%�������⴫����ģ�ͺͲ��ٻ������˶�ģ��
odometryModel = robotics.OdometryMotionModel;
odometryModel.Noise = [0.2 0.2 0.2 0.2];
rangeFinderModel = robotics.LikelihoodFieldSensorModel;
rangeFinderModel.SensorLimits = [0.45 8];
rangeFinderModel.Map = myOccMap;

% Query the Transformation Tree (tf tree) in ROS.
tftree = rostf;
% waitForTransform(tftree,'/base_link','/camera_depth_frame');
% sensorTransform = getTransform(tftree,'/base_link', '/camera_depth_frame');
waitForTransform(tftree,'/base_link','/laser_link');
sensorTransform = getTransform(tftree,'/base_link', '/laser_link');

% Get the euler rotation angles.
laserQuat = [sensorTransform.Transform.Rotation.W sensorTransform.Transform.Rotation.X ...
    sensorTransform.Transform.Rotation.Y sensorTransform.Transform.Rotation.Z];
laserRotation = quat2eul(laserQuat, 'ZYX');

% Setup the |SensorPose|, which includes the translation along base_link's
% +X, +Y direction in meters and rotation angle along base_link's +Z axis
% in radians.
rangeFinderModel.SensorPose = ...
    [sensorTransform.Transform.Translation.X sensorTransform.Transform.Translation.Y laserRotation(1)];

%���Ĵ���������
%laserSub = rossubscriber('/scan');
odomSub = rossubscriber('/odom');

%ObstacleAvoidance
vfh = robotics.VectorFieldHistogram;
vfh.DistanceLimits = [0.05 1];
vfh.RobotRadius = 0.1;
vfh.MinTurningRadius = 0.2;
vfh.SafetyDistance = 0.1;

targetDir = 0; %Ŀ�귽������Ϊ0

%��ʼ��AMCL����
amcl = robotics.MonteCarloLocalization;
amcl.UseLidarScan = true;
amcl.MotionModel = odometryModel;
amcl.SensorModel = rangeFinderModel;
%amcl.UpdateThresholds = [0.2,0.2,0.2]; %������ֵ
amcl.UpdateThresholds = [0.2,0.2,0.2];  %������ֵ
%amcl.ResamplingInterval = 1;           %�ظ��������
amcl.ResamplingInterval = 1;           %�ظ��������

%ʹ�ó�ʼ��̬Ԥ������AMCL���ж�λ
%amcl.ParticleLimits = [500 5000];%�����������޺�����
amcl.ParticleLimits = [50 100];
amcl.GlobalLocalization = false; %ȫ�ֶ�λʹ��
amcl.InitialPose = TruePose;%��������ʵ����
%amcl.InitialPose = [0 0 0];%��������ʵ����
amcl.InitialCovariance = eye(3)*0.5;

robotCurrentLocation = path(1,:); %�����˳�ʼλ��
robotGoal = path(end,:);		%������Ŀ��λ��
initialOrientation = 0;			%��ʼ��̬��
robotCurrentPose = [robotCurrentLocation initialOrientation];
laserSub = rossubscriber('/scan');  
%[velPub, velMsg] = rospublisher('/cmd_vel');
robot = rospublisher('cmd_vel','geometry_msgs/Twist');
velMsg = rosmessage(robot);
controller = robotics.PurePursuit;
controller.Waypoints = path;
%controller.DesiredLinearVelocity = 0.3;
controller.DesiredLinearVelocity = 0.3;
%controller.MaxAngularVelocity = 2;
controller.MaxAngularVelocity = 0.3;
% controller.LookaheadDistance = 0.5;
controller.LookaheadDistance = 0.5;
% Define a goal radius, which is the desired distance threshold between the robot's final location and the goal location. Once the robot is within this distance from the goal, it will stop. Also, you compute the current distance between the robot location and the goal location. This distance is continuously checked against the goal radius and the robot stops when this distance is less than the goal radius.
% Note that too small value of the goal radius may cause the robot to miss the goal, which may result in an unexpected behavior near the goal.
goalRadius = 0.1; 
%goalRadius = 0.1; 
distanceToGoal = norm(robotCurrentLocation - robotGoal);
%controlRate = robotics.Rate(10);
controlRate = robotics.Rate(20);
while(distanceToGoal > goalRadius)   
	[v, omega] = controller(robotCurrentPose);
    disp([v,omega]);

    % Get laser scan data
    laserScan = receive(laserSub);
    ranges = double(laserScan.Ranges);
    angles = double(laserScan.readScanAngles)   

    % Call VFH object to computer steering direction
    steerDir = vfh(ranges, angles, targetDir);    
        % Calculate velocities
    if ~isnan(steerDir) % If steering direction is valid
        %desiredV = 0.2;
        desiredV = v;
        %w = ComputeAngularVelocity(steerDir, 1);
        w = omega;
    else % Stop and search for valid direction
        desiredV = 0.0;
        w = 0.5;
    end

    % Assign and send velocity commands
    velMsg.Linear.X = desiredV;
    velMsg.Angular.Z = w;
    velPub.send(velMsg);

	%drive(robot, v, omega);
    %velMsg.Linear.X = v;
    %velMsg.Angular.Z = omega;
    %send(robot,velMsg);
    %rospublisher('cmd_vel', 'IsLatching', true);   
	%robotCurrentPose = robot.getRobotPose;
    robotCurrentPose = TruePose();%��λʵʱλ��
    disp(robotCurrentPose);
	distanceToGoal = norm(robotCurrentPose(1:2) - robotGoal);
	waitfor(controlRate);
end
velMsg.Linear.X = 0;
velMsg.Angular.Z = 0;
send(robot,velMsg);
disp(robotCurrentPose);

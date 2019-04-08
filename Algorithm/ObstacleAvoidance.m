%VFH避障算法
laserSub = rossubscriber('/scan');
[velPub, velMsg] = rospublisher('/mobile_base/commands/velocity');

%初始化VFH对象
vfh = robotics.VectorFieldHistogram;
vfh.DistanceLimits = [0.05 1];
vfh.RobotRadius = 0.1;
vfh.MinTurningRadius = 0.2;
vfh.SafetyDistance = 0.1;

targetDir = 0; %目标方向设置为0

%设置速率对象
rate = robotics.Rate(10);

%创建循环，收集数据，计算转向方向并驱动机器人
while rate.TotalElapsedTime < 30

	% Get laser scan data
	laserScan = receive(laserSub);
	ranges = double(laserScan.Ranges);
	angles = double(laserScan.readScanAngles);
	
	% Call VFH object to computer steering direction
	steerDir = vfh(ranges, angles, targetDir);  
    
	% Calculate velocities
	if ~isnan(steerDir) % If steering direction is valid
		desiredV = 0.2;
		w = exampleHelperComputeAngularVelocity(steerDir, 1);
	else % Stop and search for valid direction
		desiredV = 0.0;
		w = 0.5;
	end

	% Assign and send velocity commands
	velMsg.Linear.X = desiredV;
	velMsg.Angular.Z = w;
	velPub.send(velMsg);
end

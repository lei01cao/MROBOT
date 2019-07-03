function estimatedPose = G_Localize()
%AMCL_Localize(LM_observing,Match_Global(k,:),ldf{k,1});
%   输入：
%        NULL        
%   输出：
%       estimatedPose    机器人预估位置  
global laserSub;
global odomSub;
global amcl;

%Localization Procedure
numUpdates = 60;
i = 0;
    while i < numUpdates
    % Receive laser scan and odometry message.
    scanMsg = receive(laserSub);
    odompose = odomSub.LatestMessage;
    
    % Create lidarScan object to pass to the AMCL object.
    scan = lidarScan(scanMsg);
    
    % For sensors that are mounted upside down, you need to reverse the
    % order of scan angle readings using 'flip' function.
    
    % Compute robot's pose [x,y,yaw] from odometry message.
    odomQuat = [odompose.Pose.Pose.Orientation.W, odompose.Pose.Pose.Orientation.X, ...
        odompose.Pose.Pose.Orientation.Y, odompose.Pose.Pose.Orientation.Z];
    odomRotation = quat2eul(odomQuat);
    pose = [odompose.Pose.Pose.Position.X, odompose.Pose.Pose.Position.Y odomRotation(1)];
    
    % Update estimated robot's pose and covariance using new odometry and
    % sensor readings.
    [isUpdated,estimatedPose, estimatedCovariance] = amcl(pose, scan);
    
    % Plot the robot's estimated pose, particles and laser scans on the map.
    if isUpdated
        i = i + 1;
    end
    %[isUpdated,estimatedPose, estimatedCovariance] = amcl(pose, scan);
end
    
    
    
    

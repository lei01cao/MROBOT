function groundTruthPose = TruePose()
% This function subscribes to gazebo/model_states and find the
% 'mobile_base' pose data. 

% Obtain 'mobile_base' pose from gazebo.
posesub = rossubscriber('gazebo/model_states');
gazeboModelsPoses = receive(posesub);
for i = 1:size(gazeboModelsPoses.Pose)
    if strcmp(gazeboModelsPoses.Name(i), 'mobile_base')
        break
    end
end
% Compute [x,y,yaw] from Gazebo pose data
quat = [gazeboModelsPoses.Pose(i).Orientation.W, gazeboModelsPoses.Pose(i).Orientation.X, ...
    gazeboModelsPoses.Pose(i).Orientation.Y, gazeboModelsPoses.Pose(i).Orientation.Z];
rot = quat2eul(quat);
x = gazeboModelsPoses.Pose(i).Position.X;
y = gazeboModelsPoses.Pose(i).Position.Y;
yaw = rot(1);
groundTruthPose = [x,y,yaw];
end


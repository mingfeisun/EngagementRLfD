posePub = rospublisher('/pepper_dcm/Head_controller/command', 'trajectory_msgs/JointTrajectory');
timeSub = rossubscriber('/clock');

% load data
X = 2*sin(1:0.1:2*pi);
Y = cos(1:0.1:2*pi) + 1;

T = size(X, 2);

poseMsg = rosmessage(posePub);
poseMsg.JointNames = {'HeadYaw'};

while 1  
    point = rosmessage('trajectory_msgs/JointTrajectoryPoint');
    point.Accelerations = ones(1, length(X));
    point.Velocities = ones(1, length(X));
    point.Effort = ones(1, length(X));
    point.Positions = X;
    
    tempTime = receive(timeSub, 2);
    
    poseMsg.Header.Stamp.Sec = tempTime.Clock_.Sec+1;
    poseMsg.Header.Stamp.Nsec = tempTime.Clock_.Nsec;
    
    point.TimeFromStart.Sec = tempTime.Clock_.Sec + 5;
    point.TimeFromStart.Nsec = tempTime.Clock_.Nsec;
    
    poseMsg.Points = point;
    send(posePub, poseMsg);
    pause(10);
end

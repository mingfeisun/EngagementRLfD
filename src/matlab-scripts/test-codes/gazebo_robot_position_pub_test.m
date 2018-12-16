headYawPub = rospublisher('/pepper_dcm/LShoulderPitch_position_controller/command', 'std_msgs/Float64');
timeSub = rossubscriber('/clock');

% load data
X = sin(1:0.1:2*pi);
Y = cos(1:0.1:2*pi) + 1;

T = size(X, 2); 

ind = 1;

while 1  
    point = rosmessage('std_msgs/Float64');
    point.Data = X(ind);
    send(headYawPub, point);
    pause(0.5);
    ind = ind+1;
    if ind > length(X)
        ind = 1;
    end
end

test = RobotControl();

pt = rosmessage('geometry_msgs/Point');
pt.X = 0;
pt.Y = 0;
pt.Z = 0;

upperLimit = 100;
i = 0;

while i <= upperLimit
    % test.LookAt(pt);
    jointPos = {...
        [0, 0, -1]; ...
        [0, 0, 0]; ...
        [0, 1, 0]; ...
        [0.4*sin(0.1*i)+0.5, 0.4*cos(0.1*i)+1, 0.4*sin(0.1*i)+0.5]; ...
        [0.6*sin(0.1*i)+0.7, 0.6*cos(0.1*i)+1.2, 0.6*sin(0.1*i)+0.7]...
    };
    test.FollowThis(jointPos, 'left');
    i = i+1;
    if i > upperLimit
        i = 0;
    end
    pause(1/5);
end
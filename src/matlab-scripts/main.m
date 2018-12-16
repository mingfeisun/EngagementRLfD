clear;

robot = RobotControlTf();
robot.Init();

while 1
    % robot.AttentionEngage();
    robot.MimicEngage();
end
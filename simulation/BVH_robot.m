tfTree = rostf;

actorFrame = {'actor_LeftShoulder', 'actor_LeftArm', 'actor_RightShoulder', 'actor_RightArm'};
actorFrame_c = {'actor_LeftArm', 'actor_LeftForeArm', 'actor_RightArm', 'actor_RightForeArm'};
msgType = 'std_msgs/Float64';

lshoulderRoll = '/pepper_dcm/LShoulderRoll_position_controller/command';
lshoulderPitch = '/pepper_dcm/LShoulderPitch_position_controller/command';
lelbowYaw = '/pepper_dcm/LElbowYaw_position_controller/command';
lelbowRoll = '/pepper_dcm/LElbowRoll_position_controller/command';
lwristYaw = '/pepper_dcm/LWristYaw_position_controller/command';

rshoulderRoll = '/pepper_dcm/RShoulderRoll_position_controller/command';
rshoulderPitch = '/pepper_dcm/RShoulderPitch_position_controller/command';
relbowYaw = '/pepper_dcm/RElbowYaw_position_controller/command';
relbowRoll = '/pepper_dcm/RElbowRoll_position_controller/command';
rwristYaw = '/pepper_dcm/RWristYaw_position_controller/command';

LSRPub = rospublisher(lshoulderRoll, msgType);
LSPPub = rospublisher(lshoulderPitch, msgType);
LEYPub = rospublisher(lelbowYaw, msgType);
LERPub = rospublisher(lelbowRoll, msgType);
LWYPub = rospublisher(lwristYaw, msgType);

RSRPub = rospublisher(rshoulderRoll, msgType);
RSPPub = rospublisher(rshoulderPitch, msgType);
REYPub = rospublisher(relbowYaw, msgType);
RERPub = rospublisher(relbowRoll, msgType);
RWYPub = rospublisher(rwristYaw, msgType);

while 1    
    waitForTransform(tfTree, actorFrame_c{1}, actorFrame{1});
    tform_new = getTransform(tfTree, actorFrame_c{1}, actorFrame{1});
    tform_rot = tform_new.Transform.Rotation;
    eul = quat2eul([tform_rot.W, tform_rot.X, tform_rot.Y, tform_rot.Z], 'ZYX');
    
%     eul_degree = eul*180/pi;
%     display(eul_degree);
        
    msg = rosmessage('std_msgs/Float64');
    msg.Data = 0.2 + eul(1);
    send(LSPPub, msg);
    
    msg = rosmessage('std_msgs/Float64');
    msg.Data = eul(2) - 1.5;
    send(LEYPub, msg);
    
    msg = rosmessage('std_msgs/Float64');
    msg.Data = eul(3) + 1;
    send(LSRPub, msg);

    waitForTransform(tfTree, actorFrame_c{2}, actorFrame{2});
    tform_new = getTransform(tfTree, actorFrame_c{2}, actorFrame{2});
    tform_rot = tform_new.Transform.Rotation;
    eul = quat2eul([tform_rot.W, tform_rot.X, tform_rot.Y, tform_rot.Z], 'ZYX');
    
%     eul_degree = eul*180/pi;
%     display(eul_degree);

    msg = rosmessage('std_msgs/Float64');
    msg.Data = -eul(2) - 0.5;
    send(LWYPub, msg);
        
    msg = rosmessage('std_msgs/Float64');
    msg.Data = eul(3);
    send(LERPub, msg);
    
    waitForTransform(tfTree, actorFrame_c{3}, actorFrame{3});
    tform_new = getTransform(tfTree, actorFrame_c{3}, actorFrame{3});
    tform_rot = tform_new.Transform.Rotation;
    eul = quat2eul([tform_rot.W, tform_rot.X, tform_rot.Y, tform_rot.Z], 'ZYX');
    
%     eul_degree = eul*180/pi;
%     display(eul_degree);
        
    msg = rosmessage('std_msgs/Float64');
    msg.Data = -eul(1) - 0.2;
    send(RSPPub, msg);
    
    msg = rosmessage('std_msgs/Float64');
    msg.Data = eul(2) + 1.7;
    send(REYPub, msg);
    
    msg = rosmessage('std_msgs/Float64');
    msg.Data = eul(3) + 1;
    send(RSRPub, msg);

    waitForTransform(tfTree, actorFrame_c{4}, actorFrame{4});
    tform_new = getTransform(tfTree, actorFrame_c{4}, actorFrame{4});
    tform_rot = tform_new.Transform.Rotation;
    eul = quat2eul([tform_rot.W, tform_rot.X, tform_rot.Y, tform_rot.Z], 'ZYX');
    
%     eul_degree = eul*180/pi;
%     display(eul_degree);
        
    msg = rosmessage('std_msgs/Float64');
    msg.Data = -eul(2) - 0.5;
    send(RWYPub, msg);
    
    msg = rosmessage('std_msgs/Float64');
    msg.Data = -eul(3);
    send(RERPub, msg);
end

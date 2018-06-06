clear;
rosinit;

h=axes;
PFilter = 0;
linkPose = GazeboLinkPose();
linkPose.Init();
robot = RobotControlTf();
attention = AttentionMap();
mimic = ApproximateMimic();

while 1
    [pepperPose, actorPose, flag] = linkPose.Update();
    if ~flag
        pause(0.01);
        continue;
    end
    
    % lookPt = linkPose.getJointPt('actor::RightHand');
    % robot.LookAt(lookPt);
    
    [posePred, PFilter, dominanceLevel] = attention.StartTracking(actorPose);
    
    attention.generateAttentionMap(actorPose);
    attention.showAttentionMap();

    [headYaw, headPitch] = attention.getAttentionPoint();
    % robot.TurnHead(headYaw, headPitch);
    
%     cla;
%     skel_vis(actorPose, 1, PFilter, h, 'true');
%     skel_vis(posePred, 1, PFilter, h, 'pred');

    keyJointsPos = linkPose.getKeyJoints();
    angleConfigs = mimic.generateConfigs(keyJointsPos);
    robot.BehaveLike(angleConfigs);
    
end
rosshutdown;
    
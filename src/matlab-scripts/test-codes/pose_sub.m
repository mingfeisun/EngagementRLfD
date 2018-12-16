poseSub = rossubscriber('/node_skeleton/skeleton');

bodyPose = zeros(1, 80);
PFilter = 0;
h=axes;

while 1
    skeleton = receive(poseSub, 10);
    for joint=0:19
        bodyPose(1, joint*4+1) = skeleton.Position(joint+1).X;
        bodyPose(1, joint*4+2) = skeleton.Position(joint+1).Y;
        bodyPose(1, joint*4+3) = skeleton.Position(joint+1).Z;
        bodyPose(1, joint*4+4) = 1;
    end
    
    cla;
    skel_vis(bodyPose, 1, PFilter, h, 'true');
    drawnow;
end
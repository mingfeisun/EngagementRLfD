% poseSub = rossubscriber('/node_skeleton/skeleton');

bag = rosbag('test-data/temp.bag');
msgs = readMessages(bag);
bagLength = length(msgs);

bodyPose = zeros(bagLength, 80);
PFilter = 0;
h=axes;

for i=1:bagLength
    skeleton = msgs{i, 1};
    for joint=0:19
        bodyPose(i, joint*4+1) = skeleton.Position(joint+1).X;
        bodyPose(i, joint*4+2) = skeleton.Position(joint+1).Y;
        bodyPose(i, joint*4+3) = skeleton.Position(joint+1).Z;
        bodyPose(i, joint*4+4) = 1;
    end
    
    cla;
    skel_vis(bodyPose, i, PFilter, h, 'true');
    drawnow;
end
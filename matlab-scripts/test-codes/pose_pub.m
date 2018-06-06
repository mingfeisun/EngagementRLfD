posePub = rospublisher('/node_skeleton/skeleton', 'emotion_intensity/Skeleton');

% load data
filename = 'P3_2_8_p29';
[X, Y, tagset] = load_file(filename);
T = size(X, 1);

ind = 1;
poseMsg = rosmessage(posePub);

while ind <= T    
    skeleton = X(ind, :);
    poseMsg.Position =[];
    for i=0:19
        joint = rosmessage('geometry_msgs/Vector3');
        joint.X = skeleton(i*4+1);
        joint.Y = skeleton(i*4+2);
        joint.Z = skeleton(i*4+3);
        poseMsg.Position = [poseMsg.Position; joint];
    end
    send(posePub, poseMsg);
    % pause(1/30);
    ind = ind + 1;
    disp(ind);
end

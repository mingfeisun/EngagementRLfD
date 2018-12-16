clear all;
Untitled
% load data
filename = 'P1_1_1A_p19';
[X, Y, tagset] = load_file(filename);
T = size(X, 1);

Xpred = X(:, :);

simulationIndex = 1;
h=axes;

testJointTracking = JointTracking();

tic;

while simulationIndex <= T
    currPose = X(simulationIndex, :);
%     lastTime = toc;UntitledUntitled
    [Xpred(simulationIndex, :), PFilter] = testJointTracking.Start(currPose);
%     display(toc-lastTime);
    cla;
%     lastTime = toc;
	skel_vis(X, simulationIndex, PFilter, h, 'true');
    % skel_vis(Xpred, simulationIndex, PFilter, h, 'pred');
	drawnow;Untitled
%     display(toc-lastTime);UntitledUntitled
	% pause(1/30);
    
    simulationIndex = simulationIndex + 1;
end

clear all;

poseSub = rossubscriber('/node_skeleton/skeleton', 'emotion_intensity/Skeleton');

skel_model
bodyPose = ones(1, 80);
bodyPosePred = ones(1, 80);

upperBodyJoints = [...
    NUI_SKELETON_POSITION_HEAD,...
    NUI_SKELETON_POSITION_SHOULDER_CENTER,...
    NUI_SKELETON_POSITION_SHOULDER_LEFT,...
    NUI_SKELETON_POSITION_SHOULDER_RIGHT,...
    NUI_SKELETON_POSITION_ELBOW_LEFT,...
    NUI_SKELETON_POSITION_ELBOW_RIGHT,...
    NUI_SKELETON_POSITION_WRIST_LEFT,...
    NUI_SKELETON_POSITION_WRIST_RIGHT,...
    NUI_SKELETON_POSITION_HAND_LEFT,...
    NUI_SKELETON_POSITION_HAND_RIGHT,...
    NUI_SKELETON_POSITION_SPINE,...
    NUI_SKELETON_POSITION_HIP_CENTER];

trackJointNum = length(upperBodyJoints);

PFilter = [];
for i=1:trackJointNum
    PFilter = [PFilter; robotics.ParticleFilter];
end

% initialize 
rng('default');
particleNum = 100;
for i=1:trackJointNum
    PFilter(i).StateEstimationMethod = 'mean';
    PFilter(i).ResamplingMethod = 'systematic';
    initialize(PFilter(i), particleNum, zeros(1, 3), eye(3));
    PFilter(i).StateTransitionFcn = @stateTransition;
    PFilter(i).MeasurementLikelihoodFcn = @measurementLikelihood;
end
lastMeasurement1 = zeros(trackJointNum, 3);
lastMeasurement2 = zeros(trackJointNum, 3);

h=axes;
noiseVar = 0.03*ones(trackJointNum, 1);

while 1
    skeleton = receive(poseSub, 10);
    
    for i=1:20
        measurement = [skeleton.Position(i).X, skeleton.Position(i).Y, skeleton.Position(i).Z];
        bodyPose(4*i-3:4*i-1) = measurement(:);
        
        if i <= trackJointNum
            uCmd = lastMeasurement1(i, :) - lastMeasurement2(i, :);
            lastMeasurement2(i, :) = lastMeasurement1(i, :);
            lastMeasurement1(i, :) = measurement(:);

            [statePred, covPred] = predict(PFilter(i), uCmd, noiseVar(i));
            [stateCorrected, covCorrected] = correct(PFilter(i), measurement);

            bodyPosePred(4*i-3:4*i-1) = stateCorrected(:); 

            % adjust the particle distribution 
            measurementError = bsxfun(@minus, PFilter(i).Particles, measurement);
            meanError = mean(sum(abs(measurementError), 2));
            if meanError <= 0.2 && noiseVar(i) >= 0.001
                noiseVar(i) = noiseVar(i)/2;
            end
            if meanError >= 0.1 && noiseVar(i) < 0.005
                noiseVar(i) = noiseVar(i)*3;
            end

            val = mvnpdf(measurement, stateCorrected, covCorrected);
        else
            bodyPosePred(4*i-3:4*i-1) = measurement(:); 
        end
    end
    
    cla;
	skel_vis(bodyPose, 1, PFilter, h, 'true');
    skel_vis(bodyPosePred, 1, PFilter, h, 'pred');
	drawnow;
	pause(1/30);
end

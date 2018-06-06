clear all;

% load data
filename = 'P1_1_1_p28';
[X, Y, tagset] = load_file(filename);
T = size(X, 1);
skel_model
Xpred = X(:, :);

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

simulationIndex = 1;
h=axes;
noiseVar = 0.03*ones(trackJointNum, 1);

while simulationIndex <= T
    for i=1:trackJointNum
        measurement = X(simulationIndex, upperBodyJoints(i)*4+1:upperBodyJoints(i)*4+3);
    
        uCmd = lastMeasurement1(i, :) - lastMeasurement2(i, :);
        lastMeasurement2(i, :) = lastMeasurement1(i, :);
        lastMeasurement1(i, :) = measurement(:);

        [statePred, covPred] = predict(PFilter(i), uCmd, noiseVar(i));
        [stateCorrected, covCorrected] = correct(PFilter(i), measurement);

        Xpred(simulationIndex, upperBodyJoints(i)*4+1:upperBodyJoints(i)*4+3) = stateCorrected; 

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
    end
    
    cla;
	skel_vis(X, simulationIndex, PFilter, h, 'true');
    skel_vis(Xpred, simulationIndex, PFilter, h, 'pred');
	drawnow;
	% pause(1/30);
    
    simulationIndex = simulationIndex + 1;
end

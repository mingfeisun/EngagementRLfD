clear all;

% load data
filename = 'P3_2_8_p29';
[X, Y, tagset] = load_file(filename);
T = size(X, 1);
skel_model
Xpred = X(:, :);

% upperBodyJoints = [...
%     NUI_SKELETON_POSITION_HEAD,...
%     NUI_SKELETON_POSITION_SHOULDER_CENTER,...
%     NUI_SKELETON_POSITION_SHOULDER_LEFT,...
%     NUI_SKELETON_POSITION_SHOULDER_RIGHT,...
%     NUI_SKELETON_POSITION_ELBOW_LEFT,...
%     NUI_SKELETON_POSITION_ELBOW_RIGHT,...
%     NUI_SKELETON_POSITION_WRIST_LEFT,...
%     NUI_SKELETON_POSITION_WRIST_RIGHT,...
%     NUI_SKELETON_POSITION_HAND_LEFT,...
%     NUI_SKELETON_POSITION_HAND_RIGHT,...
%     NUI_SKELETON_POSITION_SPINE,...
%     NUI_SKELETON_POSITION_HIP_CENTER]';

bodyJoint = NUI_SKELETON_POSITION_HEAD;

PFilter = robotics.ParticleFilter;

% initialize 
rng('default');
particleNum = 100;

PFilter.StateEstimationMethod = 'mean';
PFilter.ResamplingMethod = 'systematic';
initialize(PFilter, particleNum, zeros(1, 3), eye(3));
PFilter.StateTransitionFcn = @stateTransition;
PFilter.MeasurementLikelihoodFcn = @measurementLikelihood;

lastMeasurement1 = zeros(1, 3);
lastMeasurement2 = zeros(1, 3);

simulationIndex = 1;
h=axes;
noiseVar = 0.03;

while simulationIndex <= T

    measurement = Xpred(simulationIndex, bodyJoint*4+1:bodyJoint*4+3);

    uCmd = lastMeasurement1 - lastMeasurement2;
    lastMeasurement2 = lastMeasurement1;
    lastMeasurement1 = measurement;

    [statePred, covPred] = predict(PFilter, uCmd, noiseVar);
    [stateCorrected, covCorrected] = correct(PFilter, measurement);

    Xpred(simulationIndex, bodyJoint*4+1:bodyJoint*4+3) = stateCorrected; 

    % adjust the particle distribution 
    measurementError = bsxfun(@minus, PFilter.Particles, measurement);
    meanError = mean(sum(abs(measurementError), 2));
    if meanError <= 0.2 && noiseVar >= 0.001
        noiseVar = noiseVar/2;
    end
    if meanError >= 0.1 && noiseVar < 0.005
        noiseVar = noiseVar*3;
    end

    val = mvnpdf(measurement, stateCorrected, covCorrected);

    cla;
	skel_vis(X, simulationIndex, PFilter, h, 'true');
    skel_vis(Xpred, simulationIndex, PFilter, h, 'pred');
	drawnow;
	
    simulationIndex = simulationIndex + 1;
end
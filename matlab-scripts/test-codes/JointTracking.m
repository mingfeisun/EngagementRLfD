classdef JointTracking < handle
    properties
        upperBodyJoints;
        PFilter;
        trackJointNum;
        lastMeasurement1;
        lastMeasurement2;
        noiseVar;
    end
    
    methods
        function obj = JointTracking()
            skel_model;
%             obj.upperBodyJoints = [...
%                 NUI_SKELETON_POSITION_HEAD,...
%                 NUI_SKELETON_POSITION_SHOULDER_CENTER,...
%                 NUI_SKELETON_POSITION_SHOULDER_LEFT,...
%                 NUI_SKELETON_POSITION_SHOULDER_RIGHT,...
%                 NUI_SKELETON_POSITION_ELBOW_LEFT,...
%                 NUI_SKELETON_POSITION_ELBOW_RIGHT,...
%                 NUI_SKELETON_POSITION_WRIST_LEFT,...
%                 NUI_SKELETON_POSITION_WRIST_RIGHT,...
%                 NUI_SKELETON_POSITION_HAND_LEFT,...
%                 NUI_SKELETON_POSITION_HAND_RIGHT,...
%                 NUI_SKELETON_POSITION_SPINE,...
%                 NUI_SKELETON_POSITION_HIP_CENTER...
%             ];

            obj.upperBodyJoints = [...
                NUI_SKELETON_POSITION_HEAD,...
                NUI_SKELETON_POSITION_SHOULDER_CENTER,...
                NUI_SKELETON_POSITION_SHOULDER_LEFT,...
                NUI_SKELETON_POSITION_SHOULDER_RIGHT,...
                NUI_SKELETON_POSITION_ELBOW_LEFT,...
                NUI_SKELETON_POSITION_ELBOW_RIGHT,...
                NUI_SKELETON_POSITION_HAND_LEFT,...
                NUI_SKELETON_POSITION_HAND_RIGHT...
            ];
        
%             obj.upperBodyJoints = [NUI_SKELETON_POSITION_HAND_LEFT];
        
            obj.trackJointNum = length(obj.upperBodyJoints);
            obj.PFilter = [];
            for i=1:obj.trackJointNum
                obj.PFilter = [obj.PFilter; robotics.ParticleFilter];
            end
            rng('default');
            particleNum = 100;
            for i=1:obj.trackJointNum
                obj.PFilter(i).StateEstimationMethod = 'mean';
                obj.PFilter(i).ResamplingMethod = 'systematic';
                initialize(obj.PFilter(i), particleNum, zeros(1, 3), eye(3));
                obj.PFilter(i).StateTransitionFcn = @obj.stateTransition;
                obj.PFilter(i).MeasurementLikelihoodFcn = @obj.measurementLikelihood;
            end
            obj.lastMeasurement1 = zeros(obj.trackJointNum, 3);
            obj.lastMeasurement2 = zeros(obj.trackJointNum, 3);
            obj.noiseVar = 0.03*ones(obj.trackJointNum, 1);
        end
        
        function [predPose, updatedParticles, dominanceLevel] = Start(obj, currPose)
            predPose = ones(size(currPose));
            for i=1:20
                measurement(:) = currPose(4*i-3:4*i-1);
                if ismember(i-1, obj.upperBodyJoints)
                    i_shift = find(obj.upperBodyJoints == (i-1) );
                    uCmd = obj.lastMeasurement1(i_shift, :) - obj.lastMeasurement2(i_shift, :);
                    obj.lastMeasurement2(i_shift, :) = obj.lastMeasurement1(i_shift, :);
                    obj.lastMeasurement1(i_shift, :) = measurement(:);

                    [statePred, covPred] = predict(obj.PFilter(i_shift), uCmd, obj.noiseVar(i_shift));
                    [stateCorrected, covCorrected] = correct(obj.PFilter(i_shift), measurement);

                    % adjust the particle distribution
                    % TODO: how to adjust the particle distribution, 
                    % ball --> ellipsoid
                    
                    measurementError = bsxfun(@minus, obj.PFilter(i_shift).Particles, measurement);
                    meanError = mean(sum(abs(measurementError), 2));
                    if meanError <= 0.2 && obj.noiseVar(i_shift) >= 0.001
                        obj.noiseVar(i_shift) = obj.noiseVar(i_shift)/2;
                    end
                    if meanError >= 0.1 && obj.noiseVar(i_shift) < 0.005
                        obj.noiseVar(i_shift) = obj.noiseVar(i_shift)*3;
                    end
                    predPose(4*i-3:4*i-1) = stateCorrected(:); 
                    % val = mvnpdf(measurement, stateCorrected, covCorrected);
                else
                    predPose(4*i-3:4*i-1) = measurement(:); 
                end
            end
            updatedParticles = obj.PFilter;
            dominanceLevel = obj.noiseVar;
        end
        function predictParticles = stateTransition(obj, pf, prevParticles, uCmd, noiseLevel)
            stateTransitionModel = eye(pf.NumStateVariables);
            processNoise = eye(pf.NumStateVariables);
            dist = matlabshared.tracking.internal.NormalDistribution(pf.NumStateVariables);
            dist.Mean = zeros(1, pf.NumStateVariables);
            dist.Covariance = processNoise;
            noise = noiseLevel * dist.sample(pf.NumParticles);
            deltaPosition = ones(pf.NumParticles, 1) * uCmd;
            predictParticles = prevParticles * stateTransitionModel + deltaPosition + noise;
        end
        
        function likelihood = measurementLikelihood(obj, pf, predictParticles, measurement)
            predictMeasurement = predictParticles; 
            measurementError = bsxfun(@minus, predictMeasurement, measurement);
            measurementErrorNorm = sum(abs(measurementError), 2);
            measurementNoise = eye(pf.NumStateVariables);
            likelihood = 1/sqrt((2*pi).^3 * det(measurementNoise)) * exp(-2 * measurementErrorNorm);
        end
    end
end


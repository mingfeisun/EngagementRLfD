classdef AttentionMap < handle
    properties
        % Markov decision process
        state;
        reward;
        transMatrix;
        discount;
        
        % CSpace
        headConfigs;
        jointContour;
        
        % attention map
        headYawRes;
        headPitchRes;
        attentionMap;
        
        % fix
        fixScale;
        headYaw_min;
        headYaw_max;
        headPitch_min;
        headPitch_max;
        
        % head configs limit
        headYaw_ceil;
        headYaw_floor;
        headPitch_ceil;
        headPitch_floor;
        
        % tf
        tfTree;
        
        % pf
        upperBodyJoints;
        PFilter;
        particleNum;
        trackJointNum;
        lastMeasurement1;
        lastMeasurement2;
        noiseVar;
        noiseVar_prev;
        
    end
    
    methods
        function obj = AttentionMap()
            obj.discount = 0.9;
            obj.headYawRes = 0.01;
            obj.headPitchRes = 0.01;
            
            % head yaw: -2.08567 -- 2.08567
            % head pitch: -0.706858 -- 0.637045
            obj.headYaw_ceil = 2.08567;
            obj.headYaw_floor = -2.08567;
            obj.headPitch_ceil = 0.637045;
            obj.headPitch_floor = -0.637045;
            
            obj.fixScale = 100;
            obj.headYaw_max = -inf;
            obj.headYaw_min = inf;
            obj.headPitch_max = -inf;
            obj.headPitch_min = inf;
            
            obj.tfTree = rostf;
            
            obj.initPF();
        end
        
        function Run(obj, currPose)
            % TODO: warp up
        end
        
        function initPF(obj)
            skel_model;
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
       
            obj.trackJointNum = length(obj.upperBodyJoints);
            obj.PFilter = [];
            for i=1:obj.trackJointNum
                obj.PFilter = [obj.PFilter; robotics.ParticleFilter];
            end
            rng('default');
            obj.particleNum = 200;
            for i=1:obj.trackJointNum
                obj.PFilter(i).StateEstimationMethod = 'mean';
                obj.PFilter(i).ResamplingMethod = 'systematic';
                initialize(obj.PFilter(i), obj.particleNum, zeros(1, 3), eye(3));
                obj.PFilter(i).StateTransitionFcn = @obj.stateTransition;
                obj.PFilter(i).MeasurementLikelihoodFcn = @obj.measurementLikelihood;
            end
            obj.lastMeasurement1 = zeros(obj.trackJointNum, 3);
            obj.lastMeasurement2 = zeros(obj.trackJointNum, 3);
            obj.noiseVar = 0.03*ones(obj.trackJointNum, 1);
            obj.noiseVar_prev = 0.03*ones(obj.trackJointNum, 1);
        end
        
        function [predPose, updatedParticles, dominanceLevel] = StartTracking(obj, currPose)
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

                    obj.adjustDistribution(measurement, i_shift);
                    predPose(4*i-3:4*i-1) = stateCorrected(:);
                else
                    predPose(4*i-3:4*i-1) = measurement(:); 
                end
            end
            updatedParticles = obj.PFilter;
            dominanceLevel = obj.noiseVar;
        end
        
        function adjustDistribution(obj, measurement, i_shift)
            % FIXME: adjust the particle distribution, ball --> ellipsoid
            measurementError = bsxfun(@minus, obj.PFilter(i_shift).Particles, measurement);
            meanError = mean(sum(abs(measurementError), 2));
            
            obj.noiseVar_prev(i_shift) = obj.noiseVar(i_shift);
            obj.noiseVar(i_shift) = meanError/50;
            
%             if meanError <= 0.2 && obj.noiseVar(i_shift) >= 0.001
%                 obj.noiseVar(i_shift) = obj.noiseVar(i_shift)/2;
%             end
%             if meanError >= 0.1 && obj.noiseVar(i_shift) < 0.005
%                 obj.noiseVar(i_shift) = obj.noiseVar(i_shift)*3;
%             end
        end
        
        function predictParticles = stateTransition(obj, pf, prevParticles, uCmd, noiseLevel)
            stateTransitionModel = eye(pf.NumStateVariables);
            
            % FIXME: update the covariance matrix
            % if norm(uCmd) ~= 0
            %     uCmd_norm = uCmd/norm(uCmd);
            % else
            %     uCmd_norm = uCmd;
            % end
            
            processNoise = noiseLevel * eye(pf.NumStateVariables);
            % processNoise = eye(pf.NumStateVariables);
            dist = matlabshared.tracking.internal.NormalDistribution(pf.NumStateVariables);
            dist.Mean = zeros(1, pf.NumStateVariables);
            dist.Covariance = processNoise;
            noise = dist.sample(pf.NumParticles);
            
            deltaPosition = ones(pf.NumParticles, 1) * uCmd;
            
            predictParticles = prevParticles * stateTransitionModel + deltaPosition + 0.1*noise;
        end
        
        function likelihood = measurementLikelihood(obj, pf, predictParticles, measurement)
            predictMeasurement = predictParticles; 
            measurementError = bsxfun(@minus, predictMeasurement, measurement);
            measurementErrorNorm = sum(abs(measurementError), 2);
            measurementNoise = eye(pf.NumStateVariables);
            likelihood = 1/sqrt((2*pi).^3 * det(measurementNoise)) * exp(-2 * measurementErrorNorm);
        end        
        
        % TODO: generate attention map
        function generateAttentionMap(obj, currPose)
            obj.computeHeadCspaceForPose(currPose);
            
            if max(obj.headConfigs(:, 1)) > obj.headYaw_max
                obj.headYaw_max = max(obj.headConfigs(:, 1));
            end
            if min(obj.headConfigs(:, 1)) < obj.headYaw_min
                obj.headYaw_min = min(obj.headConfigs(:, 1));
            end
            if max(obj.headConfigs(:, 2)) > obj.headPitch_max
                obj.headPitch_max = max(obj.headConfigs(:, 2));
            end
            if min(obj.headConfigs(:, 2)) < obj.headPitch_min
                obj.headPitch_min = min(obj.headConfigs(:, 2));
            end
            
            yawLength = floor( (obj.headYaw_max - obj.headYaw_min)/obj.headYawRes + 1 ) + 1;
            pitchLength = floor( (obj.headPitch_max - obj.headPitch_min)/obj.headPitchRes + 1 ) + 1;
            
            obj.attentionMap = zeros([yawLength, pitchLength]);
            
            yawIndex = floor( (obj.headConfigs(:, 1) - obj.headYaw_min)/obj.headYawRes ) + 1;
            pitchIndex = floor( (obj.headConfigs(:, 2) - obj.headPitch_min)/obj.headPitchRes ) + 1;
            
            for i=1:length(yawIndex)
                yawI = yawIndex(i);
                pitchI = pitchIndex(i);
                if ismember(i-1, obj.upperBodyJoints)
                    % KL divergence: (1-1/a)*log(a)
                    % alpha = obj.noiseVar(obj.upperBodyJoints == (i-1)) / obj.noiseVar_prev(obj.upperBodyJoints == (i-1));
                    % kl_div = log(alpha);
                    % obj.attentionMap(yawI, pitchI) = kl_div;
                    obj.attentionMap(yawI, pitchI) = obj.noiseVar(obj.upperBodyJoints == (i-1));
                else
                    obj.attentionMap(yawI, pitchI) = 0;
                end
            end
        end
        
        function [headYaw, headPitch] = getAttentionPoint(obj)
            [~, I] = max(obj.attentionMap(:));
            [I_row, I_col] = ind2sub(size(obj.attentionMap), I);
            
            headYaw = I_row * obj.headYawRes + obj.headYaw_min;
            headPitch = I_col * obj.headPitchRes + obj.headPitch_min;
        end
        
        function generateAttentionMap_fix(obj, PFilter)
            obj.computeHeadCspaceForOneFilter(PFilter);
            obj.attentionMap = zeros([obj.fixScale+1, obj.fixScale+1]);
            
            obj.headYaw_max = max(obj.headConfigs(:, 1));
            obj.headYaw_min = min(obj.headConfigs(:, 1));
            obj.headPitch_max = max(obj.headConfigs(:, 2));
            obj.headPitch_min = min(obj.headConfigs(:, 2));
            
            headYaw_res = ( obj.headYaw_max - obj.headYaw_min )/obj.fixScale;
            headPitch_res = ( obj.headPitch_max - obj.headPitch_min )/obj.fixScale;
            
            yawIndex = floor( (obj.headConfigs(:, 1) - obj.headYaw_min)/headYaw_res ) + 1;
            pitchIndex = floor( (obj.headConfigs(:, 2) - obj.headPitch_min)/headPitch_res ) + 1;
            
            for i=1:length(yawIndex)
                yawI = yawIndex(i);
                pitchI = pitchIndex(i);
                % obj.attentionMap(yawI, pitchI) = obj.attentionMap(yawI, pitchI) + 1;
                obj.attentionMap(yawI, pitchI) = 1;
            end
        end

        
        function generateAttentionMap_res(obj, PFilter)
            for i = 1:20
            end
            
            obj.computeHeadCspace(PFilter);
            
            yawLevels = floor( (obj.headYaw_ceil - obj.headYaw_floor)/obj.headYawRes ) + 1;
            pitchLevels = floor( (obj.headPitch_ceil - obj.headPitch_floor)/obj.headPitchRes ) + 1;
            
            obj.attentionMap = zeros([yawLevels, pitchLevels]);
            yawIndex = floor( (obj.headConfigs(:, 1) - obj.headYaw_floor)/obj.headYawRes ) + 1;
            pitchIndex = floor( (obj.headConfigs(:, 2) - obj.headPitch_floor)/obj.headPitchRes ) + 1;
            
            for i=1:length(yawIndex)
                yawI = yawIndex(i);
                pitchI = pitchIndex(i);
                % obj.attentionMap(yawI, pitchI) = obj.attentionMap(yawI, pitchI) + 1;
                obj.attentionMap(yawI, pitchI) = 1;
            end
        end
        
        function computeHeadCspaceForPose(obj, currPose)
            temp = reshape(currPose, 4, [])';
            poseData = temp(:, 1:3);
            tfInfo = getTransform(obj.tfTree, 'Head', 'odom');
            tfQuat = [tfInfo.Transform.Rotation.W, tfInfo.Transform.Rotation.X, tfInfo.Transform.Rotation.Y, tfInfo.Transform.Rotation.Z];
            tfRot = quat2rotm(tfQuat);
            tfTrans = [tfInfo.Transform.Translation.X, tfInfo.Transform.Translation.Y, tfInfo.Transform.Translation.Z]';
            
            [~, numPar] = size(poseData');
            tfTransBatch = repmat(tfTrans, [1, numPar]); 
            
            tfPts = (tfRot * poseData' + tfTransBatch)';

            headYaw = atan(tfPts(:, 2)./tfPts(:, 1));
            headPitch = -atan(tfPts(:, 3)./sqrt(tfPts(:, 1).^2 + tfPts(:, 2).^2));

            headYaw(headYaw > obj.headYaw_ceil) = obj.headYaw_ceil;
            headYaw(headYaw < obj.headYaw_floor) = obj.headYaw_floor;
            headPitch(headPitch > obj.headPitch_ceil) = obj.headPitch_ceil;
            headPitch(headPitch < obj.headPitch_floor) = obj.headPitch_floor;
            
            obj.headConfigs = [headYaw, headPitch];
        end
        
        function computeHeadCspaceForFilter(obj, PFilter)
            obj.particleNum = PFilter(1).NumParticles;
            
            tfInfo = getTransform(obj.tfTree, 'Head', 'odom');
            
            particles = PFilter.Particles(:, :);
                
            tfQuat = [tfInfo.Transform.Rotation.W, tfInfo.Transform.Rotation.X, tfInfo.Transform.Rotation.Y, tfInfo.Transform.Rotation.Z];
            tfRot = quat2rotm(tfQuat);
            tfTrans = [tfInfo.Transform.Translation.X, tfInfo.Transform.Translation.Y, tfInfo.Transform.Translation.Z]';
            
            [~, numPar] = size(particles');
            tfTransBatch = repmat(tfTrans, [1, numPar]); 
            
            tfPts = (tfRot * particles' + tfTransBatch)';

            headYaw = atan(tfPts(:, 2)./tfPts(:, 1));
            headPitch = -atan(tfPts(:, 3)./sqrt(tfPts(:, 1).^2 + tfPts(:, 2).^2));

            headYaw(headYaw > obj.headYaw_ceil) = obj.headYaw_ceil;
            headYaw(headYaw < obj.headYaw_floor) = obj.headYaw_floor;
            headPitch(headPitch > obj.headPitch_ceil) = obj.headPitch_ceil;
            headPitch(headPitch < obj.headPitch_floor) = obj.headPitch_floor;
            
            obj.headConfigs = [headYaw, headPitch];
        end
        
        function generateJointContour(obj)
            % TODO : generate joint contour
        end

        function showAttentionMap(obj)
            h = heatmap(obj.attentionMap');
            h.Title = 'Attention map in C-Space';
            h.YLabel = 'Head pitch';
            h.XLabel = 'Head yaw';
        end
    end
    

end


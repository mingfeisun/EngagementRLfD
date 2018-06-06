classdef ApproximateMimic < handle
    properties
        cspacePts;
        tfTree;
        plotXYZ;
    end
    
    methods
        function obj = ApproximateMimic()
            obj.cspacePts = zeros([8, 1]);
            obj.tfTree = rostf;
            % obj.plotXYZ = true;
            obj.plotXYZ = false;
        end
        
        function angleConfigs = generateConfigs(obj, keyJointPos)
            % the order: 
            % LHand(1), LArm(2), LShoulder(3), RShoulder(4), RArm(5),
            % RHand(6), Spine(7), Spine1(8)
            
            LHandV = keyJointPos(1, :)' - keyJointPos(2, :)';
            LArmV = keyJointPos(2, :)' - keyJointPos(3, :)';
            RHandV = keyJointPos(6, :)' - keyJointPos(5, :)';
            RArmV = keyJointPos(5, :)' - keyJointPos(4, :)';
            
            SpineV = keyJointPos(8, :)' - keyJointPos(7, :)';
            
            % config order: LElbowRoll, LShoudlerRoll, LShoulderPitch,
            % RShoulderPitch, RShoulderRoll, RElbowRoll

            obj.cspacePts(1) = -2;
            
            obj.cspacePts(2) = - obj.handAngles(LArmV, LHandV);
            
            rightToLeftShoulder = keyJointPos(3, :)'- keyJointPos(4, :)';
            [obj.cspacePts(3), obj.cspacePts(4)] = obj.armAngles(SpineV, rightToLeftShoulder, LArmV); % from right to left
            
            [obj.cspacePts(5), obj.cspacePts(6)] = obj.armAngles(SpineV, rightToLeftShoulder, RArmV);
            
            obj.cspacePts(7) = 0;
            obj.cspacePts(8) = obj.handAngles(RArmV, RHandV);
            
            angleConfigs = obj.cspacePts;
        end
        
        function [armRoll, armPitch] = armAngles(obj, spineV, shoulderV, armV)
            armPitch = pi*2/3 - acos( dot(armV, spineV)/(norm(armV) * norm(spineV)) );
            armRoll = pi*2/3 - acos( dot(shoulderV, armV) / (norm(shoulderV) * norm(armV)) );     
        end
        
        function elbowRoll = handAngles(obj, armV, handV)
            angleHand = acos( dot(armV, handV)/ ( norm(armV)*norm(handV)) );
            elbowRoll = angleHand;
        end

    end
end


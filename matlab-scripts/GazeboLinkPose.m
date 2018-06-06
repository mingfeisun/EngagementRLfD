classdef GazeboLinkPose < handle
    properties
        linkSub;
        latestFlag;
        actorPose;
        pepperPose;
        mapActorToKinect;
        mapPepperJoint;
        linkMsgsNames;
        keyJointIndex
        
        spine1Pos;
    end
    
    methods
        function obj = GazeboLinkPose()
            
            obj.latestFlag = false;
            obj.actorPose = ones(1, 80);
            obj.pepperPose = ones(1, 16);
            obj.linkMsgsNames = {};
            
            skel_model;
            
            obj.keyJointIndex = [NUI_SKELETON_POSITION_WRIST_LEFT, ...
                NUI_SKELETON_POSITION_ELBOW_LEFT, ...
                NUI_SKELETON_POSITION_SHOULDER_LEFT, ...
                NUI_SKELETON_POSITION_SHOULDER_RIGHT, ...
                NUI_SKELETON_POSITION_ELBOW_RIGHT, ...
                NUI_SKELETON_POSITION_WRIST_RIGHT, ...
                NUI_SKELETON_POSITION_SHOULDER_CENTER, ...
                NUI_SKELETON_POSITION_SPINE];
            
            actorJointName = {'actor::Hips';... % 1  NUI_SKELETON_POSITION_HIP_CENTER
                'actor::LHipJoint'         ;... % 2  
                'actor::LeftUpLeg'         ;... % 3  NUI_SKELETON_POSITION_HIP_LEFT
                'actor::LeftLeg'           ;... % 4  NUI_SKELETON_POSITION_KNEE_LEFT
                'actor::LeftFoot'          ;... % 5  NUI_SKELETON_POSITION_ANKLE_LEFT
                'actor::LeftToeBase'       ;... % 6  NUI_SKELETON_POSITION_FOOT_LEFT
                'actor::RHipJoint'         ;... % 7  
                'actor::RightUpLeg'        ;... % 8  NUI_SKELETON_POSITION_HIP_RIGHT
                'actor::RightLeg'          ;... % 9  NUI_SKELETON_POSITION_KNEE_RIGHT
                'actor::RightFoot'         ;... % 10 NUI_SKELETON_POSITION_ANKLE_RIGHT
                'actor::RightToeBase'      ;... % 11 NUI_SKELETON_POSITION_FOOT_RIGHT
                'actor::LowerBack'         ;... % 12
                'actor::Spine'             ;... % 13 NUI_SKELETON_POSITION_SPINE
                'actor::Spine1'            ;... % 14
                'actor::Neck'              ;... % 15 NUI_SKELETON_POSITION_SHOULDER_CENTER
                'actor::Neck1'             ;... % 16
                'actor::Head'              ;... % 17 NUI_SKELETON_POSITION_HEAD
                'actor::LeftShoulder'      ;... % 18
                'actor::LeftArm'           ;... % 19 NUI_SKELETON_POSITION_SHOULDER_LEFT
                'actor::LeftForeArm'       ;... % 20 NUI_SKELETON_POSITION_ELBOW_LEFT
                'actor::LeftHand'          ;... % 21 NUI_SKELETON_POSITION_WRIST_LEFT
                'actor::LeftFingerBase'    ;... % 22 NUI_SKELETON_POSITION_HAND_LEFT
                % 'actor::LeftHandFinger1'  ;... 
                'actor::LeftHandIndex1'    ;... % 23
                'actor::LThumb'            ;... % 24
                'actor::RightShoulder'     ;... % 25
                'actor::RightArm'          ;... % 26 NUI_SKELETON_POSITION_SHOULDER_RIGHT
                'actor::RightForeArm'      ;... % 27 NUI_SKELETON_POSITION_ELBOW_RIGHT
                'actor::RightHand'         ;... % 28 NUI_SKELETON_POSITION_WRIST_RIGHT
                'actor::RightFingerBase'   ;... % 29 NUI_SKELETON_POSITION_HAND_RIGHT
                % 'actor::RightHandFinger1' ;... 
                'actor::RightHandIndex1'   ;... % 30
                'actor::RThumb'            ;... % 31
                'actor::actor_pose'...          % 32
            };
            actorTargetValues =  [...
                NUI_SKELETON_POSITION_HIP_CENTER     ,... % 1
                -1                                   ,... % 2
                NUI_SKELETON_POSITION_HIP_LEFT       ,... % 3
                NUI_SKELETON_POSITION_KNEE_LEFT      ,... % 4
                NUI_SKELETON_POSITION_ANKLE_LEFT     ,... % 5
                NUI_SKELETON_POSITION_FOOT_LEFT      ,... % 6
                -1                                   ,... % 7
                NUI_SKELETON_POSITION_HIP_RIGHT      ,... % 8
                NUI_SKELETON_POSITION_KNEE_RIGHT     ,... % 9
                NUI_SKELETON_POSITION_ANKLE_RIGHT    ,... % 10
                NUI_SKELETON_POSITION_FOOT_RIGHT     ,... % 11
                -1                                   ,... % 12
                NUI_SKELETON_POSITION_SPINE          ,... % 13
                -1                                   ,... % 14
                NUI_SKELETON_POSITION_SHOULDER_CENTER,... % 15
                -1                                   ,... % 16 special need
                NUI_SKELETON_POSITION_HEAD           ,... % 17
                -1                                   ,... % 18
                NUI_SKELETON_POSITION_SHOULDER_LEFT  ,... % 19
                NUI_SKELETON_POSITION_ELBOW_LEFT     ,... % 20
                NUI_SKELETON_POSITION_WRIST_LEFT     ,... % 21
                NUI_SKELETON_POSITION_HAND_LEFT      ,... % 22
                -1                                   ,... % 23
                -1                                   ,... % 24
                -1                                   ,... % 25
                NUI_SKELETON_POSITION_SHOULDER_RIGHT ,... % 26
                NUI_SKELETON_POSITION_ELBOW_RIGHT    ,... % 27
                NUI_SKELETON_POSITION_WRIST_RIGHT    ,... % 28
                NUI_SKELETON_POSITION_HAND_RIGHT     ,... % 29
                -1                                   ,... % 30
                -1                                   ,... % 31
                -1                                    ... % 32
            ];
            obj.mapActorToKinect = containers.Map(actorJointName, actorTargetValues);
            
            pepperJointName = {...
                'pepper::base_link'     ;... % 1
                'pepper::Neck'          ;... % 2
                'pepper::Head'          ;... % 3
                'pepper::Hip'           ;... % 4
                'pepper::Pelvis'        ;... % 5
                'pepper::Tibia'         ;... % 6
                'pepper::WheelB_link'   ;... % 7
                'pepper::WheelFL_link'  ;... % 8
                'pepper::WheelFR_link'  ;... % 9
                'pepper::LShoulder'     ;... % 10
                'pepper::LBicep'        ;... % 11
                'pepper::LElbow'        ;... % 12
                'pepper::LForeArm'      ;... % 13
                'pepper::l_wrist'       ;... % 14
                'pepper::LFinger11_link';... % 15
                'pepper::LFinger12_link';... % 16
                'pepper::LFinger13_link';... % 17
                'pepper::LFinger21_link';... % 18 
                'pepper::LFinger22_link';... % 19 
                'pepper::LFinger23_link';... % 20 
                'pepper::LFinger31_link';... % 21 
                'pepper::LFinger32_link';... % 22 
                'pepper::LFinger33_link';... % 23 
                'pepper::LFinger41_link';... % 24 
                'pepper::LFinger42_link';... % 25 
                'pepper::LFinger43_link';... % 26 
                'pepper::l_gripper'     ;... % 27
                'pepper::LThumb1_link'  ;... % 28 
                'pepper::LThumb2_link'  ;... % 29 
                'pepper::RShoulder'     ;... % 30 
                'pepper::RBicep'        ;... % 31 
                'pepper::RElbow'        ;... % 32 
                'pepper::RForeArm'      ;... % 33 
                'pepper::r_wrist'       ;... % 34 
                'pepper::RFinger11_link';... % 35 
                'pepper::RFinger12_link';... % 36 
                'pepper::RFinger13_link';... % 37
                'pepper::RFinger21_link';... % 38 
                'pepper::RFinger22_link';... % 39 
                'pepper::RFinger23_link';... % 40 
                'pepper::RFinger31_link';... % 41 
                'pepper::RFinger32_link';... % 42 
                'pepper::RFinger33_link';... % 43 
                'pepper::RFinger41_link';... % 44 
                'pepper::RFinger42_link';... % 45 
                'pepper::RFinger43_link';... % 46 
                'pepper::r_gripper'     ;... % 47
                'pepper::RThumb1_link'  ;... % 48
                'pepper::RThumb2_link'...    % 49
            };
            pepperTargetValues = [...
                -1,...% 1
                -1,...% 2
                 0,...% 3
                 1,...% 4
                -1,...% 5
                -1,...% 6
                -1,...% 7
                -1,...% 8
                -1,...% 9
                -1,...% 10
                -1,...% 11
                -1,...% 12
                -1,...% 13
                 2,...% 14
                -1,...% 15
                -1,...% 16
                -1,...% 17
                -1,...% 18
                -1,...% 19
                -1,...% 20
                -1,...% 21
                -1,...% 22
                -1,...% 23
                -1,...% 24
                -1,...% 25
                -1,...% 26
                -1,...% 27
                -1,...% 28
                -1,...% 29
                -1,...% 30
                -1,...% 31
                -1,...% 32
                -1,...% 33
                 3,...% 34
                -1,...% 35
                -1,...% 36
                -1,...% 37
                -1,...% 38
                -1,...% 39
                -1,...% 40
                -1,...% 41
                -1,...% 42
                -1,...% 43
                -1,...% 44
                -1,...% 45
                -1,...% 46
                -1,...% 47
                -1,...% 48
                -1    % 49
            ];
            obj.mapPepperJoint = containers.Map(pepperJointName, pepperTargetValues);
        end
        
        function Init(obj)
            obj.linkSub = rossubscriber('/gazebo/link_states', @obj.onCallback);
        end
        
        function [currPepperPose, currActorPose, currFlag] = Update(obj)
            currPepperPose = obj.pepperPose;
            currActorPose = obj.actorPose;
            currFlag = obj.latestFlag;
            obj.latestFlag = false;
        end
        
        function onCallback(obj, ~, linkMsgs)
            if isempty(obj.linkMsgsNames)
                obj.linkMsgsNames = linkMsgs.Name;
            end
            for i=1:length(obj.linkMsgsNames)
                currName = obj.linkMsgsNames{i};
                obj.parseMsgs(linkMsgs.Pose(i, 1), currName);
            end
            obj.latestFlag = true;
        end
        
        function parseMsgs(obj, currPose, currName)
            nameCell = strsplit(currName, '::');
            if strcmp(nameCell(1), 'actor')
                mapInd = obj.mapActorToKinect(currName);
                if mapInd == -1
                    return;
                end
                if mapInd == -2
                    obj.spine1Pos = [currPose.Position.X, currPose.Position.Y, currPose.Position.Z];
                    return;
                end
                obj.actorPose(mapInd*4+1:mapInd*4+3) = [currPose.Position.X, currPose.Position.Y, currPose.Position.Z];
            elseif strcmp(nameCell(1), 'pepper')
                mapInd = obj.mapPepperJoint(currName);
                if mapInd == -1
                    return;
                end
                obj.pepperPose(mapInd*4+1:mapInd*4+3) = [currPose.Position.X, currPose.Position.Y, currPose.Position.Z];
            end
        end
        
        function jointPt = getJointPt(obj, jointName)
            mapInd = obj.mapActorToKinect(jointName);
            jointPt = rosmessage('geometry_msgs/Point');
            if mapInd == -1
                return;
            else
                jointPt.X = obj.actorPose(mapInd*4+1);
                jointPt.Y = obj.actorPose(mapInd*4+2);
                jointPt.Z = obj.actorPose(mapInd*4+3);
            end
        end
                
        function jointPos = getKeyJoints(obj)
            jointPos = zeros([length(obj.keyJointIndex), 3]);
            for i = 1:length(obj.keyJointIndex)
                ind = obj.keyJointIndex(i);
                jointPos(i, :) = [obj.actorPose(ind*4+1), obj.actorPose(ind*4+2), obj.actorPose(ind*4+3)];
            end
        end
    end
end


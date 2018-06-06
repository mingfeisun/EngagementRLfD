classdef RobotControl < handle
    properties
        headPub;  
        armPub;     
        torsoPub;
        tfTree;
        msgType;
    end
    
    methods
        function obj = RobotControl()
            obj.msgType = 'std_msgs/Float64';
            
            headStr = {'/pepper_dcm/HeadYaw_position_controller/command'; '/pepper_dcm/HeadPitch_position_controller/command'};
            
            armStr = {
                '/pepper_dcm/LElbowYaw_position_controller/command'; '/pepper_dcm/LElbowRoll_position_controller/command';...
                '/pepper_dcm/LShoulderRoll_position_controller/command'; '/pepper_dcm/LShoulderPitch_position_controller/command';...
                '/pepper_dcm/RShoulderRoll_position_controller/command'; '/pepper_dcm/RShoulderPitch_position_controller/command';...
                '/pepper_dcm/RElbowYaw_position_controller/command'; '/pepper_dcm/RElbowRoll_position_controller/command'};
            
            torsoStr = {'/pepper_dcm/HipRoll_position_controller/command'; '/pepper_dcm/HipPitch_position_controller/command'};
            
            obj.headPub = [];
            obj.armPub = [];
            obj.torsoPub = [];
             
            obj.tfTree = rostf;
            pause(1);
            
            for i=1:length(headStr)
                obj.headPub = [obj.headPub; rospublisher(headStr{i}, obj.msgType)];
            end
            
            for i=1:length(armStr)
                obj.armPub = [obj.armPub; rospublisher(armStr{i}, obj.msgType)];
            end
            
            for i=1:length(torsoStr)
                obj.torsoPub = [obj.torsoPub; rossubscriber(torsoStr{i}, obj.msgType)];
            end
        end
        
        function [headYaw, headPitch] = GetHeadCspace(obj, pt)
            waitForTransform(obj.tfTree, 'Head', 'odom');
            targetPt = rosmessage('geometry_msgs/PointStamped');
            targetPt.Header.FrameId = 'odom';
            targetPt.Point = pt;
            tfPt = transform(obj.tfTree, 'Head', targetPt);
            
            % head yaw: -2.08567 -- 2.08567
            % head pitch: -0.706858 -- 0.637045
            headYaw = atan(tfPt.Point.Y/tfPt.Point.X);
            headPitch = -atan(tfPt.Point.Z/sqrt(tfPt.Point.X^2 + tfPt.Point.Y^2));   
        end
        
        function LookAt(obj, pt)
            [headYaw, headPitch] = obj.GetHeadCspace(pt);
            rotMsgs = [rosmessage(obj.msgType), rosmessage(obj.msgType)];
            rotMsgs(1).Data = headYaw;
            rotMsgs(2).Data = headPitch;
            for i=1:length(obj.headPub)
                send(obj.headPub(i), rotMsgs(i));
            end
        end
        
        function TurnHead(obj, headYaw, headPitch)
            msg = rosmessage(obj.msgType);
            msg.Data = headYaw;
            send(obj.headPub(1), msg);
            msg.Data = headPitch;
            send(obj.headPub(2), msg);
        end
        
        function BehaveLike(obj, angleConfigs)
            msg = rosmessage('std_msgs/Float64');
            for i=1:length(obj.armPub)
                msg.Data = angleConfigs(i)/2;
                send(obj.armPub(i), msg);
            end
        end
    end
end


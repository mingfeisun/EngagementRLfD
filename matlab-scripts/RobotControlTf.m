classdef RobotControlTf < handle
    properties
        vis_mode;
        delay_time;
        
        clock_sub;
        
        actorFrameP;
        actorFrameC;
        
        numAngle;
        eulerAngles;
        
        headPub;
        
        LSRPub;
        LSPPub;
        LEYPub;
        LERPub;
        LWYPub;
        
        RSRPub;
        RSPPub;
        REYPub;
        RERPub;
        RWYPub;        
        
        tfTree;
        msgType;
        
        attention;
        linkPose;
        h;
        PFilter;
    end
    
    methods
        function obj = RobotControlTf()
            obj.vis_mode = true;
            obj.delay_time = 0.8;
            
            obj.tfTree = rostf;
            
            obj.numAngle = 30;
            obj.eulerAngles = cell(obj.numAngle, 1);
            
            obj.clock_sub = rossubscriber('/clock');

            obj.actorFrameP = {'actor_LeftShoulder', 'actor_LeftArm', 'actor_RightShoulder', 'actor_RightArm'};
            obj.actorFrameC = {'actor_LeftArm', 'actor_LeftForeArm', 'actor_RightArm', 'actor_RightForeArm'};
            msgType = 'std_msgs/Float64';

            lshoulderRoll = '/pepper_dcm/LShoulderRoll_position_controller/command';
            lshoulderPitch = '/pepper_dcm/LShoulderPitch_position_controller/command';
            lelbowYaw = '/pepper_dcm/LElbowYaw_position_controller/command';
            lelbowRoll = '/pepper_dcm/LElbowRoll_position_controller/command';
            lwristYaw = '/pepper_dcm/LWristYaw_position_controller/command';

            rshoulderRoll = '/pepper_dcm/RShoulderRoll_position_controller/command';
            rshoulderPitch = '/pepper_dcm/RShoulderPitch_position_controller/command';
            relbowYaw = '/pepper_dcm/RElbowYaw_position_controller/command';
            relbowRoll = '/pepper_dcm/RElbowRoll_position_controller/command';
            rwristYaw = '/pepper_dcm/RWristYaw_position_controller/command';

            obj.LSRPub = rospublisher(lshoulderRoll, msgType);
            obj.LSPPub = rospublisher(lshoulderPitch, msgType);
            obj.LEYPub = rospublisher(lelbowYaw, msgType);
            obj.LERPub = rospublisher(lelbowRoll, msgType);
            obj.LWYPub = rospublisher(lwristYaw, msgType);

            obj.RSRPub = rospublisher(rshoulderRoll, msgType);
            obj.RSPPub = rospublisher(rshoulderPitch, msgType);
            obj.REYPub = rospublisher(relbowYaw, msgType);
            obj.RERPub = rospublisher(relbowRoll, msgType);
            obj.RWYPub = rospublisher(rwristYaw, msgType);

            obj.msgType = 'std_msgs/Float64';
            
            headStr = {'/pepper_dcm/HeadYaw_position_controller/command'; '/pepper_dcm/HeadPitch_position_controller/command'};
            
            obj.headPub = [];
             
            obj.tfTree = rostf;
            pause(1);
            
            for i=1:length(headStr)
                obj.headPub = [obj.headPub; rospublisher(headStr{i}, obj.msgType)];
            end
            
            obj.attention = AttentionMap();
            
            if obj.vis_mode
                obj.h=axes;
            end
            
            obj.PFilter = 0;
            obj.linkPose = GazeboLinkPose();
        end
        
        function Init(obj)
            obj.linkPose.Init();
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
        
        function AttentionEngage(obj)
            [~, actorPose, flag] = obj.linkPose.Update();
            if ~flag
                pause(0.01);
                return;
            end

            [posePred, obj.PFilter, ~] = obj.attention.StartTracking(actorPose);

            obj.attention.generateAttentionMap(actorPose);
            
            [headYaw, headPitch] = obj.attention.getAttentionPoint();
            obj.TurnHead(headYaw, headPitch);
            
            if obj.vis_mode
                % obj.attention.showAttentionMap();
                cla;
                skel_vis(actorPose, 1, obj.PFilter, obj.h, 'true');
                skel_vis(posePred, 1, obj.PFilter, obj.h, 'pred');
            end
        end
        
        function MimicEngage(obj)
            waitForTransform(obj.tfTree, obj.actorFrameC{1}, obj.actorFrameP{1});
            % tform_new1 = getTransform(obj.tfTree, obj.actorFrameC{1}, obj.actorFrameP{1}, desiredTime); 
            tform_new1 = getTransform(obj.tfTree, obj.actorFrameC{1}, obj.actorFrameP{1});
            
            waitForTransform(obj.tfTree, obj.actorFrameC{2}, obj.actorFrameP{2});
            % tform_new2 = getTransform(obj.tfTree, obj.actorFrameC{2}, obj.actorFrameP{2}, desiredTime);
            tform_new2 = getTransform(obj.tfTree, obj.actorFrameC{2}, obj.actorFrameP{2});
            

            waitForTransform(obj.tfTree, obj.actorFrameC{3}, obj.actorFrameP{3});
            % tform_new3 = getTransform(obj.tfTree, obj.actorFrameC{3}, obj.actorFrameP{3}, desiredTime);
            tform_new3 = getTransform(obj.tfTree, obj.actorFrameC{3}, obj.actorFrameP{3});
            

            waitForTransform(obj.tfTree, obj.actorFrameC{4}, obj.actorFrameP{4});
            % tform_new4 = getTransform(obj.tfTree, obj.actorFrameC{4}, obj.actorFrameP{4}, desiredTime);
            tform_new4 = getTransform(obj.tfTree, obj.actorFrameC{4}, obj.actorFrameP{4});
            
            tform_rot = tform_new1.Transform.Rotation;
            eul1 = quat2eul([tform_rot.W, tform_rot.X, tform_rot.Y, tform_rot.Z], 'ZYX');
            
            tform_rot = tform_new2.Transform.Rotation;
            eul2 = quat2eul([tform_rot.W, tform_rot.X, tform_rot.Y, tform_rot.Z], 'ZYX');
            
            tform_rot = tform_new3.Transform.Rotation;
            eul3 = quat2eul([tform_rot.W, tform_rot.X, tform_rot.Y, tform_rot.Z], 'ZYX');
            
            tform_rot = tform_new4.Transform.Rotation;
            eul4 = quat2eul([tform_rot.W, tform_rot.X, tform_rot.Y, tform_rot.Z], 'ZYX');
            
            firstEuler = obj.eulerAngles{1};
            obj.eulerAngles = { obj.eulerAngles{2:obj.numAngle}, {eul1, eul2, eul3, eul4} };
            
            if ~isempty(firstEuler)
                obj.SendAngles(firstEuler{1}, firstEuler{2}, firstEuler{3}, firstEuler{4});
            end
        end
        
        function SendAngles(obj, eul1, eul2, eul3, eul4)
            % eul_degree = eul*180/pi;
            % display(eul_degree);

            msg = rosmessage('std_msgs/Float64');
            msg.Data = eul1(1) - 0.5;
            
            send(obj.LSPPub, msg);

            msg = rosmessage('std_msgs/Float64');
            msg.Data = eul1(2) - 1.5;
            send(obj.LEYPub, msg);

            msg = rosmessage('std_msgs/Float64');
            msg.Data = - eul1(3) - 0.5;
            send(obj.LSRPub, msg);


            % eul_degree = eul*180/pi;
            % display(eul_degree);

            msg = rosmessage('std_msgs/Float64');
            msg.Data = -eul2(2) - 0.5;
            send(obj.LWYPub, msg);

            msg = rosmessage('std_msgs/Float64');
            msg.Data = eul2(3);
            send(obj.LERPub, msg);
            

            % eul_degree = eul*180/pi;
            % display(eul_degree);
            
            msg = rosmessage('std_msgs/Float64');
            msg.Data = -eul3(1) - 0.5;
            send(obj.RSPPub, msg);

            msg = rosmessage('std_msgs/Float64');
            msg.Data = eul3(2) + 1.5;
            send(obj.REYPub, msg);

            msg = rosmessage('std_msgs/Float64');
            
            msg.Data = - eul3(3) - 0.5;
            send(obj.RSRPub, msg);
            

            % eul_degree = eul*180/pi;
            % display(eul_degree);

            msg = rosmessage('std_msgs/Float64');
            msg.Data = -eul4(2) - 0.5;
            send(obj.RWYPub, msg);

            msg = rosmessage('std_msgs/Float64');
            msg.Data = -eul4(3);
            send(obj.RERPub, msg);
        end
    end
end


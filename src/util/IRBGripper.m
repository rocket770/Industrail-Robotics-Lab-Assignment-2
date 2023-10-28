classdef IRBGripper < handle
    % Controlls the Gripper, which is comprised of 3 Finger Robots.
    
    properties
        robotModel % Robot the griper is attached to
        finger1 % Fingers that the gripper is comprised of
        finger2
        finger3
        qFinger1OpenToClose
        qFingerOpenToClose
        qFinger1CloseToOpen
        qFingerCloseToOpen
        gripperSteps % Animation detail
        delay % Animation delay
        prevState = [0 0] % check state update
    end
    
    methods
        function obj = IRBGripper(robotModel, delay, gripperOpen, gripperClosed)
            obj.robotModel = robotModel;

            % A gripper is comprised of 3 fingers that may move
            % indepednantly from one another
            obj.finger1 = Finger();
            obj.finger2 = Finger();
            obj.finger3 = Finger();
            
            % 0 animation delay, as every robot part will be controlled by 1 variable
            obj.finger1.model.delay = 0;
            obj.finger2.model.delay = 0;
            obj.finger3.model.delay = 0;

            obj.gripperSteps = 10; 
            
            % Pre-calcualte the joint angles to animate finger 1 opening and closing,
            % as it goes the opposite direction to fingers 2 and 3
            obj.qFinger1OpenToClose = jtraj(gripperOpen, gripperClosed, obj.gripperSteps);
            obj.qFinger1CloseToOpen = jtraj(gripperClosed, gripperOpen, obj.gripperSteps);
            
            % Joint angles for animation of fingers 2 and 3 opening and closing
            obj.qFingerOpenToClose = jtraj(gripperOpen, -gripperClosed, obj.gripperSteps);
            obj.qFingerCloseToOpen = jtraj(-gripperClosed, gripperOpen, obj.gripperSteps);



            % Initally set the gripper to the EE of the robot, in an open state.
            obj.updateGripperPosition(robotModel, gripperOpen);

            obj.delay = delay;

        end
        function updateGripperPosition(obj, robotModel, qF)
            % End effector of the robot
            EE = robotModel.fkine(robotModel.getpos()).T;

            %fprintf('Robot End Effector: %f', EE );
            
            %The offset each finger sits relative to the end effector of the robot
            offset1 = transl(0.04, 0, 0);
            offset2 = transl(-0.02, 0.0333, 0);
            offset3 = transl(-0.02, -0.0333, 0);
            % Compute each position relative to the end effector, given a
            % translation and rotation relative to it 
            obj.finger1.model.base = EE * offset1 * troty(-pi/2);
            
            obj.finger2.model.base = EE *  offset2 * troty(-pi/2);
            
            obj.finger3.model.base = EE * offset3 * troty(-pi/2);
            
            
            obj.finger1.model.animate(qF);
            obj.finger2.model.animate(-qF); % Fingers 2 and 3 are facing the other way/
            obj.finger3.model.animate(-qF);


            %Disp(qF)
        end

        % Animate the hand from an open state all through to a defined
        % close state
        function closeHand(obj) 
            for j = 1:obj.gripperSteps
                obj.finger1.model.animate(obj.qFinger1OpenToClose(j,:));
                obj.finger2.model.animate(obj.qFingerOpenToClose(j,:));
                obj.finger3.model.animate(obj.qFingerOpenToClose(j,:));
                drawnow();
                pause(obj.delay);
            end
        end
        
        % Animate the hand from an closed state all through to a defined
        % open state
        function openHand(obj)
             for j = 1:obj.gripperSteps
                obj.finger1.model.animate(obj.qFinger1CloseToOpen(j,:));
                obj.finger2.model.animate(obj.qFingerCloseToOpen(j,:));
                obj.finger3.model.animate(obj.qFingerCloseToOpen(j,:));
                drawnow();
                pause(obj.delay);
            end
        end

    end
end


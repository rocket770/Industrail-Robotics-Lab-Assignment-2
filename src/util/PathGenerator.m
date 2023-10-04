classdef PathGenerator < handle
    %RMRC Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        robot
        mask
    end
    
    methods
        function obj = PathGenerator(robot, mask)
            obj.robot = robot;
            obj.mask = mask;
        end

        function [iTraj, fTraj] = getRMRC(obj, x, steps, deltaT) 
            % Moving the robot to the starting point of the circle
            t_start = [x(1,1) x(2,1) x(3,1)]';
            T_start = [eye(3) t_start; zeros(1,3) 1];
            
            % Getting the initial quess of the robot            
            initial_q = obj.generateInitalQ(t_start); 

            q_start = obj.robot.ikine(T_start, initial_q, 'mask', obj.mask); % inital_q is a bad guess
            
            iTraj = obj.getMinJerkProfile(initial_q, q_start, steps); % Generating a joint-space trajectory
                         
            
            fTraj = nan(steps, obj.robot.n);
            fTraj(1,:) = q_start;
            
            % RMRC Calculation
            for i = 1:steps-1
                xdot = (x(:,i+1) - x(:,i))/deltaT;
                J = obj.robot.jacob0(fTraj(i,:)); % Get the Jacobian at the current state
                J = J(1:3,:); % Taking only the first 3 rows
                qdot = pinv(J)*xdot; % Using pseudo-inverse to solve velocities via RMRC
                fTraj(i+1,:) = fTraj(i,:) + deltaT*qdot'; % Update next joint state
            end

        end        
    
        function qMatrix = getMinJerkProfile(obj, q1, q2, steps)
            % Generate the LSPB speed profile between 0 and 1 for the given number of steps
            % s is a vector where each value is a blend between q1 and q2.
            s = lspb(0, 1, steps);
            
            % Initialize the trajectory matrix with NaN values
            qMatrix = nan(steps, obj.robot.n);
            
            % Loop over each step to compute the joint configurations
            for i = 1:steps
                % Blend between the initial and final configurations using the LSPB speed profile
                qMatrix(i, :) = (1 - s(i)) * q1 + s(i) * q2;
            end
        end

        function q =  generateInitalQ(obj, t)
               % Calculate the direction vector from the robot to the target.
   
                direction = t - obj.robot.base.t;
                
                % Compute the azimuth and elevation angles.
                azimuth = atan2(direction(2), direction(1));
                %elevation = atan2(direction(3), sqrt(direction(1)^2 + direction(2)^2));
                % Set the initial guess based on the angles and some mid-range values.
                q = [azimuth, 0.087266, zeros(1, obj.robot.n-2)];  % Just an example. Adjust accordingly.
        end



    end
end


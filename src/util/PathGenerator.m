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

        function [qMatrix] = getWaypointRMRC(obj, waypoints, steps_per_waypoint, deltaT)
            epsilon = 0.002;



            traj_waypoints = length(waypoints);
            qMatrix = zeros(steps_per_waypoint, 5, traj_waypoints);

            for point = 1:traj_waypoints -1

                m = zeros(steps_per_waypoint, 1);             % Array for Measure of Manipulability
                qdot = zeros(steps_per_waypoint, 4);

                x1 = waypoints(:, point);
                x2 = waypoints(:, point+1);


                T1 = [eye(3) [x1(1:3)]; zeros(1,3) 1] * troty(-pi); % start point


                if point == 1
                    q1 = obj.robot.ikine(T1, 'q0', [0    0.7854    1.5708    0.7854   0], 'mask', obj.mask);
                    qMatrix(1,:,1) = q1;
                    %obj.robot.animate(q1)
                else
                    qMatrix(1, :, point) = qMatrix(steps_per_waypoint,:, point-1);
                end

                x = zeros(4,steps_per_waypoint);
                s = lspb(0,1,steps_per_waypoint); % Create interpolation scalar

                for step = 1:steps_per_waypoint
                    x(:,step) = x1*(1-s(step)) + s(step)*x2; % Create trajectory in x-y  plane
                end

                for i = 1:steps_per_waypoint-1
                    xdot = (x(:,i+1) - x(:,i))/deltaT; % Calculate velocity at discrete time step



                    J = obj.robot.jacob0(qMatrix(i, :, point)); % Get the Jacobian at the current state

                    J = J(1:4,1:4); % Take only first 4 rows

                    m(i) = sqrt(det(J*J'));

                    if  m(i) < epsilon  % If manipulability is less than given threshold
                        lambda = (1 - m(i)/epsilon)*5E-2;
                    else
                        lambda = 0;
                    end

                    invJ = inv(J'*J + lambda * eye(4))*J';                                   % DLS Inverse
                    qdot(i,:) = (invJ*xdot)';% Solve velocitities via RMRC

                    for j = 1:4                                                            % Loop through joints 1 to 4 as that is what we're changing
                        if qMatrix(i,j) + deltaT*qdot(i,j) < obj.robot.qlim(j,1)                     % If next joint angle is lower than joint limit...
                            qdot(i,j) = 0; % Stop the motor
                        elseif qMatrix(i,j) + deltaT*qdot(i,j) >  obj.robot.qlim(j,2)                 % If next joint angle is greater than joint limit ...
                            qdot(i,j) = 0; % Stop the motor                
                        end
                    end

                    qMatrix(i+1, :, point) = qMatrix(i, :, point) + deltaT*[qdot(i,:), 0];
                end
            end

        end

    
        function [qMatrix] = getPointRMRC(obj, x1, x2, steps, deltaT)
            epsilon = 0.02;
            m = zeros(steps, 1);             % Array for Measure of Manipulability
            qdot = zeros(steps, 4);


            T1 = [eye(3) x1(1:3); zeros(1,3) 1] * troty(-pi); % start point

            q1 = obj.robot.ikine(T1, 'q0',[0    0.7854    1.5708    0.7854   0], 'mask', obj.mask); % Solve for jointangles in START

            obj.robot.animate(q1);


            x = zeros(4,steps);
            s = lspb(0,1,steps); % Create interpolation scalar

            for i = 1:steps
                x(:,i) = x1*(1-s(i)) + s(i)*x2; % Create trajectory in x-y  plane
            end
            qMatrix = nan(steps,5);
            qMatrix(1,:) = q1; % Solve for joint angles

            for i = 1:steps-1
                xdot = (x(:,i+1) - x(:,i))/deltaT; % Calculate velocity at discrete time step
                J = obj.robot.jacob0(qMatrix(i,:));
                % Get the Jacobian at the current state
                J = J(1:4,1:4); % Take only first 2 rows
                m(i) = sqrt(det(J*J'));

                if  m(i) < epsilon  % If manipulability is less than given threshold
                    lambda = (1 - m(i)/epsilon)*5E-2;
                else
                    lambda = 0;
                end

                invJ = inv(J'*J + lambda * eye(4))*J';                                   % DLS Inverse
                qdot(i,:) = (invJ*xdot)';% Solve velocitities via RMRC

                for j = 1:4                                                            % Loop through joints 1 to 4 as that is what we're changing
                    if qMatrix(i,j) + deltaT*qdot(i,j) < obj.robot.qlim(j,1)                     % If next joint angle is lower than joint limit...
                        qdot(i,j) = 0; % Stop the motor
                    elseif qMatrix(i,j) + deltaT*qdot(i,j) >  obj.robot.qlim(j,2)                 % If next joint angle is greater than joint limit ...
                        qdot(i,j) = 0; % Stop the motor

                    end
                end

                qMatrix(i+1, :) = qMatrix(i, :) + deltaT*[qdot(i,:), 0];
            end

        end
    end
end

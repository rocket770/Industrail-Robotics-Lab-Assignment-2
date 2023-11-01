close all;
clear all;

%% Global Variables
mask = [1 1 1 0 0 0]; % Consider x, y, and z translations and Y rotations for RMRC

% used to toggle collision demonstration
useCollisions = false;
% collisionPoint = 1 is box on table, = 2 is box before oven
collisionPoint = 2;

% only set true if arduino is connected
useArduino = false;
comPort = "COM3";

%% Workspace Setup

environment = EnvironmentSetup;

[bowl, bowl_verts] = environment.placeBowl();

dobot = environment.placeDobot();
IRB = environment.placeIRB12009();

dobot.model.delay = 0;
IRB.model.delay = 0;

%% Collision Detector

collisionDetector = CollisionDetector(IRB.model);

%% Collision Object
if useCollisions
    plotOptions.plotFaces = true;

    if collisionPoint == 1
    centerpnt = [1.5,-1.2,0.6];
    [vertex,faces,faceNormals] = RectangularPrism(centerpnt-0.1, centerpnt + 0.3, plotOptions);


    else
    centerpnt = [1.2,-2.2,0.1];
    [vertex,faces,faceNormals] = RectangularPrism(centerpnt-0.1, centerpnt + 0.8, plotOptions);

    end
    
    % Plot box here  
    collisionDetector.addObstacle(vertex,faces,faceNormals);
end
%% RMRC Cirlce Path

pathGenerator = PathGenerator(dobot.model, mask);

% Circle parameters
center = [2 -1.24 0.6]'; % [x, y, z] - Center of the circle, z-coordinate specifies the height
radius = 0.02; % Radius of the circle

steps_per_waypoint = 20;
traj_waypoints = 20;

waypoints = zeros(4,traj_waypoints);

angle = linspace(0, 2*pi, traj_waypoints); % Generating n points along the circle for more accuracy


% Generating the circular trajectory in x-y plane with specified height
for i = 1:traj_waypoints
    waypoints(1,i) = center(1) + radius * cos(angle(i));
    waypoints(2,i) = center(2) + radius * sin(angle(i));
    waypoints(3,i) = center(3);
    waypoints(4,i) = 0;
end

deltaT = 0.02;

circleQMatrix = pathGenerator.getWaypointRMRC(waypoints, steps_per_waypoint, deltaT);

%% Lift Up RMRC
x1 = [center(1) - radius center(2) center(3)  0]';
x2 = [center(1) - radius center(2) center(3) + 0.04 0]';

pointSteps = 30;

liftQMatrix = pathGenerator.getPointRMRC(x1, x2, pointSteps, deltaT);

%% Move Away Dobot
x3 = x2 + [0.1, -0.06 0 0]';

moveQMatrix = pathGenerator.getPointRMRC(x2, x3, pointSteps, deltaT);

%% IRB Waypoints
%Position Waypoints

IRBPositions = [
    1.7 -1.227 0.64;
    1.82 -1.227 0.64;
    1.8067 -1.17 0.8802;
    0.795 -1.5 1.1
    0.795 -1.81 0.8287;
    0.795 -1.5 1.1
    ];

IRBAngles = [
    -0.2967    1.4451   -0.3770         0   -0.7854         0;
    -0.2967    1.4451   -0.3770         0   -0.7854         0;
    -0.3    0.8   -0.05        0         0         0;
    -0.2546   0.6178   -0.1599         0         0         0;   % THish put this in
    -2   1.1   -0.5         0         0         0;
    -0.2546   0.6178   -0.1599         0         0         0;   % THish put this in
    ];

%% Gripper

gripperOpen = [0,0];
gripperClosed = [0, pi/8];

% Init each finger in the gripper
gripper = IRBGripper(IRB.model, 0.1, gripperOpen, gripperClosed);
gripper.updateGripperPosition(IRB.model, gripperOpen)

%% E Stop
eStop = EStop();

if useArduino
    listener = ArduinoListener(comPort, 9600, eStop);
end

%% Main loop

stiringRepetions = 1;

delay = 0.005;

state = "INIT";

currentRep = 1;
currentPoint = 1;
currentStep = 1;
currentIRBPos = 1;
currentIRBStep = 1;

while ~strcmp(state, 'FINISHED')

    if useArduino
        listener.checkForData();
    end

    
    paused = eStop.getEStopState();

    if useCollisions && ~paused
        q = IRB.model.getpos();
        collision = collisionDetector.detectCollision(q);
        
        if collision
            disp('Collision Detected')
            eStop.triggerEStop();
        end

    end

    if paused
        drawnow();
        %disp('paused')
        continue;
    end

    switch state
        case "INIT"
            % Initialization

            state = "STIR_BOWL";

        case "STIR_BOWL"
            if currentRep <= stiringRepetions
                if currentPoint < length(waypoints)
                    if currentStep <= steps_per_waypoint
                        dobot.model.animate(circleQMatrix(currentStep, :, currentPoint));
                        drawnow();
                        currentStep = currentStep + 1;

                    else
                        currentStep = 1;
                        currentPoint = currentPoint + 1;
                    end
                else
                    currentPoint = 1;
                    currentRep = currentRep + 1;
                end

                if currentRep > stiringRepetions
                    state = "LIFT_ARM";
                    delay = 0.01;
                end

                
            end

        case "LIFT_ARM"
            if currentStep <= pointSteps
                dobot.model.animate(liftQMatrix(currentStep,:));
                drawnow();
                currentStep = currentStep + 1;
            else
                currentStep = 1;
                state = "MOVE_DOBOT";
            end

        case "MOVE_DOBOT"
            if currentStep <= pointSteps
                dobot.model.animate(moveQMatrix(currentStep,:));
                drawnow();
                currentStep = currentStep + 1;
            else
                currentStep = 1;
                state = "IRB_MOVEMENT";
            end

        case "IRB_MOVEMENT"
            if currentIRBPos <= length(IRBPositions)

                if ~exist('irbSubState', 'var') || strcmp(irbSubState, 'NEW_POSITION')
                    qCurrent = IRB.model.getpos();
                    goal = transl(IRBPositions(currentIRBPos,:)) * troty(pi/2);
                    qGoal = IRB.model.ikine(goal, IRBAngles(currentIRBPos,:), 'mask', mask);
                    qMatrix = jtraj(qCurrent, qGoal, pointSteps);

                    switch currentIRBPos
                        case 1
                            gripperState = gripperOpen;
                        case 3
                            gripper.closeHand();
                            gripperState = gripperClosed;
                        case 6
                            gripper.openHand();
                            gripperState = gripperOpen;
                    end

                    irbSubState = 'RUNNING_STEPS';
                end

                if strcmp(irbSubState, 'RUNNING_STEPS')
                    if currentIRBStep <= pointSteps
                        IRB.model.animate(qMatrix(currentIRBStep,:));
                        gripper.updateGripperPosition(IRB.model, gripperState);

                        if (currentIRBPos > 2 && currentIRBPos < 6)
                            T = getPos(IRB.model);
                            environment.updateObjectPosition(bowl, bowl_verts, T);
                        end

                        drawnow();
                        currentIRBStep = currentIRBStep + 1;
                    else
                        currentIRBStep = 1;
                        currentIRBPos = currentIRBPos + 1;
                        irbSubState = 'NEW_POSITION';
                    end
                end

            else
                state = "FINISHED";
            end

        case "FINISHED"
            % Code has finished all tasks.
            break;
    end
    pause(delay)
end

if useArduino
    listener.delete();
end

function position = getPos(robotModel)
    % Compute the end effector's position and rotation
    endEffectorTransform = robotModel.fkine(robotModel.getpos());
    endEffectorPos = endEffectorTransform.t;
    
    % Get the directions from the rotation matrix
    endEffectorRot = endEffectorTransform.R;
    xDirection = endEffectorRot(:, 1);
    yDirection = endEffectorRot(:, 2);
    zDirection = endEffectorRot(:, 3);
    
    % Compute the offset position
    offsetPos = endEffectorPos;
    
    % Create a rotation matrix that keeps the y-axis pointing upwards

    % Global Z axis
    zAxis = [0; 0; 1];
    
    % The y-axis is perpendicular to the z-axis and the end effector's x-axis
    yDirection = cross(zAxis, xDirection);
    yDirection = yDirection / norm(yDirection); % Normalize the vector
    
    % Recompute the x-axis to ensure orthogonality
    xDirection = cross(yDirection, zAxis);
    
    % Assemble the rotation matrix
    uprightRot = [xDirection, yDirection, zAxis];
    
    % Create a transformation matrix for the offset position and upright rotation
    position = [uprightRot, offsetPos; 0, 0, 0, 1] * transl(0.2, 0, -0.1);
end


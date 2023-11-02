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
comPort = "COM3"; %connect to according com for arduino connection 

%% Set up the environment for the workspace
environment = EnvironmentSetup;

% Place a bowl in the environment
[bowl, bowl_verts] = environment.placeBowl();

% Place Dobot and IRB robot in the environment
dobot = environment.placeDobot();
IRB = environment.placeIRB12009();

% Set model delays to zero assuming instantaneous robot responses
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
    
    % Add the obstacle (the box) to the collision detector
    collisionDetector.addObstacle(vertex,faces,faceNormals);
end
%% RMRC Circle Path
% Create a path generator for the dobot model using the specified mask
pathGenerator = PathGenerator(dobot.model, mask);

% Define circle parameters
center = [2 -1.24 0.6]'; % [x, y, z] - Center of the circle, z-coordinate specifies the height
radius = 0.02; % Radius of the circle

%intermediate steps between waypoints to generate a trajectory
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

% Generate a RMRC trajectory for the circular path
circleQMatrix = pathGenerator.getWaypointRMRC(waypoints, steps_per_waypoint, deltaT);

%% Lift Up RMRC
% Define waypoints for lifting the dobot
x1 = [center(1) - radius center(2) center(3)  0]';
x2 = [center(1) - radius center(2) center(3) + 0.04 0]';

pointSteps = 30;

% Generate a RMRC trajectory for lifting the dobot
liftQMatrix = pathGenerator.getPointRMRC(x1, x2, pointSteps, deltaT);

%% Define waypoints for moving the dobot away
x3 = x2 + [0.1, -0.06 0 0]';

% Generate a RMRC trajectory for moving the dobot away
moveQMatrix = pathGenerator.getPointRMRC(x2, x3, pointSteps, deltaT);

%% IRB Waypoints
%Position Waypoints

IRBPositions = [
    1.7 -1.227 0.64;
    1.82 -1.227 0.64;
    1.6 -1.227 0.64;
    0.795 -1.5 1.1
    0.795 -1.81 0.8287;
    0.795 -1.5 1.1
    ];

% Joint Angle Waypoints for IRB robot
IRBAngles = [
    -0.2967    1.4451   -0.3770         0   -0.7854         0;
    -0.2967    1.4451   -0.3770         0   -0.7854         0;
    -0.3    0.8   -0.05        0         0         0;
    -0.2546   0.6178   -0.1599         0         0         0;   
    -2   1.1   -0.5         0         0         0;
    -0.2546   0.6178   -0.1599         0         0         0;   
    ];

%% Gripper

% Define gripper open and closed states
gripperOpen = [0,0];
gripperClosed = [0, pi/8];

% Initialise each finger in the gripper
gripper = IRBGripper(IRB.model, 0.1, gripperOpen, gripperClosed);
gripper.updateGripperPosition(IRB.model, gripperOpen)

%%introducing E Stop
eStop = EStop();

%using signal from arduino to trigger the E-Stop
if useArduino
    listener = ArduinoListener(comPort, 9600, eStop);
end

%% Main loop

stiringRepetions = 1; % Set the number of repetitions for stirring

delay = 0.005;

state = "INIT"; %variable which determines the state

%various counters and indices for tracking repetitions, waypoints, steps, IRB positions, and IRB movement steps
currentRep = 1;
currentPoint = 1;
currentStep = 1;
currentIRBPos = 1;
currentIRBStep = 1;

while ~strcmp(state, 'FINISHED') %Continue until the state is finished

    if useArduino
        listener.checkForData(); % Check for data from Arduino if enabled
    end

    
    paused = eStop.getEStopState(); % Check the emergency stop state to see if its okay to continue or in stop state

    % Get the current joint angles to detect for collisions
    if useCollisions && ~paused
        q = IRB.model.getpos(); 
        collision = collisionDetector.detectCollision(q); 

        % Display a collision detection message and trigger the e stop
        if collision
            disp('Collision Detected')
            eStop.triggerEStop();
        end

    end
    % Continue to the next iteration if paused
    if paused
        drawnow();
        %disp('paused')
        continue;
    end

    switch state
        case "INIT"
            % Initialisation, set up initial conditions

            state = "STIR_BOWL"; %transition states to begin the stirring process

        %Execute RMRC for the Dobot in a circular path while updating the visualisation, ensuring if there are more repetitions, waypoints, and steps to perform within the circular path
        case "STIR_BOWL"
            if currentRep <= stiringRepetions
                if currentPoint < length(waypoints)
                    if currentStep <= steps_per_waypoint
                        dobot.model.animate(circleQMatrix(currentStep, :, currentPoint)); %Move the actual Dobot
                        drawnow();
                        currentStep = currentStep + 1;
                        
                    % Reset the step counter and move to the next waypoint
                    else
                        currentStep = 1;
                        currentPoint = currentPoint + 1;
                    end
                else
                    %reset the waypoint index and move to the next repitition
                    currentPoint = 1;
                    currentRep = currentRep + 1;
                end

                %when the steps to this stirring process are complete, the state moves on to lifting the dobot arm
                if currentRep > stiringRepetions
                    state = "LIFT_ARM";
                    delay = 0.01;
                end

            end
            
        %Elevate the Dobots arm out of the bowl through a sequence of steps , updating the visualisation as it progresses
        case "LIFT_ARM"
            if currentStep <= pointSteps
                dobot.model.animate(liftQMatrix(currentStep,:));
                drawnow();
                currentStep = currentStep + 1;
            else
                %the step counter is now reset and the states change
                currentStep = 1;
                state = "MOVE_DOBOT";
            end
        %move the Dobot accordingly, for the IRB robot to come in to pick the bowl up without collision 
        case "MOVE_DOBOT"
            if currentStep <= pointSteps
                dobot.model.animate(moveQMatrix(currentStep,:));
                drawnow();
                currentStep = currentStep + 1;
            else
                %the steps once again reset and the states move along
                currentStep = 1;
                state = "IRB_MOVEMENT";
            end

        case "IRB_MOVEMENT"
            if currentIRBPos <= length(IRBPositions) % Check if there are more IRB movement positions

                if ~exist('irbSubState', 'var') || strcmp(irbSubState, 'NEW_POSITION')
                    qCurrent = IRB.model.getpos();  % Get the current joint angles of the IRB
                    goal = transl(IRBPositions(currentIRBPos,:)) * troty(pi/2); %goal position
                    qGoal = IRB.model.ikine(goal, IRBAngles(currentIRBPos,:), 'mask', mask); % Calculate joint angles for the goal
                    qMatrix = jtraj(qCurrent, qGoal, pointSteps); % Generate a joint angle trajectory

                    switch currentIRBPos
                        case 1
                            gripperState = gripperOpen; % Set the gripper state to open
                        case 3
                            gripper.closeHand();
                            gripperState = gripperClosed; % Set the gripper state to closed
                        case 6
                            gripper.openHand(); % Open the gripper
                            gripperState = gripperOpen;
                    end

                    irbSubState = 'RUNNING_STEPS'; %moving states for IRB to continue its other movements
                end

                if strcmp(irbSubState, 'RUNNING_STEPS')
                    if currentIRBStep <= pointSteps %are there more IRB steps to carry out
                        IRB.model.animate(qMatrix(currentIRBStep,:)); %move the IRB 
                        gripper.updateGripperPosition(IRB.model, gripperState); %update the gripper position 

                        if (currentIRBPos > 2 && currentIRBPos < 6)
                            T = getPos(IRB.model); % Get the IRB's current position
                            environment.updateObjectPosition(bowl, bowl_verts, T); %making sure to update the bowls postition 
                        end

                        drawnow(); 
                        currentIRBStep = currentIRBStep + 1;
                    else
                        currentIRBStep = 1; 
                        currentIRBPos = currentIRBPos + 1;
                        irbSubState = 'NEW_POSITION'; %new state is reached
                    end
                end

            else
                % Transition to the "FINISHED" state, once all the tasks have finished
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

%calculates the position and orientation of an end effector based on the robot, which has been used with offsets to make the end effector perfectly reach the bowl
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



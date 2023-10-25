close all;
clear all;

%% Global Variables
mask = [1 1 1 0 0 0]; % Consider x, y, and z translations and Y rotations


%% Workspace Setup

environment = EnvironmentSetup;

[bowl, bowl_verts] = environment.placeBowl();

dobot = environment.placeDobot();
IRB = environment.placeIRB12009();

dobot.model.delay = 0;
IRB.model.delay = 0;

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
x1 = [center(1) + radius center(2) center(3)  0]';
x2 = [center(1) + radius center(2) center(3) + 0.05 0]';

pointSteps = 30;

liftQMatrix = pathGenerator.getPointRMRC(x1, x2, pointSteps, deltaT);

%% Move Away Dobot
x3 = x2 + [0.1, -0.06 0 0]';

moveQMatrix = pathGenerator.getPointRMRC(x2, x3, pointSteps, deltaT);

%% IRB Waypoints
%Position Waypoints

IRBPositions = [
    1.843 -1.227 0.5623;
    1.8067 -1.2161 0.8802;
    0.795 -1.81 0.8287;
    0.3578 -1.1898 0.5604;
    0.56, -1.181, 0.4834;
    0.609, -1.164, 0.7;
    ];


IRBAngles = [
    -0.2967    1.4451   -0.3770         0   -0.7854         0
    -0.3    0.8   -0.05        0         0         0;
    -2   1.1   -0.5         0         0         0;
    -2.8   1.5   -0.4    0.1885   -1.1525         0;
    -2.7925    1.5010    0.0698    0.1745   -1.5603         0;
    -2.7925    0.9791    0.5411    0.1745   -1.5603         0

    ];

%% Gripper

gripperOpen = [0,0];
gripperClosed = [0, pi/16];

% Init each finger in the gripper
gripper = IRBGripper(IRB.model, 0.1, gripperOpen, gripperClosed);
gripper.updateGripperPosition(IRB.model, gripperOpen)


%% Movements
%{
%Stir Bowl
stiringRepetions = 2;
for rep = 1:stiringRepetions
    for point = 1:length(waypoints)-1
        for i = 1:steps_per_waypoint
            dobot.model.animate(circleQMatrix(i, :, point));    
            pause(0.005);
            drawnow();
        end
    end
end

% Lift arm up
for i = 1:pointSteps
    dobot.model.animate(liftQMatrix(i,:));
    pause(0.05);
    drawnow();
end

% Move dobot out of the way
for i = 1:pointSteps
    dobot.model.animate(moveQMatrix(i,:));
    pause(0.05);
    drawnow();
end

%}

% IRB Movement loop
for i = 1:length(IRBPositions)

    qCurrent = IRB.model.getpos();

    goal = transl(IRBPositions(i,:)) * troty(pi/2);
    qGoal = IRB.model.ikine(goal, IRBAngles(i,:), 'mask', [1 1 1 0 0 0]);

    qMatrix = jtraj(qCurrent, qGoal, pointSteps);

    
    switch i
        case 1
            gripperState = gripperOpen;
        case 2
            gripper.closeHand();
            gripperState = gripperClosed;
        case 5
            gripper.openHand();
            gripperState = gripperOpen;
    end
    
    for j = 1:pointSteps
        IRB.model.animate(qMatrix(j,:));
        gripper.updateGripperPosition(IRB.model, gripperState);

        pause(0.05);

        if (i > 1 && i  < 5)
            T = getPos(IRB.model);
            environment.updateObjectPosition(bowl, bowl_verts, T)
        end

        drawnow();
    end


end

gripper.openHand();

function position = getPos(robotModel)
    % Compute the end effector's position and rotation
    endEffectorTransform = robotModel.fkine(robotModel.getpos());
    endEffectorPos = endEffectorTransform.t;
    endEffectorRot = endEffectorTransform.R;
    
    % Extract the z-axis direction from the rotation matrix
    zDirection = endEffectorRot(:, 3);
    
    % Compute the offset position
    offsetPos = endEffectorPos + (0.1 * zDirection);

    relativeRotation = eye(3);
    
    % Create a transformation matrix for the offset position and rotation
    position = [relativeRotation, offsetPos; 0, 0, 0, 1];
end





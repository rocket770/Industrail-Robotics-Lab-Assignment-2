close all;
clear all;

dobot = DobotMagician;
robot = dobot.model;

M = [1 1 1 1 0 0]; % Consider x, y, and z translations

path = PathGenerator(robot, M);


% Circle parameters
center = [0.2 0.1  0.05]'; % [x, y, z] - Center of the circle, z-coordinate specifies the height
radius = 0.1; % Radius of the circle
angle = linspace(0, 2*pi, 100); % Generating 100 points along the circle for more accuracy
steps = length(angle);
robot.delay = 0;

x = zeros(3,steps); 

% Generating the circular trajectory in x-y plane with specified height
for i = 1:steps
    x(1,i) = center(1) + radius * cos(angle(i));
    x(2,i) = center(2) + radius * sin(angle(i));
    x(3,i) = center(3); % Height of the circle
end


[iTraj, RMRC] = path.getRMRC(x, steps, 0.02); 

hold on;

for i = 1:steps
    robot.animate(iTraj(i,:));
    pause(0.01);
    drawnow();
end


for i = 1:steps
    robot.animate(RMRC(i,:));        
    t = robot.fkine(robot.getpos).t;
    plot3(t(1), t(2), t(3), 'r.-');
    pause(0.01);
    drawnow();
end


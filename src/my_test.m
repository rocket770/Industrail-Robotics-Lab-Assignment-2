close all
clear all
% environment = EnvironmentSetup;


% IRB = environment.placeIRB12009();

% IRB.model.delay = 0;

IRBAngles = [
    -0.2967    1.4451   -0.3770         0   -0.7854         0;
    -0.3    0.8   -0.05        0         0         0;
    -0.2546   0.6178   -0.1599         0         0         0;   % THish put this in
    -2   1.1   -0.5         0         0         0;
    -0.2546   0.6178   -0.1599         0         0         0;   % THish put this in
    ];

pointSteps = 20;

 for i = 1:5   
    % IRB.model.fkine(IRBAngles(i,:))
    qCurrent = IRB.model.getpos();
    % goal = transl(IRBPositions(currentIRBPos,:)) * troty(pi/2);
    qGoal = IRBAngles(i,:)
    qMatrix = jtraj(qCurrent, qGoal, pointSteps);
    for j = 1:size(qMatrix,1)
    IRB.model.animate(qMatrix(j,:));
    end
    Collision_detection_2.detect_collision(IRB, IRBAngles)
 end


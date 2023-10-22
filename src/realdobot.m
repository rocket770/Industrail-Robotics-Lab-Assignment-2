%rosinit('192.168.27.1')

steps = 20;

x = zeros(1,steps);
y = zeros(1,steps);

radius = 0.05;

anlges = deg2rad(linspace(0,360,steps));

center = [0.2, 0, 0];

for i = 1:steps
    x(i) = cos(anlges(i)) * radius + center(1);
    y(i) = sin(anlges(i)) * radius + center(2);

end

% plot(x,y)

[targetEndEffectorPub,targetEndEffectorMsg] = rospublisher('/dobot_magician/target_end_effector_pose');

%Get feedback about where the end effector pose is currently
endEffectorSub = rossubscriber('/dobot_magician/current_end_effector_pose');   

endEffectorRotation = [0,0,0];
qua = eul2quat(endEffectorRotation);

        % targetEndEffectorMsg.Position.X = 0.3; 
        % targetEndEffectorMsg.Position.Y = 0;
        % targetEndEffectorMsg.Position.Z = 0;
        % 
        % send(targetEndEffectorPub,targetEndEffectorMsg)

for i = 1:5
    for i = 1:steps

        targetEndEffectorMsg.Position.X = x(i); 
        targetEndEffectorMsg.Position.Y = y(i);
        targetEndEffectorMsg.Position.Z = 0;

        qua = eul2quat(endEffectorRotation);
        targetEndEffectorMsg.Orientation.W = qua(1);
        targetEndEffectorMsg.Orientation.X = qua(2);
        targetEndEffectorMsg.Orientation.Y = qua(3);
        targetEndEffectorMsg.Orientation.Z = qua(4);


        send(targetEndEffectorPub,targetEndEffectorMsg);
        % pause(0.5)

       
    end
end

        % targetEndEffectorMsg.Position.X = 0.3; 
        % targetEndEffectorMsg.Position.Y = 0;
        % targetEndEffectorMsg.Position.Z = 0;
        % 
        % send(targetEndEffectorPub,targetEndEffectorMsg)
%rosinit('192.168.27.1')
%number of points on the circular path
steps = 20;
%arrays of zeros with a length of 'steps'
x = zeros(1,steps);
y = zeros(1,steps);

%representing the radius of the circular path
radius = 0.05;

%distributing the steps to make up a full circle by evenly dividing it
anlges = deg2rad(linspace(0,360,steps));

%center of the circular path
center = [0.2, 0, 0];

%trig is used to create the circular path, the calculated x and y are centered around a point
for i = 1:steps
    x(i) = cos(anlges(i)) * radius + center(1);
    y(i) = sin(anlges(i)) * radius + center(2);

end


% plot(x,y)

[targetEndEffectorPub,targetEndEffectorMsg] = rospublisher('/dobot_magician/target_end_effector_pose');

%Get feedback about where the end effector pose is currently
% endEffectorSub = rossubscriber('/dobot_magician/current_end_effector_pose');   

%initial end effector position - no rotation 
endEffectorRotation = [0,0,0];
% qua = eul2quat(endEffectorRotation);

        % targetEndEffectorMsg.Position.X = 0.3; 
        % targetEndEffectorMsg.Position.Y = 0;
        % targetEndEffectorMsg.Position.Z = 0;
        % 
        % send(targetEndEffectorPub,targetEndEffectorMsg)
        
        %The initial pose of the end effector is set 
        targetEndEffectorMsg.Position.X = 0.3; 
        targetEndEffectorMsg.Position.Y = 0;
        targetEndEffectorMsg.Position.Z = 0.05;

        %The orientation is calculated as a quaternion and then assigned to the message
        qua = eul2quat(endEffectorRotation);
        targetEndEffectorMsg.Orientation.W = qua(1);
        targetEndEffectorMsg.Orientation.X = qua(2);
        targetEndEffectorMsg.Orientation.Y = qua(3);
        targetEndEffectorMsg.Orientation.Z = qua(4);


        send(targetEndEffectorPub,targetEndEffectorMsg);
        pause(0.5) %time for processing

%2 nested loops are used to create the circular motion
for j = 1:3 %iterates 3 times 
    for i = 1:steps 
        % this iterates through each point on the circular path and updates the end effector pose using x(i) and y(i) from the circular path calculation
        targetEndEffectorMsg.Position.X = x(i); 
        targetEndEffectorMsg.Position.Y = y(i   
        targetEndEffectorMsg.Position.Z = 0;

        %orientation remains constant throughout this process
        qua = eul2quat(endEffectorRotation);
        targetEndEffectorMsg.Orientation.W = qua(1);
        targetEndEffectorMsg.Orientation.X = qua(2);
        targetEndEffectorMsg.Orientation.Y = qua(3);
        targetEndEffectorMsg.Orientation.Z = qua(4);

        %sends the new end-effector pose to the ROS system 
        send(targetEndEffectorPub,targetEndEffectorMsg);
        pause(0.5)
        % disp("rotation")
       
    end
end
        %sets the end effector pose back to the initial pose and sends it to the ROS system
        targetEndEffectorMsg.Position.X = 0.3; 
        targetEndEffectorMsg.Position.Y = 0;
        targetEndEffectorMsg.Position.Z = 0.05;

        qua = eul2quat(endEffectorRotation);
        targetEndEffectorMsg.Orientation.W = qua(1);
        targetEndEffectorMsg.Orientation.X = qua(2);
        targetEndEffectorMsg.Orientation.Y = qua(3);
        targetEndEffectorMsg.Orientation.Z = qua(4);


        send(targetEndEffectorPub,targetEndEffectorMsg);
        pause(0.5)

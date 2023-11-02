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


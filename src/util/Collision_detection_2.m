% Note: Not finished yet

% Work to be done here:

% Do 8 lines similar to the links around the links
% Test it with the UR3 from another class

%% 
% function [  ] = collision_detection_2( )

% clf
close all;
clc;

2.1: Make a 3DOF model
L1 = Link('d',0,'a',1,'alpha',0,'qlim',[-pi pi]);
L2 = Link('d',0,'a',1,'alpha',0,'qlim',[-pi pi]);
L3 = Link('d',0,'a',1,'alpha',0,'qlim',[-pi pi]);       
robot = SerialLink([L1 L2 L3],'name','myRobot');                     
q = zeros(1,3);                                                     % Create a vector of initial joint angles        
scale = 0.5;
workspace = [-2 2 -2 2 -0.05 2];                                       % Set the size of the workspace when drawing the robot
robot.plot(q,'workspace',workspace,'scale',scale);                  % Plot the robot

% Random rectangular prism placed. Place an invisible rectangle 
centerpnt = [2,0,-0.5];
side = 1.5;
plotOptions.plotFaces = true;
[vertex,faces,faceNormals] = RectangularPrism(centerpnt-side/2, centerpnt+side/2,plotOptions);
axis equal
camlight
robot.teach;



 Go through until there are no step sizes larger than 1 degree
q1 = [-pi/4,0,0];
q2 = [pi/4,0,0];
steps = 2;
while ~isempty(find(1 < abs(diff(rad2deg(jtraj(q1,q2,steps)))),1))
    steps = steps + 1;
end
qMatrix = jtraj(q1,q2,steps);

Stepping through each of the wrote down joint states
Stop when there is a collision







result = true(steps,1);
for i = 1: steps
    result(i) = IsCollision(robot,qMatrix(i,:),faces,vertex,faceNormals,false);
    if result(i) == true
        disp("first collision. Collision detected, robot stopped");
        break;
    end
    robot.animate(qMatrix(i,:));
end




% end

%% IsIntersectionPointInsideTriangle
% Given a point which is known to be on the same plane as the triangle
% determine if the point is 
% inside (result == 1) or 
% outside a triangle (result ==0 )
function result = IsIntersectionPointInsideTriangle(intersectP,triangleVerts)

u = triangleVerts(2,:) - triangleVerts(1,:);
v = triangleVerts(3,:) - triangleVerts(1,:);

uu = dot(u,u);
uv = dot(u,v);
vv = dot(v,v);

w = intersectP - triangleVerts(1,:);
wu = dot(w,u);
wv = dot(w,v);

D = uv * uv - uu * vv;

% Get and test parametric coords (s and t)
s = (uv * wv - vv * wu) / D;
if (s < 0.0 || s > 1.0)        % intersectP is outside Triangle
    result = 0;
    return;
end

t = (uv * wu - uu * wv) / D;
if (t < 0.0 || (s + t) > 1.0)  % intersectP is outside Triangle
    result = 0;
    return;
end

result = 1;                      % intersectP is in Triangle
end

%% IsCollision
% This is based upon the output of questions 2.5 and 2.6
% Given a robot model (robot), and trajectory (i.e. joint state vector) (qMatrix)
% and triangle obstacles in the environment (faces,vertex,faceNormals)


function [result] = IsCollision(robot,qMatrix,faces,vertex,faceNormals,returnOnceFound)



if nargin < 6
    returnOnceFound = true;
end
result = false;

% This one is the offset distance
offsetValue = 0.1;

for qIndex = 1:size(qMatrix,1)
    % Get the transform of every joint (i.e. start and end of every link)
    tr1 = GetLinkPoses(qMatrix(qIndex,:), robot) + transl(0,-offsetValue,0) ;
    tr2 = GetLinkPoses(qMatrix(qIndex,:), robot) + transl(-offsetValue,-offsetValue,0) ;
    tr3 = GetLinkPoses(qMatrix(qIndex,:), robot) + transl(0,offsetValue,0) ;
    tr4 = GetLinkPoses(qMatrix(qIndex,:), robot) + transl(offsetValue,offsetValue,0) ;
    tr5 = GetLinkPoses(qMatrix(qIndex,:), robot) + transl(-offsetValue,offsetValue,0) ;
    tr6 = GetLinkPoses(qMatrix(qIndex,:), robot) + transl(offsetValue,-offsetValue,0) ;
    tr7 = GetLinkPoses(qMatrix(qIndex,:), robot) + transl(offsetValue,0,0) ;
    tr8 = GetLinkPoses(qMatrix(qIndex,:), robot) + transl(-offsetValue,0,0) ;
    % disp(tr)

    % Go through each link and also each triangle face
    for i = 1 : size(tr1,3)-1    
        for faceIndex = 1:size(faces,1)
            vertOnPlane = vertex(faces(faceIndex,1)',:);
            [intersectP1,check1] = LinePlaneIntersection(faceNormals(faceIndex,:),vertOnPlane,tr1(1:3,4,i)',tr1(1:3,4,i+1)'); 
            [intersectP2,check2] = LinePlaneIntersection(faceNormals(faceIndex,:),vertOnPlane,tr2(1:3,4,i)',tr2(1:3,4,i+1)'); 
            [intersectP3,check3] = LinePlaneIntersection(faceNormals(faceIndex,:),vertOnPlane,tr3(1:3,4,i)',tr3(1:3,4,i+1)'); 
            [intersectP4,check4] = LinePlaneIntersection(faceNormals(faceIndex,:),vertOnPlane,tr4(1:3,4,i)',tr4(1:3,4,i+1)'); 
            [intersectP5,check5] = LinePlaneIntersection(faceNormals(faceIndex,:),vertOnPlane,tr5(1:3,4,i)',tr5(1:3,4,i+1)'); 
            [intersectP6,check6] = LinePlaneIntersection(faceNormals(faceIndex,:),vertOnPlane,tr6(1:3,4,i)',tr6(1:3,4,i+1)'); 
            [intersectP7,check7] = LinePlaneIntersection(faceNormals(faceIndex,:),vertOnPlane,tr7(1:3,4,i)',tr7(1:3,4,i+1)'); 
            [intersectP8,check8] = LinePlaneIntersection(faceNormals(faceIndex,:),vertOnPlane,tr8(1:3,4,i)',tr8(1:3,4,i+1)'); 

            check = 0;

            if check1  || check2 || check3 || check4 || check5 || check6 || check7 || check8
                check = 1;
            end

            int1 = IsIntersectionPointInsideTriangle(intersectP1,vertex(faces(faceIndex,:)',:));
            int2 = IsIntersectionPointInsideTriangle(intersectP2,vertex(faces(faceIndex,:)',:));
            int3 = IsIntersectionPointInsideTriangle(intersectP3,vertex(faces(faceIndex,:)',:));
            int4 = IsIntersectionPointInsideTriangle(intersectP4,vertex(faces(faceIndex,:)',:));
            int5 = IsIntersectionPointInsideTriangle(intersectP5,vertex(faces(faceIndex,:)',:));
            int6 = IsIntersectionPointInsideTriangle(intersectP6,vertex(faces(faceIndex,:)',:));
            int7 = IsIntersectionPointInsideTriangle(intersectP7,vertex(faces(faceIndex,:)',:));
            int8 = IsIntersectionPointInsideTriangle(intersectP8,vertex(faces(faceIndex,:)',:));


            if check == 1 && (int1 || int2 || int3 || int4 || int5 || int6 || int7 || int8)

                % Use the following to plot the point where the intersection happens
                % You can individually change the numbers to see which line
                % is intersecting with the rectangular prism

                % if check2 && my2
                % plot3(intersectP2(1),intersectP2(2),intersectP2(3),'r*');
                % end

                result = true;


                if returnOnceFound
                    return
                end
            end
        end    
    end
end
end




























% function [result, my_tr] = IsCollision(robot,qMatrix,faces,vertex,faceNormals,returnOnceFound)
% h = 0;
% % my_tr = zeros(1);
% 
% if nargin < 6
%     returnOnceFound = true;
% end
% result = false;
% 
% for qIndex = 1:size(qMatrix,1)
%     % Get the transform of every joint (i.e. start and end of every link)
%     tr = GetLinkPoses(qMatrix(qIndex,:), robot);
% 
%     % Go through each link and also each triangle face
%     for i = 1 : size(tr,3)-1    
%         for faceIndex = 1:size(faces,1)
%             vertOnPlane = vertex(faces(faceIndex,1)',:);
%             [intersectP,check] = LinePlaneIntersection(faceNormals(faceIndex,:),vertOnPlane,tr(1:3,4,i)',tr(1:3,4,i+1)'); 
%             if check == 1 && IsIntersectionPointInsideTriangle(intersectP,vertex(faces(faceIndex,:)',:))
%                 plot3(intersectP(1),intersectP(2),intersectP(3),'g*');
%                 % disp('Intersection');
%                 result = true;
% 
%                 % if h == 0
%                 %     my_tr(1) = qIndex;
%                 %     disp(my_tr)
%                 % end
%                 % h = h + 1;
% 
%             % else
%             %     disp("no col")
%                 if returnOnceFound
%                     return
%                 end
%             end
%         end    
%     end
% end
% end

%% GetLinkPoses
% q - robot joint angles
% robot -  seriallink robot model
% transforms - list of transforms
function [ transforms ] = GetLinkPoses( q, robot)

links = robot.links;
transforms = zeros(4, 4, length(links) + 1);
transforms(:,:,1) = robot.base;

for i = 1:length(links)
    L = links(1,i);

    current_transform = transforms(:,:, i);

    current_transform = current_transform * trotz(q(1,i) + L.offset) * ...
    transl(0,0, L.d) * transl(L.a,0,0) * trotx(L.alpha);
    transforms(:,:,i + 1) = current_transform;
end
end




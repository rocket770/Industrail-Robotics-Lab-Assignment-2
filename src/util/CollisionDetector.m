classdef CollisionDetector < handle
    properties
        robot
        obstacles % Struct array to hold obstacle data
    end

    methods
        function obj = CollisionDetector(robot)
            obj.robot = robot;
            obj.obstacles = struct('vertex', {}, 'faces', {}, 'faceNormals', {}); % Initialize as an empty struct array

        end

        function obj = addObstacle(obj, vertex, faces, faceNormals)
            % Create a struct to hold the obstacle data
            obstacleData.vertex = vertex;
            obstacleData.faces = faces;
            obstacleData.faceNormals = faceNormals;

            % Add the new obstacle data struct to the obstacles array
            if isempty(obj.obstacles)
                obj.obstacles = obstacleData; % Initialize if empty
            else
                obj.obstacles(end + 1) = obstacleData; % Append otherwise
            end
        end

        function result = detectCollision(obj, q)
            result = false;
            for i = 1:length(obj.obstacles)
                vertex = obj.obstacles(i).vertex;
                faces = obj.obstacles(i).faces;
                faceNormals = obj.obstacles(i).faceNormals;

                result = obj.IsCollision(obj.robot,q,faces, vertex, faceNormals, true);

                if result == true
                    break
                end
            end

        end


        %% IsCollision
        % This is based upon the output of questions 2.5 and 2.6
        % Given a robot model (robot), and trajectory (i.e. joint state vector) (qMatrix)
        % and triangle obstacles in the environment (faces,vertex,faceNormals)
        function result = IsCollision(obj, robot,qMatrix,faces,vertex,faceNormals,returnOnceFound)



            if nargin < 6
                returnOnceFound = true;
            end
            result = false;

            % This one is the offset distance
            offsetValue = 0.1;

            for qIndex = 1:size(qMatrix,1)
                % Get the transform of every joint (i.e. start and end of every link)
                tr1 = obj.GetLinkPoses(qMatrix(qIndex,:), robot) + transl(0,-offsetValue,0) ;
                tr2 = obj.GetLinkPoses(qMatrix(qIndex,:), robot) + transl(-offsetValue,-offsetValue,0) ;
                tr3 = obj.GetLinkPoses(qMatrix(qIndex,:), robot) + transl(0,offsetValue,0) ;
                tr4 = obj.GetLinkPoses(qMatrix(qIndex,:), robot) + transl(offsetValue,offsetValue,0) ;
                tr5 = obj.GetLinkPoses(qMatrix(qIndex,:), robot) + transl(-offsetValue,offsetValue,0) ;
                tr6 = obj.GetLinkPoses(qMatrix(qIndex,:), robot) + transl(offsetValue,-offsetValue,0) ;
                tr7 = obj.GetLinkPoses(qMatrix(qIndex,:), robot) + transl(offsetValue,0,0) ;
                tr8 =obj. GetLinkPoses(qMatrix(qIndex,:), robot) + transl(-offsetValue,0,0) ;
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

                        int1 = obj.IsIntersectionPointInsideTriangle(intersectP1,vertex(faces(faceIndex,:)',:));
                        int2 = obj.IsIntersectionPointInsideTriangle(intersectP2,vertex(faces(faceIndex,:)',:));
                        int3 = obj.IsIntersectionPointInsideTriangle(intersectP3,vertex(faces(faceIndex,:)',:));
                        int4 = obj.IsIntersectionPointInsideTriangle(intersectP4,vertex(faces(faceIndex,:)',:));
                        int5 = obj.IsIntersectionPointInsideTriangle(intersectP5,vertex(faces(faceIndex,:)',:));
                        int6 = obj.IsIntersectionPointInsideTriangle(intersectP6,vertex(faces(faceIndex,:)',:));
                        int7 = obj.IsIntersectionPointInsideTriangle(intersectP7,vertex(faces(faceIndex,:)',:));
                        int8 = obj.IsIntersectionPointInsideTriangle(intersectP8,vertex(faces(faceIndex,:)',:));


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



        %% IsIntersectionPointInsideTriangle
        % Given a point which is known to be on the same plane as the triangle
        % determine if the point is
        % inside (result == 1) or
        % outside a triangle (result ==0 )
        function result = IsIntersectionPointInsideTriangle(~, intersectP,triangleVerts)

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
            if (s < 0.0 || s > 1.0) % intersectP is outside Triangle
                result = 0;
                return;
            end

            t = (uv * wu - uu * wv) / D;
            if (t < 0.0 || (s + t) > 1.0)  % intersectP is outside Triangle
                result = 0;
                return;
            end

            result = 1; % intersectP is in Triangle
        end

        %% GetLinkPoses
        % q - robot joint angles
        % robot -  seriallink robot model
        % transforms - list of transforms
        function transforms = GetLinkPoses(~, q, robot)

            links = robot.links;
            transforms = zeros(4, 4, length(links) + 1);
            transforms(:,:,1) = robot.base;

            for i = 1:length(links)
                L = links(1,i);
                current_transform = transforms(:,:, i);
                current_transform = current_transform * trotz(q(1,i) + L.offset) * transl(0,0, L.d) * transl(L.a,0,0) * trotx(L.alpha);
                transforms(:,:,i + 1) = current_transform;
            end
        end
    end
end

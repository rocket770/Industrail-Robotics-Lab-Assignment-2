classdef AdvancedTeach < handle
    properties
        fig
        robot
        sliders
        qlim
        valueEdits
        step = 0.01;
        positionLabel
    end

    methods
        function app = AdvancedTeach(robot)
            if nargin < 1
                error('Please provide a robot model.');
            end

            app.robot = robot;
            app.qlim = robot.qlim;
            app.createComponents();
        end

        function createComponents(app)
            app.fig = gcf;
            ax = gca;
            ax.Position(1) = 0.3; % To make sure the axes and panels do not overlap
            app.fig.Position(3) = 820;
            app.fig.Position(4) = 480;
        
            numJoints = app.robot.n;
            app.sliders = gobjects(1, numJoints);
            app.valueEdits = gobjects(1, numJoints);
        
            sliderPanel = uipanel('Title', 'Joint Controls', 'Units', 'normalized', 'Position', [0.005 0.2 0.29 0.8]);
            cartesianPanel = uipanel('Title', 'Cartesian Controls', 'Units', 'normalized', 'Position', [0.005 0.005 0.29 0.4]);
            positionPanel = uipanel('Title', 'Cartesian Position', 'Units', 'normalized', 'Position', [0.005 0.005 0.29 0.13]);
    
            
            sliderWidth = 0.7; % To make sliders wider within the panel
            sliderHeight = 0.1; % To make sliders higher within the panel
            editWidth = 0.15;
            sliderGap = 0.01;
            sliderYPos = 0.97;
        
            textPosLeft = 0.01; 
            textWidth = 0.08; % Width for the static text
        
        
            for i = 1:numJoints
                uicontrol('Parent', sliderPanel, 'Style', 'text', 'String', ['q', num2str(i)], ...
                         'Units', 'normalized', 'Position', [textPosLeft, sliderYPos - i * (sliderHeight + sliderGap), textWidth, sliderHeight], ...
                         'HorizontalAlignment', 'center', 'FontSize', 10); 
        
                % Adjust the position of the slider to match the new static text position and width.
                app.sliders(i) = uicontrol('Parent', sliderPanel, 'Style', 'slider', 'Min', app.qlim(i,1), 'Max', app.qlim(i,2), 'Value', 0, ...
                                          'Units', 'normalized', 'Position', [textPosLeft + textWidth, sliderYPos - i * (sliderHeight + sliderGap), sliderWidth, sliderHeight], ...
                                          'Callback', @(src, event) app.sliderCallback(i));
                                          
                app.valueEdits(i) = uicontrol('Parent', sliderPanel, 'Style', 'edit', 'String', '0', ...
                                             'Units', 'normalized', 'Position', [0.82, sliderYPos - i * (sliderHeight + sliderGap), editWidth, sliderHeight], ...
                                             'Callback', @(src, event) app.textboxCallback(src, i));
            end
                
            btnWidth = 0.45; % Adjusted button width for neat appearance
            btnHeight = 0.19; % Adjusted button height for neat appearance
            btnGap = 0.02; % Gap between buttons
            btnXStart = 0.025; % X Position for first button
            
            % Adjust the position of the buttons
            createButton(app, cartesianPanel, '+X', [btnXStart, 0.78, btnWidth, btnHeight], @(src, event) app.moveInCartesian('x', app.step));
            createButton(app, cartesianPanel, '-X', [btnXStart + btnWidth + btnGap, 0.78, btnWidth, btnHeight], @(src, event) app.moveInCartesian('x', -app.step));

            createButton(app, cartesianPanel, '+Y', [btnXStart, 0.58, btnWidth, btnHeight], @(src, event) app.moveInCartesian('y', app.step));
            createButton(app, cartesianPanel, '-Y', [btnXStart + btnWidth + btnGap, 0.58, btnWidth, btnHeight], @(src, event) app.moveInCartesian('y', -app.step));

            createButton(app, cartesianPanel, '+Z', [btnXStart, 0.38, btnWidth, btnHeight], @(src, event) app.moveInCartesian('z', app.step));
            createButton(app, cartesianPanel, '-Z', [btnXStart + btnWidth + btnGap, 0.38, btnWidth, btnHeight], @(src, event) app.moveInCartesian('z', -app.step));
        
            % Create UI components in positionPanel to display x, y, z coordinates
            app.positionLabel = uicontrol('Parent', positionPanel, 'Style', 'text', 'String', 'X: 0 Y: 0 Z: 0', ...
                'Units', 'normalized', 'Position', [0.1, 0.3, 0.8, 0.3], 'HorizontalAlignment', 'center', 'FontSize', 10);


        end

        function createButton(app, parent, label, position, callback)
            uicontrol('Parent', parent, 'Style', 'pushbutton', 'String', label, 'Units', 'normalized', 'Position', position, 'Callback', callback);
        end


        function updateRobot(app)
            q = zeros(1, app.robot.n);
            for i = 1:app.robot.n
                q(i) = app.sliders(i).Value;
                
                app.valueEdits(i).String = num2str(q(i));
            end
            
            app.robot.animate(q);
            app.updatePositionPanel();
        end

        function textboxCallback(app, src, jointIndex)
            value = str2double(src.String);
            
            % Clamp the value between qlim
            value = max(app.qlim(jointIndex, 1), min(value, app.qlim(jointIndex, 2)));
            
            % Update the slider value
            app.sliders(jointIndex).Value = value;
            
            % Update the robot
            app.updateRobot();
            
            % Update the Edit Text String in case the value was clamped
            src.String = num2str(value);
        end

        function sliderCallback(app, jointIndex)
            value = app.sliders(jointIndex).Value;
            
            % Update the Edit Text String with the slider value
            app.valueEdits(jointIndex).String = num2str(value);
            
            app.updateRobot();
        end
    
        function moveInCartesian(app, direction, step)
            q = app.robot.getpos();
            T = app.robot.fkine(q);
        
            switch direction
                case 'x'
                    T.t(1) = T.t(1) + step;
                case 'y'
                    T.t(2) = T.t(2) + step;
                case 'z'
                    T.t(3) = T.t(3) + step;
            end
        
            try
                qNew = app.robot.ikine(T, q);
            catch
                % Does not work atm 
                warning('The robot is reaching out too far and cannot accurately step in the desired direction alone.\nConsider using  "ikine()"');
                warndlg('The robot is reaching out too far and cannot accurately step in the desired direction alone.\nConsider using  "ikine()"');
            end
            
            if ~isempty(qNew)
                app.robot.animate(qNew);
                for i = 1:app.robot.n
                    app.sliders(i).Value = qNew(i);
                    app.valueEdits(i).String = num2str(qNew(i));
                end
                app.updatePositionPanel();
            end
        end

        function modifyStep(app, src)
            app.step = str2double(src.String);
            if app.step <= 0
                app.step = 0.01; % Default value or a minimal positive value
                src.String = num2str(app.step);
            end
        end

        function updatePositionPanel(app)
            % Get the current position of the robot
            q = app.robot.getpos();
            T = app.robot.fkine(q);
            
            % Update the positionLabel with the new Cartesian position
            app.positionLabel.String = sprintf('X: %.2f Y: %.2f Z: %.2f', T.t(1), T.t(2), T.t(3));
        end


        function controllerCallback(app, ~, ~)
            threshold = 0.1;
            [axes, buttons, ~] = read(app.joy);

            disp(axes);

            % Add deadzone, abs threshold = 0.1
            for i = 1:length(axes)
                if abs(axes(i)) < threshold
                    axes(i) = 0;
                end
            end

            dt = 0.15;
            app.joyQ = app.robot.getpos();
            Kv = 0.3; % linear velocity gain
            Kw = 0.8; % angular velocity gain
            vx = Kv * axes(1);
            vy = Kv * axes(2);
            vz = Kv * (buttons(5) - buttons(7));
            wx = Kw * axes(4);
            wy = Kw * axes(3);
            wz = Kw * (buttons(6) - buttons(8));
            dx = [vx; vy; vz; wx; wy; wz]; % combined velocity vector



            % Use DLS J inverse to calculate joint velocity
            lambda = 0.5;
            J = app.robot.jacob0(app.joyQ);
            Jinv_dls = inv((J'*J)+lambda^2*eye(6))*J';
            dq = Jinv_dls*dx;

            % Apply joint velocity to step robot joint angles
            q = app.joyQ + dq'*dt;
            
            inJointLimits = true;
            % Check if the next movement is within qlim, by looking at
            % every joint before it is applied
            for i = 1:app.robot.n
                if q(i) < app.qlim(i, 1) || q(i) > app.qlim(i, 2)
                    inJointLimits = false;
                end
            end

            if inJointLimits
                app.joyQ = q;
                app.updateRobotPosition(app.joyQ);
            end

        end

        function closeRequestFcn(app, ~, ~)
            
            % Stop and delete the joystick timer
            stop(app.joyTimer);
            delete(app.joyTimer);

            delete(app.fig);
        end

    end
end

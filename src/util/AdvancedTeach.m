classdef RobotTeach < handle
    properties
        fig
        robot
        sliders
        qlim
        valueEdits
    end

    methods
        function app = RobotTeach(robot)
            if nargin < 1
                error('Please provide a robot model.');
            end

            app.robot = robot;
            app.qlim = robot.qlim;
            app.createComponents();
        end

        function createComponents(app)
            app.fig = gcf; % get existing figure

            % Adjust the position of the existing axes
            ax = gca; % Get the handle to the current axes
            ax.Position(1) = 0.2; % shif the plot over 0.2 units to the right
            app.fig.Position(3) = 820; % Set window width
            app.fig.Position(4) = 480; % Set window height

            numJoints = app.robot.n;

            app.sliders = gobjects(1, numJoints);
            app.valueEdits = gobjects(1, numJoints); % This line is crucial.



            sliderWidth = 0.15; % Adjust the width of the sliders
            sliderHeight = 0.05; % Adjust the height of the sliders
            textWidth = 0.05; % Width of the Static and Edit Text
            editWidth = 0.05; % Width of the Edit Text
            sliderGap = 0.003;
            sliderYPos = 0.95;

            for i = 1:numJoints
                % Static Text to the left of the slider
                uicontrol('Style', 'text', 'String', ['q', num2str(i)],...
                    'Units', 'normalized', 'Position', [0.005, (sliderYPos-0.01) - i * (sliderHeight + sliderGap), textWidth, sliderHeight],...
                    'HorizontalAlignment', 'right');
    
                % Slider
                app.sliders(i) = uicontrol('Style', 'slider', 'Min', app.qlim(i,1), 'Max', app.qlim(i,2), 'Value', 0,...
                    'Units', 'normalized', 'Position', [0.065, sliderYPos - i * (sliderHeight + sliderGap), sliderWidth, sliderHeight],...
                    'Callback', @(src, event) app.sliderCallback(i));
    
                % Edit Text to the right of the slider
                app.valueEdits(i) = uicontrol('Style', 'edit', 'String', '0', ...
                    'Units', 'normalized', ...
                    'Position', [0.216, sliderYPos - i * (sliderHeight + sliderGap), editWidth, sliderHeight], ...
                    'Callback', @(src, event) app.textboxCallback(src, i));
            end


            uicontrol('Style', 'pushbutton', 'String', 'Move in X',...
                'Units', 'normalized', 'Position', [0.05, 0.05, 0.2, 0.05],...
                'Callback', @(src, event) app.moveInCartesian('x'));

            uicontrol('Style', 'pushbutton', 'String', 'Move in Y',...
                'Units', 'normalized', 'Position', [0.4, 0.05, 0.2, 0.05],...
                'Callback', @(src, event) app.moveInCartesian('y'));

            uicontrol('Style', 'pushbutton', 'String', 'Move in Z',...
                'Units', 'normalized', 'Position', [0.75, 0.05, 0.2, 0.05],...
                'Callback', @(src, event) app.moveInCartesian('z'));
        end

        function updateRobot(app)
            q = zeros(1, app.robot.n);
            for i = 1:app.robot.n
                q(i) = app.sliders(i).Value;
                
                app.valueEdits(i).String = num2str(q(i));
            end
            
            app.robot.animate(q);
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
            
            % You can also call the function to update the robot here, if needed
            app.updateRobot();
        end


        
        function moveInCartesian(app, direction)
            currentPosition = app.robot.fkine(app.robot.getpos());
            goalPosition = currentPosition;

            step = 0.01; % Define your step size for Cartesian movement.
            switch direction
                case 'x'
                    goalPosition.t(1) = goalPosition.t(1) + step;
                case 'y'
                    goalPosition.t(2) = goalPosition.t(2) + step;
                case 'z'
                    goalPosition.t(3) = goalPosition.t(3) + step;
            end

            q = app.robot.ikcon(goalPosition, app.robot.getpos());
            if ~isempty(q)
                app.robot.animate(q);
                for i = 1:app.robot.n
                    app.sliders(i).Value = q(i);
                                app.valueEdits(i).String = num2str(q(i)); % Update the text values

                end
            end
        end
    end
end

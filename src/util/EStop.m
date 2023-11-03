classdef EStop < handle


    properties (Access = private)
        eStopButton % GUI control
        state = 'running'; % Initial state of the system
        f
        listener
    end

    methods
        function obj = EStop()
            % Initialise the class properties and create the GUI
            obj.createGUI();
        end

        % Get the current state of the emergency stop system.
        function state = getEStopState(obj)
            % Return true if the current state is not running
            state = ~strcmp(obj.state, 'running');
        end

        % Trigger the emergency stop
        function triggerEStop(obj)
            if strcmp(obj.state, 'running')
                % If the system is running, set the state to stopped
                obj.state = 'stopped';
                set(obj.eStopButton, 'String', 'E-Stop Active. Press Again to Resume');
            elseif strcmp(obj.state, 'stopped')
                % If the system is stopped, set the state to waiting
                obj.state = 'waiting';
                set(obj.eStopButton, 'String', 'Are You Sure?');
            elseif strcmp(obj.state, 'waiting')
            % If the system is waiting, set the state to running
                obj.state = 'running';
                set(obj.eStopButton, 'String', 'E-Stop');
            end
        end


    end

    methods (Access = private)

        % Set up the graphical user interface for the emergency stop system
        function createGUI(obj)
            obj.f = gcf; % Get current figure (GUI window)

            % Add a listener for keypress events and specify the callback method
            addlistener(obj.f, 'KeyPress', @(src, event) obj.keyPressCallback(src, event));

            % Set a callback for when the figure is resized to update the button's position
            set(obj.f, 'SizeChangedFcn', @(src, event) updateButtonPosition(obj));

            % Create the initial E-Stop button and position the button correctly on the GUI
            createButton(obj);
            updateButtonPosition(obj);
        end

        function createButton(obj)
            %set the width and the height of the buttons
            btnWidth = 200;
            btnHeight = 50;
            
            btnLeft = 20; % Temporary values, will be set correctly in updateButtonPosition
            btnBottom = 20; % Temporary values

            % Create a push button with label E-Stop and set its position and callback
            obj.eStopButton = uicontrol('Style', 'pushbutton', 'String', 'E-Stop',...
                'Position', [btnLeft btnBottom btnWidth btnHeight],...
                'Callback', @(src, event) obj.triggerEStop());
        end

        % Update the position of the E-Stop button to ensure it's correctly positioned on the GUI
        function updateButtonPosition(obj)

            figPos = obj.f.Position; % Get figure position [left, bottom, width, height]
            btnWidth = 200;
            btnHeight = 50;
            btnLeft = figPos(3) - btnWidth - 20; % Position button to the right
            btnBottom = 20; % Position button to the bottom

            % Update the button's position based on the calculated values
            obj.eStopButton.Position = [btnLeft btnBottom btnWidth btnHeight];
        end



        function keyPressCallback(obj, ~, event)
                % If the 's' key is pressed, trigger the emergency stop
                if event.Key == 's'
                    obj.triggerEStop();
                end
                %obj.triggerEStop();
         end

     end
end

classdef EStop < handle


    properties (Access = private)
        eStopButton % GUI control
        state = 'running'; % Initial state of the system
        f
    end

    methods
        function obj = EStop()
            obj.createGUI();
        end

        function state = getEStopState(obj)
            state = ~strcmp(obj.state, 'running');
        end

        function triggerEStop(obj)
            if strcmp(obj.state, 'running')
                obj.state = 'stopped';
                set(obj.eStopButton, 'String', 'E-Stop Active. Press Again to Resume');
            elseif strcmp(obj.state, 'stopped')
                obj.state = 'waiting';
                set(obj.eStopButton, 'String', 'Are You Sure?');
            elseif strcmp(obj.state, 'waiting')
                obj.state = 'running';
                set(obj.eStopButton, 'String', 'E-Stop');
            end
        end


    end

    methods (Access = private)


        function createGUI(obj)
            obj.f = gcf; % Get current figure

            % Listen for any keypress event
            addlistener(obj.f, 'KeyPress', @(src, event) obj.keyPressCallback(src, event));

            % Listen for figure resize event
            set(obj.f, 'SizeChangedFcn', @(src, event) updateButtonPosition(obj));

            % Initial button creation
            createButton(obj);
            updateButtonPosition(obj);
        end

        function createButton(obj)
            btnWidth = 200;
            btnHeight = 50;
            btnLeft = 20; % Temporary values, will be set correctly in updateButtonPosition
            btnBottom = 20; % Temporary values

            % Create button
            obj.eStopButton = uicontrol('Style', 'pushbutton', 'String', 'E-Stop',...
                'Position', [btnLeft btnBottom btnWidth btnHeight],...
                'Callback', @(src, event) obj.triggerEStop());
        end

        function updateButtonPosition(obj)

            figPos = obj.f.Position; % Get figure position [left, bottom, width, height]
            btnWidth = 200;
            btnHeight = 50;
            btnLeft = figPos(3) - btnWidth - 20; % Position button to the right
            btnBottom = 20; % Position button to the bottom

            % Update button's position
            obj.eStopButton.Position = [btnLeft btnBottom btnWidth btnHeight];
        end



        function keyPressCallback(obj, ~, event)
                %if event.Key == 's'
                %    obj.triggerEStop();
                %end
                obj.triggerEStop();
         end

     end
end

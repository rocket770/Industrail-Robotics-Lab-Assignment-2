classdef EStop < handle
    properties (Access = private)
        eStopButton % GUI control
        state = 'running'; % Initial state of the system
    end
    
    methods
        function obj = EStop()
            obj.createGUI();
        end
        
        function createGUI(obj)
            f = gcf; % Get current figure


            addlistener(f, 'KeyPress', @(src, event) obj.keyPressCallback(src, event));


            figPos = f.Position; % Get figure position [left, bottom, width, height]
            btnWidth = 200;
            btnHeight = 50;
            btnLeft = figPos(3) - btnWidth - 20; % Position button to the right
            btnBottom = 20; % Position button to the bottom
            
            obj.eStopButton = uicontrol('Style', 'pushbutton', 'String', 'E-Stop',...
                'Position', [btnLeft btnBottom btnWidth btnHeight],...
                'Callback', @(src, event) obj.onPress());

        end
    end
    
    methods (Access = private)

        
        function keyPressCallback(obj, src, event)
            %if event.Key == 's'
            %    obj.onPress();
            %end
            obj.onPress();
        end

        function onPress(obj)
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
    
    methods
        function state = getEStopState(obj)
            state = ~strcmp(obj.state, 'running');
        end
    end
end

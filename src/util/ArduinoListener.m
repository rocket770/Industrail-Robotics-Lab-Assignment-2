classdef ArduinoListener < handle
    properties(Access = private)
        arduinoObj
        eStopInstance % Instance of the EStop class
    end
    
    methods
        function obj = ArduinoListener(port, baudRate, eStopInstance)

            if nargin < 2
                baudRate = 9600; % Default baud rate
            end
            
            % Create and configure the serial port object
            obj.arduinoObj = serialport(port, baudRate);
            configureTerminator(obj.arduinoObj, "CR/LF");
            flush(obj.arduinoObj);
            
            % Assign the EStop instance
            obj.eStopInstance = eStopInstance;
            
        end
        
        function delete(obj)
            % Destructor to ensure the serial port is properly closed
            delete(obj.arduinoObj);
        end
        
        function checkForData(obj)
            % Check if there's any data available to read
            if obj.arduinoObj.NumBytesAvailable > 0
                data = readline(obj.arduinoObj);
                %disp(data)
                if strcmp(data, 'S')
                    obj.eStopInstance.triggerEStop(); % Call the triggerEStop method of the EStop instance
                end
            end
        end
    end
end

classdef Finger < RobotBaseClass

    properties(Access = public)              
        plyFileNameStem = 'Finger';
    end
    
    methods
%% Define robot Function 
        function self = Finger()  
            self.CreateModel();
            

            self.PlotAndColourRobot();         
        end

%% Create the robot model
        function CreateModel(self)   
            % Create the UR3 model mounted on a linear rail
        
            link(1) = Link('d',0,'a',0.05,'alpha',pi/2,'offset',0);
            link(2) = Link('d',0,'a',0.06,'alpha',0,'offset',0);

            link(1).qlim = [0 30]*pi/180;
            link(2).qlim = [0 180]*pi/180;
            
            self.model = SerialLink(link,'name',self.name);
        end
     
    end
end
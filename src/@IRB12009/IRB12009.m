classdef IRB12009 < RobotBaseClass
    %% IRB 1200 0.9 Robot version

    properties(Access = public)              
        plyFileNameStem = 'IRB12009';   % Change the ply files
    end

    methods
%% Define robot Function 
function self = IRB12009() 
            cla;
			self.CreateModel();
            
            if nargin < 1			
				baseTr = [1,0,0,0
            		      0,1,0,0
                         0,0,1,0
                         0,0,0,1];
            end
            self.model.base = self.model.base.T * baseTr; % trotx(pi/2) * troty(pi/2);

            self.PlotAndColourRobot();         
        end

%% Create the robot model 
        function CreateModel(self)   

            
            % The DH parameters derived from the internet
            link(1) = Link([0     0.399      0    -pi/2   0]); 
            link(2) = Link([0      0       0.448      0  0]); 
            link(3) = Link([0      0       0.042    -pi/2      0]);
            link(4) = Link([0      0.451     0        pi/2     0]);
            link(5) = Link([0      0         0       -pi/2     0]);
            link(6) = Link([0      0.082     0       0      	0]);

         
            % Incorporate joint limits
            link(1).qlim = [-170 170]*pi/180;
            link(2).qlim = [-100 130]*pi/180;
            link(3).qlim = [-200 70]*pi/180;
            link(4).qlim = [-270 270]*pi/180;
            link(5).qlim = [-130 130]*pi/180;
            link(6).qlim = [-360 360]*pi/180;
            % 


            link(2).offset = -pi/2;
            % 
            self.model = SerialLink(link,'name',self.name);

            % I created the below two lines for testing
            % q = zeros(1,6);
            % self.model.teach()
        end
     
    end
end
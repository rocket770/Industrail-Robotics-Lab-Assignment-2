classdef EnvironmentSetup < handle


    methods
%% Constructor
function self = EnvironmentSetup()
            cla;
            self.kitchen();
            self.safetyObjects();
         
        end

%% Placing object functions

            function kitchen(self)

                %Place kitchen top
                stove = PlaceObject('stove.ply',[0,-1.2,0]);  
                verts = [get(stove,'Vertices'), ones(size(get(stove,'Vertices'),1),1)];
                set(stove,'Vertices',verts(:,1:3));

                hold on;

                 %Place open oven with tray
                h_1 = PlaceObject('KitchenCounter.ply',[0,0,0]);  
                verts = [get(h_1,'Vertices'), ones(size(get(h_1,'Vertices'),1),1)];
                set(h_1,'Vertices',verts(:,1:3));

                %Place floor
                surf([-3,-3;3,3],[-3,3;-3,3],[0.01,0.01;0.01,0.01],'CData',imread('kitchenfloor.jpg'),'FaceColor', 'texturemap');

            end 

            

            function safetyObjects(self)
                
                % Place table to mount the IRB 1200 0.9 On
                h_1 = PlaceObject('tableBrown2.1x1.4x0.5m.ply',[2,-1,0]);  
                verts = [get(h_1,'Vertices'), ones(size(get(h_1,'Vertices'),1),1)];
                set(h_1,'Vertices',verts(:,1:3));

                %Place fire_extinguisher 
            
                fire_extinguisher = PlaceObject('fireExtinguisher.ply',[2,3,0]);
                verts = [get( fire_extinguisher,'Vertices'), ones(size(get( fire_extinguisher,'Vertices'),1),1)];
                set( fire_extinguisher,'Vertices',verts(:,1:3));

                %Place emergency button
            
                emer_button = PlaceObject('emergencyStopButton.ply',[2.7,-0.5,0.5]);
                verts = [get( emer_button ,'Vertices'), ones(size(get( emer_button ,'Vertices'),1),1)];
                set( emer_button ,'Vertices',verts(:,1:3));

                % Place four barriers 

                barrier_1 = PlaceObject('barrier1.5x0.2x1m.ply',[(0 ),(3.1),(0)]);
                verts = [get( barrier_1 ,'Vertices'), ones(size(get( barrier_1 ,'Vertices'),1),1)];
                verts(:,1) = verts(:,1) * 4;
                verts(:,2) = verts(:,2);
                verts(:,3) = verts(:,3);
                set( barrier_1 ,'Vertices',verts(:,1:3));
            
                barrier_2 = PlaceObject('barrier1.5x0.2x1m.ply',[(0 ),(-3.1 ),(0)]);
                verts = [get( barrier_2 ,'Vertices'), ones(size(get( barrier_2 ,'Vertices'),1),1)];
                verts(:,1) = verts(:,1) * 4;
                verts(:,2) = verts(:,2);
                verts(:,3) = verts(:,3);
                set( barrier_2 ,'Vertices',verts(:,1:3));
            
                barrier_3 = PlaceObject('barrier1.5x0.2x1m.ply',[(0 ),(-3.1),(0)]);
                verts = [get( barrier_3 ,'Vertices'), ones(size(get( barrier_3 ,'Vertices'),1),1)]* trotz(-pi/2);
                verts(:,1) = verts(:,1);
                verts(:,2) = verts(:,2) * 4;
                verts(:,3) = verts(:,3) ;
                set( barrier_3 ,'Vertices',verts(:,1:3));
            
                barrier_4 = PlaceObject('barrier1.5x0.2x1m.ply',[(0),(3.1),(0)]);
                verts = [get( barrier_4 ,'Vertices'), ones(size(get( barrier_4 ,'Vertices'),1),1)]* trotz(-pi/2);
                verts(:,1) = verts(:,1) ;
                verts(:,2) = verts(:,2) * 4;
                verts(:,3) = verts(:,3);
                set( barrier_4 ,'Vertices',verts(:,1:3));

            end

    end
end
function [vertex,face,faceNormals] = plot_less_rectangular_prism(lower,upper,plotOptions,axis_h)
if nargin<4
        axis_h=gca;
    if nargin<3
        plotOptions.plotVerts=false;
        plotOptions.plotEdges=true;
        plotOptions.plotFaces=true;
    end
end
hold on

vertex(1,:)=lower;
vertex(2,:)=[upper(1),lower(2:3)];
vertex(3,:)=[upper(1:2),lower(3)];
vertex(4,:)=[upper(1),lower(2),upper(3)];
vertex(5,:)=[lower(1),upper(2:3)];
vertex(6,:)=[lower(1:2),upper(3)];
vertex(7,:)=[lower(1),upper(2),lower(3)];
vertex(8,:)=upper;

face=[1,2,3;1,3,7;
     1,6,5;1,7,5;
     1,6,4;1,4,2;
     6,4,8;6,5,8;
     2,4,8;2,3,8;
     3,7,5;3,8,5;
     6,5,8;6,4,8];

if 2 < nargout    
    faceNormals = zeros(size(face,1),3);
    for faceIndex = 1:size(face,1)
        v1 = vertex(face(faceIndex,1)',:);
        v2 = vertex(face(faceIndex,2)',:);
        v3 = vertex(face(faceIndex,3)',:);
        faceNormals(faceIndex,:) = unit(cross(v2-v1,v3-v1));
    end
end

end
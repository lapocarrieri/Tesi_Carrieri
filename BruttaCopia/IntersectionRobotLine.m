kuka = importrobot('./models/kuka_lwr.urdf');

 addVisual(kuka.Base,"Mesh","./visual/base.STL")
addVisual(kuka.Bodies{1},"Mesh","./visual/link_1.STL")
addVisual(kuka.Bodies{2},"Mesh","./visual/link_2.STL")
addVisual(kuka.Bodies{3},"Mesh","./visual/link_3.STL")
addVisual(kuka.Bodies{4},"Mesh","./visual/link_4.STL")
addVisual(kuka.Bodies{5},"Mesh","./visual/link_5.STL")
addVisual(kuka.Bodies{6},"Mesh","./visual/link_6.STL")
addVisual(kuka.Bodies{7},"Mesh","./visual/link_7.STL")
H = [-1.1 pi/4 0 1.3*pi -1 0 0];

prova = kuka.show(H,'visuals','on','collision','off');
return;
prova = robot.show([0 0 0 0 0 0 0 ]','visuals','on','collision','off');
% Define the line
x1 = 0; y1 = 0;
x2 = 1; y2 = 1;
kuka.Bodies{7}


for i = 1:length(robot.Links)
    % Get the vertices of the link
    vertices = robot.Links(i).getVisualMesh('base', eye(4)).Vertices;
    
    % Find the intersection points between the line and the link
    [xi, yi] = polyxpoly([x1 x2], [y1 y2], vertices(:,1), vertices(:,2));
    
    % Plot the link and the intersection points
    figure;
    plot(vertices(:,1), vertices(:,2));
    hold on;
    plot(xi, yi, 'ro');
    axis equal;
    title(sprintf('Link %d', i));
end
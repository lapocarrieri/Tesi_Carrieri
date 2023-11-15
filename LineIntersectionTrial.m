
clc
clear all
close all
mashes=cell(7);
addpath 'visual'

line.origin = [0 0 0]; % Example line origin
line.direction = [0 0 1 ]; % Example line direction
t = linspace(0, 1, 200); % You can adjust the number of points as desired


IntersectionPoint(line,2,[0 0 0 0 0 0 0])
% Compute the coordinates of the line
x = line.origin(1) + line.direction(1)*t;
y = line.origin(2) + line.direction(2)*t;
z = line.origin(3) + line.direction(3)*t;
figure;
% Plot the 3D line
plot3(x, y, z, 'b', 'LineWidth', 0.2);  


hold on

link = 2; 
STLlink = ['./visual/link_', num2str(link), '.STL'];
[meshes, ~, ~, ~] = stlread(STLlink);
   

       T=QtoP([0 0 0 0 0 0 0],link);
       points=[meshes.Points ones(1,size(meshes.Points(:,1),1))']';
       linkPoints=T*points;

   % Create a sample 1666x3 matrix

matrix = linkPoints';

% Extract individual columns  
x = matrix(:, 1);
y = matrix(:, 2);
z = matrix(:, 3);

% Plot the data

plot3(x, y, z, 'b.');
title('3D Plot');
xlabel('X');
ylabel('Y');
zlabel('Z');
grid on;

   
        [intersection,point] = checkIntersection(meshes, line);

function [intersection,i] = checkIntersection(vertices, line)
    for i = 1:size(vertices.ConnectivityList, 1)
        indices = vertices.ConnectivityList(i, :);
        p1 = vertices.Points(indices(1), :);
        p2 = vertices.Points(indices(2), :);
        p3 = vertices.Points(indices(3), :);
        triangle=[p1;p2;p3];
        
         
   




    % triangle: 3x3 matrix representing the triangle vertices (each row is a vertex)
    % line: struct with 'origin' and 'direction' fields representing the line
    
    % Triangle vertices
    v1 = triangle(1, :);
    v2 = triangle(2, :);
    v3 = triangle(3, :);
    
    % Line origin and direction
    origin = line.origin;
    direction = line.direction;
    
    % Möller–Trumbore algorithm
    epsilon = 1e-9; % small epsilon value for floating-point comparison
    
    edge1 = v2 - v1;
    edge2 = v3 - v1;
    h = cross(direction, edge2);
    a = dot(edge1, h);
    
    if a > -epsilon && a < epsilon
        intersection = false; % Line is parallel to the triangle
        point = [];
        continue;
    end
    
    f = 1 / a;
    s = origin - v1;
    u = f * dot(s, h);
    
    if u < 0 || u > 1
        intersection = false; % Intersection point is outside the triangle
        point = [];
        continue;
    end
    
    q = cross(s, edge1);
    v = f * dot(direction, q);
    
    if v < 0 || u + v > 1
        intersection = false; % Intersection point is outside the triangle
        point = [];
        continue;
    end
    
    t = f * dot(edge2, q);
    
    if t > epsilon
        intersection = true; % Line intersects the triangle
        point = origin + t * direction; % Calculate intersection point
    else
        intersection = false; % Line intersects, but intersection point is behind the line
        point = [];

    end
    if intersection
            disp(i);
            disp(point);
    else 
        disp("no intersection points")
    end% Display the result
  end
end


clear all
close all
load('initialization6.mat')
link=6;
matrix = Meshes.Points(1:2000,1:3,link);
surface_points = matrix(matrix(:, 2, 1) ~= 0, :, :);
point_on_surface = [ 0.067 0.007 0.023];  % Point on the surface
figure();
scatter3(point_on_surface(1),point_on_surface(2),point_on_surface(3),'g', 'filled' ,'SizeData', 100);
hold on
n = size(surface_points, 1);  % Number of surface points
num_part = 20;  % Number of points to generate

generated_points = generate_points_nearby(n, point_on_surface, surface_points, num_part);
disp(generated_points);

% Generate random data points

scatter3(surface_points(:,1),surface_points(:,2),surface_points(:,3),'r', 'filled' ,'SizeData', 50);
hold on
scatter3(generated_points(:,1),generated_points(:,2),generated_points(:,3),'b', 'filled' ,'SizeData', 20);

xlabel('X');
ylabel('Y');
zlabel('Z');
title('3D Point Cloud');
axis equal;
grid on;





function generated_points = generate_points_nearby(n, point_on_surface, surface_points, b)
    
   % distances = vecnorm(surface_points - point_on_surface, 2, 2);
    
   
    sigma = 2;

   distances=  surface_points - point_on_surface ;
    min_distance = min(distances);
    max_distance = max(distances);
    
    generated_points = zeros(b, 3);
  
   
    for i = 1:20
        
        
        distance = generate_random_3d_vector(max_distance, min_distance,sigma)*0.3;
        distances=  surface_points - point_on_surface -distance ;
        norms = vecnorm(distances, 2, 2);
       %dist(1:10,:)
       
        

        
        

        [~, idx] = min(norms)
        surface_points(idx, :);
        generated_points(i, :) = surface_points(idx, :);
        surface_points(idx,:)=[];
    end
end
function random_vector = generate_random_3d_vector(M, m,sigma)
    mean = [0, 0, 0];
    cov = diag([sigma, sigma,sigma]);  % Identity covariance matrix

    while true
        vector = mvnrnd(mean, cov);
        if all(vector < M) && all(vector > m)
            random_vector = vector;
            return
        end
    end
end


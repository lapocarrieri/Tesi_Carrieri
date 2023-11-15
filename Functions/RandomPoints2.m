surface_points = (T*Meshes.Points(1:10000,:,link)')';
surface_points=surface_points(:,1:3);% Surface points forming triangles
point_on_surface = [  -0.044547203928232   0.007306028623134  -0.055358994752169];  % Point on the surface

n = size(surface_points, 1);  % Number of surface points
b = 50;  % Number of points to generate

generated_points = generate_points_nearby(n, point_on_surface, surface_points, b);
disp(generated_points);

% Generate random data points
scatter3(surface_points(:,1),surface_points(:,2),surface_points(:,3),'r', 'filled' ,'SizeData', 50);
hold on
scatter3(generated_points(:,1),generated_points(:,2),generated_points(:,3),'b', 'filled' ,'SizeData', 50);

xlabel('X');
ylabel('Y');
zlabel('Z');
title('3D Point Cloud');
grid on;





function generated_points = generate_points_nearby(n, point_on_surface, surface_points, b)
    distances = vecnorm(surface_points - point_on_surface, 2, 2);
    min_distance = min(distances);
    max_distance = max(distances);
    range = max_distance - min_distance;
    
    generated_points = zeros(b, 3);
    for i = 1:b
        relative_position = rand();
        distance = min_distance + relative_position * range;
        distance = normrnd(0, range/3);  % Adjust standard deviation as desired
        
        
        
        [~, idx] = min(abs(distances - distance));
        generated_points(i, :) = surface_points(idx, :);
    end
end
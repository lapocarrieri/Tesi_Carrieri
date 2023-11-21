% %example of use
% P = [0.043  -0.04  -0.03];
% closest_point = closest_point_to_triangle(triangles, P);
% disp('Closest point:');
% disp(closest_point);
function closest_point = closest_point_to_triangle2(points, P)
        % point: a 1x3 vector representing the 3D coordinates of the point
    % pointMatrix: an Nx3 matrix where each row represents a 3D point

    % Calculate the squared Euclidean distances
    distances = sum((points - P).^2, 2);

    % Find the index of the minimum distance
    [~, minIndex] = min(distances);

    % Retrieve the closest point
    closest_point = points(minIndex, :);
    
end
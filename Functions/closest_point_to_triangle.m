% %example of use
% P = [0.043  -0.04  -0.03];
% closest_point = closest_point_to_triangle(triangles, P);
% disp('Closest point:');
% disp(closest_point);
function [closest_point,normale] = closest_point_to_triangle(triangles, P)
    closest_point = [];
    min_distance = inf;

    for i = 1:size(triangles, 3)
        triangle = triangles(:, :, i);
        A = triangle(1, :);
        B = triangle(2, :);
        C = triangle(3, :);

        % Calculate the normal vector of the triangle's plane
        normal = cross(B - A, C - A);
        normal = normal / norm(normal);

        % Calculate the closest point on the triangle's plane to P

        t = dot(normal, P - A);
        closest_point_plane = P - t * normal;

            distance = norm(closest_point_plane - P);
            if distance < min_distance
                min_distance = distance;
                closest_point = closest_point_plane;
                normale=normal;
                
            end
        
    end
end
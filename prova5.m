

% Example usage
%triangles = cat(3, [0, 0, 0; 1, 0, 0; 0, 1, 1]);
P = [0.043  -0.04  -0.03];


closest_point = closest_point_to_triangle(triangles, P);
disp('Closest point:');
disp(closest_point);
function closest_point = closest_point_to_triangle(triangles, P)
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

        % Check if the closest point lies within the triangle
        edge1 = B - A;
        edge2 = C - A;
        point_vector = closest_point_plane - A;

        dot11 = dot(edge1, edge1);
        dot12 = dot(edge1, edge2);
        dot22 = dot(edge2, edge2);
        dot_pv1 = dot(point_vector, edge1);
        dot_pv2 = dot(point_vector, edge2);

        inv_denom = 1 / (dot11 * dot22 - dot12 * dot12);
        u = (dot22 * dot_pv1 - dot12 * dot_pv2) * inv_denom;
        v = (dot11 * dot_pv2 - dot12 * dot_pv1) * inv_denom;

        % If the closest point is within the triangle, update the minimum distance and closest point
        if u >= 0 && v >= 0 && u + v <= 1
            distance = norm(closest_point_plane - P);
            if distance < min_distance
                min_distance = distance;
                closest_point = closest_point_plane;
            end
        end
    end

end
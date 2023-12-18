function [intersection_point] = IntersectionPoint(triangles, line_origin, line_direction)
    num_triangles = size(triangles, 3);
    intersection_point = NaN(1, 3); % Default to no intersection
    is_intersect = false;

    for i = 1:num_triangles
        triangle = triangles(:, :, i);
        A = triangle(1, :);
        B = triangle(2, :);
        C = triangle(3, :);

        % Call the function to check intersection with this triangle
        [inter_point, intersects] = rayIntersectsTriangle(line_origin, line_direction, A, B, C);
        
        if intersects
            intersection_point = inter_point;
            is_intersect = true;
            return; % Return on first intersection
        end
    end
end

function [inter_point, intersects] = rayIntersectsTriangle(rayOrigin, rayVector, vertex0, vertex1, vertex2)
    EPSILON = 1e-6;
    edge1 = vertex1 - vertex0;
    edge2 = vertex2 - vertex0;
    h = cross(rayVector, edge2);
    a = dot(edge1, h);
    
    if a > -EPSILON && a < EPSILON
        intersects = false; % This means the ray is parallel to the triangle.
        inter_point = NaN(1, 3);
        return;
    end
    
    f = 1.0 / a;
    s = rayOrigin - vertex0;
    u = f * dot(s, h);
    
    if u < 0.0 || u > 1.0
        intersects = false;
        inter_point = NaN(1, 3);
        return;
    end

    q = cross(s, edge1);
    v = f * dot(rayVector, q);
    
    if v < 0.0 || u + v > 1.0
        intersects = false;
        inter_point = NaN(1, 3);
        return;
    end

    % At this stage, we can compute t to find out where the intersection point is on the line.
    t = f * dot(edge2, q);
    if t > EPSILON
        intersects = true;
        inter_point = rayOrigin + rayVector * t;
    else
        intersects = false; % This means there is a line intersection but not a ray intersection.
        inter_point = NaN(1, 3);
    end
end

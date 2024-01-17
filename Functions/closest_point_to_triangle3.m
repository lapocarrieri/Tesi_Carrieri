% %example of use
% P = [0.043  -0.04  -0.03];
% closest_point = closest_point_to_triangle(triangles, P);
% disp('Closest point:');
% disp(closest_point);
function [closest_point,normale] = closest_point_to_triangle3(triangles, P)
   num_triangles = size(triangles, 3);
    
    % Reshape the triangles array to a 2D matrix where each row represents a vertex
    triangles_reshaped = reshape(triangles, [], 3, 1);

    % Replicate the point P to match the number of vertices
    P_replicated = repmat(P, size(triangles_reshaped, 1), 1);
    
    % Calculate distances in a vectorized manner
    distances = vecnorm(triangles_reshaped - P_replicated, 2, 2);

    % Find the minimum distance and corresponding triangle
    [minDistance, minIndex] = min(distances);
    triangle_index = ceil(minIndex / 3);
    triangle = triangles(:, :, triangle_index);
    A = triangle(:,1);
    B = triangle(:,2);
    C = triangle(:,3);
    
        % Calculate the normal vector of the triangle's plane
        normal = cross(B - A, C - A);
        normale = normal / norm(normal);

  
        closest_point = closestPointOnTriangle(P, A', B', C');
        normale = processNormal(normale, closest_point', triangles);      
%         figure;
% hold on;
% grid on;
% axis equal;
% 
% % Iterate through each triangle and plot
% for i = 1:size(triangles, 3)
%     % Extract the i-th triangle
%     triangle = triangles(:, :, i);
% 
%     % Plot each edge of the triangle
%     % Adding the first point again at the end to close the triangle
%     plot3([triangle(1, 1), triangle(1, 2)], ...
%           [triangle(2, 1), triangle(2, 2)], ...
%           [triangle(3, 1), triangle(3, 2)], 'b');
%     plot3([triangle(1, 2), triangle(1, 3)], ...
%           [triangle(2, 2), triangle(2, 3)], ...
%           [triangle(3, 2), triangle(3, 3)], 'b');
%     plot3([triangle(1, 3), triangle(1, 1)], ...
%           [triangle(2, 3), triangle(2, 1)], ...
%           [triangle(3, 3), triangle(3, 1)], 'b');
% end
% 
% 
%         line([A(1) B(1) C(1) A(1)], [A(2) B(2) C(2) A(2)], [A(3) B(3) C(3) A(3)], 'Color', 'y', 'MarkerSize', 20);
% 
%         % Plot point P
%         plot3(P(1), P(2), P(3), 'k.', 'MarkerSize', 20);
%         plot3(closest_point(1), closest_point(2), closest_point(3), 'r.', 'MarkerSize', 20); 
% 
%         % Plot the normal vector
%         % The normal will start at point P and will have the direction of 'normale'
%         % quiver3(P(1), P(2), P(3), normale(1), normale(2), normale(3), 0.5, 'Color', 'g');
% 
%         % Setting up the plot
%         xlabel('X Axis');
%         ylabel('Y Axis');
%         zlabel('Z Axis');
%         title('Triangle, Point P, and Normal Vector');
%         axis equal;
          
    end
function closestPoint = closestPointOnTriangle(P, A, B, C)
    % Vettori del triangolo
    AB = B - A;
    AC = C - A;
    AP = P - A;

    % Proiezione su AB e AC
    d1 = dot(AB, AP);
    d2 = dot(AC, AP);
    
    if d1 <= 0 && d2 <= 0
        % Il punto più vicino è A
        closestPoint = A;
        return;
    end

    % Calcola BP
    BP = P - B;
    d3 = dot(AB, BP);
    d4 = dot(AC, BP);
    if d3 >= 0 && d4 <= d3
        % Il punto più vicino è B
        closestPoint = B;
        return;
    end

    % Coordinate baricentriche
    vc = d1*d4 - d3*d2;
    if d1 >= 0 && d2 >= 0 && vc <= 0
        v = d1 / (d1 - d3);
        closestPoint = A + v * AB;
        return;
    end

    % Calcola CP
    CP = P - C;
    d5 = dot(AB, CP);
    d6 = dot(AC, CP);
    if d6 >= 0 && d5 <= d6
        % Il punto più vicino è C
        closestPoint = C;
        return;
    end

    % Controlla se siamo all'interno del triangolo
    vb = d5*d2 - d1*d6;
    if d2 >= 0 && d6 <= 0 && vb <= 0
        w = d2 / (d2 - d6);
        closestPoint = A + w * AC;
        return;
    end

    va = d3*d6 - d5*d4;
    if va <= 0 && (d4 - d3) >= 0 && (d5 - d6) >= 0
        w = (d4 - d3) / ((d4 - d3) + (d5 - d6));
        closestPoint = B + w * (C - B);
        return;
    end

    % Il punto è all'interno del triangolo
    denom = 1 / (va + vb + vc);
    v = vb * denom;
    w = vc * denom;
    closestPoint = A + AB * v + AC * w;
end
function intersect = isIntersecting(normale, point, triangle)
    % Constants
    EPSILON = 1e-6;

    % Triangle vertices
    v1 = triangle(:,1);
    v2 = triangle(:,2);
    v3 = triangle(:,3);

    % Edge vectors
    edge1 = v2 - v1;
    edge2 = v3 - v1;

    % Calculate determinant
    h = cross(normale, edge2);
    a = dot(edge1, h);

    % Ray is parallel to triangle
    if (a > -EPSILON && a < EPSILON)
        intersect = false;
        return;
    end

    % Calculate barycentric coordinates
    f = 1.0 / a;
    s = point - v1;
    u = f * dot(s, h);

    if (u < 0.0 || u > 1.0)
        intersect = false;
        return;
    end

    q = cross(s, edge1);
    v = f * dot(normale, q);

    if (v < 0.0 || u + v > 1.0)
        intersect = false;
        return;
    end

    % At this stage, we can compute t to find out where
    % the intersection point is on the line
    t = f * dot(edge2, q);

    if (t > EPSILON) % Ray intersection
        intersect = true;
    else % Line intersection but not a ray intersection
        intersect = false;
    end
end
function normale = processNormal(normale, closest_point, triangles)
    % Assuming triangles is a 3x3xn matrix where each page represents a triangle

    n = size(triangles, 3); % Number of triangles
    hit = false; % Flag to check if intersection occurs

    for i = 1:n
        % Extract vertices of the triangle
        v1 = triangles(:, :, i);
        % Check for intersection with each triangle
        if isIntersecting(normale, closest_point, v1)
            hit = true;
            break;
        end
    end

    % Reverse normal if no intersection
    if ~hit
        normale = -normale;
    end
end

function [Point_intersected] = checkIntersection2(vertices, line)
    Point_intersected=[];
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
    
    
    edge1 = v2 - v1;
    edge2 = v3 - v1;    
    N = cross(edge1, edge2);
    syms t;
    line_eqn = origin + t * direction; % line equation
    plane_eqn = dot(N, (v1 - line_eqn)) % plane equation
    t_val = solve(plane_eqn, t);
    return;

   
    intersection_point = origin + t_val * direction;
    Point_intersected=[Point_intersected,intersection_point];
    end
    plot3(Point_intersected(:,1),Point_intersected(:,2),Point_intersected(:,3), 'o', 'MarkerSize', 8, 'MarkerFaceColor', 'g', 'LineWidth', 4);



end

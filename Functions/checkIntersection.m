function [Point_intersected] = checkIntersection(vertices, line,PreviousPoint)

%     vertices.ConnectivityList = [1 2 3]
%     vertices.Points=[0 0 0; 0 1 0; 1 0 0]
%     line.origin=[0.2 0.2 -1]
%     line.direction=[ 0.01 0.01 1];
%     checkIntersection(vertices, line)
    intersection1=false;
    distance=2;
    Point_intersected=[];
    % Line origin and direction
    origin = line.origin;
    direction = line.direction;
    i=0;
    
    % Möller–Trumbore algorithm
    epsilon = 1e-9; % small epsilon value for floating-point comparison
    while( i <size(vertices.ConnectivityList, 1))
        i=i+1;
        if i==size(vertices.ConnectivityList, 1)-1
            distance=distance*2;
            i=1;
        end
        if distance>2
            Point_intersected=[0 0 0];
            break;
        end
        
        if size(Point_intersected,1)>1
            break;
        end
        
        indices = vertices.ConnectivityList(i, :);
        p1 = vertices.Points(indices(1), 1:3);
        if PreviousPoint~=[0 0 0]
                if  norm(PreviousPoint-p1)>distance 
                    
                    continue;
                end
        end
       vertices.ConnectivityList = vertices.ConnectivityList([1:i-1, i+1:end], :);
%         for i=1:size(vertices.Points,1)
%             norm(PreviousPoint-vertices.Points(i, 1:3))
%         end


        p2 = vertices.Points(indices(2), 1:3);
        p3 = vertices.Points(indices(3), 1:3);
        triangle=[p1;p2;p3];
        
      

    % triangle: 3x3 matrix representing the triangle vertices (each row is a vertex)
    % line: struct with 'origin' and 'direction' fields representing the line
    
    % Triangle vertices
    v1 = triangle(1, :);
    v2 = triangle(2, :);
    v3 = triangle(3, :);
   
    
       
    edge1 = v2 - v1;
    edge2 = v3 - v1;
    h = cross(direction', edge2);
    
    a = dot(edge1, h);
 
    if a > -epsilon && a < epsilon

        continue;
    end
    
    f = 1 / a;
    s = origin' - v1;
    u = f * dot(s, h);
 
    if u < 0 || u > 1

        continue;
    end
    
    q = cross(s, edge1);
    v = f * dot(direction, q);
    
    if v < 0 || u + v > 1

        continue;
    end
    
    t = f * dot(edge2, q);

    
    if t > epsilon
        intersection1 = true; % Line intersects the triangle
        
        Point_intersected = origin' + t * direction % Calculate intersection point
        
        
%         disp(indices)
%         disp(vpa(triangle,2))
        
        %plot3(triangle(:,1),triangle(:,2),triangle(:,3))
        
        break;

    end

    

    end    
    if intersection1 == false
            
        Point_intersected=[0 0 0];
    end


    
end

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
    
    t = f * dot(v3 - v1, cross(origin' - v1, v2 - v1));

    
    if t > epsilon
        intersection1 = true; % Line intersects the triangle
        
        Point_intersected = origin' + t * direction % Calculate intersection point
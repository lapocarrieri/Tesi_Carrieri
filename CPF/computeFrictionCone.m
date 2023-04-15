function [F1, F2, F3, F4] = computeFrictionCone(point, theta, R)

    %orientation of the cone
    alpha  = -point(3,end)*10;              %rotatoin around y is not really correct should be better
    ct = cos(-theta);                       %rotation around x to orient the cone along the surface normal
    st = sin(-theta);
    ca = cos(alpha);
    sa = sin(alpha);
    len = 0.1;
    % R = eye(3);
    Rz = [[0 0 1];[0 1 0];[-1 0 0]];
    Rx = [[1 0 0];[0 ct -st];[0 st ct]];
    Ry = [[ca 0 sa];[0 1 0];[-sa 0 ca]];
    R_prime = Rz*Rx*Ry;
    Rot = R*R_prime;
   
    elevation = pi/4;
    
    %convert the azimuth and elevation in cartisian coordinates
    [x1, y1, z1] = sph2cart(45,elevation,len);
    F1 = Rot*[x1, y1, z1]';
    [x2, y2, z2] = sph2cart(-45,elevation,len);
    F2 = Rot*[x2, y2, z2]';
    [x3, y3, z3] = sph2cart(90,elevation,len);
    F3 = Rot*[x3, y3, z3]';
    [x4, y4, z4] = sph2cart(-90,elevation,len);
    F4 = Rot*[x4, y4, z4]';
    
end
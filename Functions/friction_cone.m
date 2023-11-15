%this scripts computes the friction cone for each particle but in the paper
%i didnt yous the friction cone constraints
close all

r = 0.045;           % radii of cyl
cnt = [0,0,0];       % [x,y,z] center cyl
height = -0.05;      % height of cyl, is negative because the frame is on the top 
C = [0 0 0]';       %center of bottom circle
H = [0 0 height]';  %center of top circle
Sigma = eye(6)*0.0001;

b = pi/4;
Rtest = [[cos(b) -sin(b) 0]; [sin(b) cos(b) 0 ];[0 0 1]];

for i=1:100
    theta =  normrnd(pi, 0.5);
    x = r*cos(theta)+ cnt(1);
    y = r*sin(theta) +  cnt(2);
    z = rand()*height + cnt(3);
    chi(:, i) = Rtest*[x y z]';
end


plot3(chi(1,:), chi(2,:), chi(3,:), ".");

alpha  = -chi(3,end)*10;
ct = cos(-theta);
st = sin(-theta);
ca = cos(alpha);
sa = sin(alpha);
len = 0.1;
% R = eye(3);

Rz = [[0 0 1];[0 1 0];[-1 0 0]];
Rx = [[1 0 0];[0 ct -st];[0 st ct]];
Ry = [[ca 0 sa];[0 1 0];[-sa 0 ca]];
R = Rz*Rx*Ry;
R = Rtest*R;
o = [0 0 0]'+ chi(:,end);
n = R*[0 0 len]';
plotRF(R, o+n, len);

elevation = pi/4;

plot3(o(1), o(2), o(3), ".")
hold on
[x1, y1, z1] = sph2cart(45,elevation,len);
p1 = R*[x1, y1, z1]';
line([o(1), p1(1)], [o(2), p1(2)], [o(3), p1(3)])
[x2, y2, z2] = sph2cart(-45,elevation,len);
p2 = R*[x2, y2, z2]';
line([o(1), p2(1)], [o(2), p2(2)], [o(3), p2(3)])
[x3, y3, z3] = sph2cart(90,elevation,len);
p3 = R*[x3, y3, z3]';
line([o(1), p3(1)], [o(2), p3(2)], [o(3), p3(3)])
[x4, y4, z4] = sph2cart(-90,elevation,len);
p4 = R*[x4, y4, z4]';
line([o(1), p4(1)], [o(2), p4(2)], [o(3), p4(3)])

line([p1(1) p2(1)], [p1(2) p2(2)], [p1(3) p2(3)]);
line([p2(1) p4(1)], [p2(2) p4(2)], [p2(3) p4(3)]);
line([p3(1) p4(1)], [p3(2) p4(2)], [p3(3) p4(3)]);
line([p3(1) p1(1)], [p3(2) p1(2)], [p3(3) p1(3)]);

view(0, 90)

xlim([-0.5 0.5]);
ylim([-0.5 0.5]);
zlim([-0.5 0.5]);

grid on


function plotRF(R, tran, len)

    origin = tran;
    length_axis = len;
    x = R*[length_axis 0 0]'+tran;
    y = R*[0 length_axis 0]'+tran;
    z = R*[0 0 length_axis]'+tran;

    line([origin(1) x(1)], [origin(2) x(2)], [origin(3) x(3)],"color", "r", "lineWidth", 1);
    hold on 
    line([origin(1) y(1)], [origin(2) y(2)], [origin(3) y(3)],"color", "g", "lineWidth", 1);
    hold on 
    line([origin(1) z(1)], [origin(2) z(2)], [origin(3) z(3)],"color", "b", "lineWidth", 1);
    
end
addpath 'Functions'
figure();

matrix=zeros(2000,4,7);
link=1;
STLlink = ['./visual/link_', num2str(link), '.STL'];
[meshes, ~, ~, ~] = stlread(STLlink);
R=[-1 0 0;0 0 1;0 -1 0];
points=[-1 0 0 0;0 1 0 -0.2; 0 0 1 0;0 0 0 1]*[R*meshes.Points';ones(1,size(meshes.Points(:,1),1))];
sizePoints(link)=size(points,2);
matrix(1:size(points,2),1:4,1) = points';
link=2;
STLlink = ['./visual/link_', num2str(link), '.STL'];
[meshes, ~, ~, ~] = stlread(STLlink);
R=eye(3);
points=[R*meshes.Points';ones(1,size(meshes.Points(:,1),1))];
sizePoints(link)=size(points,2);
matrix(1:size(points,2),1:4,2) = points';
link=3;
STLlink = ['./visual/link_', num2str(link), '.STL'];
[meshes, ~, ~, ~] = stlread(STLlink);
R=eye(3);
R=[-1 0 0;0 0 -1;0 1 0];
points=[1 0 0 0;0 1 0 0.2; 0 0 1 0;0 0 0 1]*[(R*meshes.Points');ones(1,size(meshes.Points(:,1),1))];
sizePoints(link)=size(points,2);
matrix(1:size(points,2),1:4,3) = points';
link=4;
STLlink = ['./visual/link_', num2str(link), '.STL'];
[meshes, ~, ~, ~] = stlread(STLlink);
R=eye(3);
R=[-1 0 0;0 1 0;0 0 1];
points=[(R*meshes.Points');ones(1,size(meshes.Points(:,1),1))];
sizePoints(link)=size(points,2);
matrix(1:size(points,2),1:4,4) = points';
link=5;
STLlink = ['./visual/link_', num2str(link), '.STL'];
[meshes, ~, ~, ~] = stlread(STLlink);
R=eye(3);
R=[-1 0 0;0 0 1;0 -1 0];
points=[1 0 0 0;0 1 0 -0.2; 0 0 1 0;0 0 0 1]*[(R*meshes.Points');ones(1,size(meshes.Points(:,1),1))];
sizePoints(link)=size(points,2);
matrix(1:sizePoints(link),1:4,5) = points';
link=6;
STLlink = ['./visual/link_', num2str(link), '.STL'];
[meshes, ~, ~, ~] = stlread(STLlink);
R=eye(3);
R=[1 0 0;0 -1 0;0 0 -1];
points=[(R*meshes.Points');ones(1,size(meshes.Points(:,1),1))];
sizePoints(link)=size(points,2);
matrix(1:sizePoints(link),1:4,6) = points';
link=7;
STLlink = ['./visual/link_', num2str(link), '.STL'];
[meshes, ~, ~, ~] = stlread(STLlink);
R=eye(3);
R=[1 0 0;0 1 0;0 0 1];
points=[1 0 0 0;0 1 0 0; 0 0 1 0;0 0 0 1]*[(R*meshes.Points');ones(1,size(meshes.Points(:,1),1))];
sizePoints(link)=size(points,2);
matrix(1:sizePoints(link),1:4,7) = points';


M=matrix(1:max(sizePoints),:,:);
Meshes.Points=double(M);
Meshes.ConnectivityList=meshes.ConnectivityList;





hold on

kuka = importrobot('./models/kuka_lwr.urdf');

addVisual(kuka.Base,"Mesh","./visual/base.STL")
addVisual(kuka.Bodies{1},"Mesh","./visual/link_1.STL")
addVisual(kuka.Bodies{2},"Mesh","./visual/link_2.STL")
addVisual(kuka.Bodies{3},"Mesh","./visual/link_3.STL")
addVisual(kuka.Bodies{4},"Mesh","./visual/link_4.STL")
addVisual(kuka.Bodies{5},"Mesh","./visual/link_5.STL")
addVisual(kuka.Bodies{6},"Mesh","./visual/link_6.STL")
addVisual(kuka.Bodies{7},"Mesh","./visual/link_7.STL")

kuka.Gravity = [0,0,-9.81];
kuka.DataFormat = 'row';

%EF=0.1*log10(ExternalForce+0.1)+0.1;
% Difference

rateCtrlObj = rateControl(10000);

H = [-1.1 pi/4 0 1.3*pi -1 0 0];

prova = kuka.show(H,'visuals','on','collision','off');
prova.CameraPosition = [-2,7,6];
hold off
q0(1:7)=[0 0 pi/2 pi/3 -pi/4 0 pi/8]
for link=1:7
    T=QtoP(q0(1:7),link);
    matrix2(:,:,link)=(T*matrix(:,:,link)')';
    x = matrix2(:, 1,link);
    y = matrix2(:, 2,link);
    z = matrix2(:, 3,link);

    % Plot the data
    addpath('Functions')
    plot3(x, y, z, 'b.');
    title('3D Plot');
    xlabel('X');
    ylabel('Y');
    zlabel('Z');
    axis equal;
    grid on;
    hold on

end


prova.CameraPosition = [-2,7,6];

prova = kuka.show(q0(1:7),'visuals','on','collision','off');
prova.CameraPosition = [-2,7,6];
waitfor(rateCtrlObj);
hold off
return;
%

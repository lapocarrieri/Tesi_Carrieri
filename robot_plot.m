%% DRAW KUKA
clear;
clc;
close all;
addpath 'C:\Users\lapoc\Downloads'
%load("PROVA_WORKSPACE.mat");
load("Force130Point0.10.20.1Link7.mat");
kuka = importrobot('./models/kuka_lwr.urdf');


addVisual(kuka.Base,"Mesh","./visual/base.STL")
addVisual(kuka.Bodies{1},"Mesh","./visual/link_1.STL")
addVisual(kuka.Bodies{2},"Mesh","./visual/link_2.STL")
addVisual(kuka.Bodies{3},"Mesh","./visual/link_3.STL")
addVisual(kuka.Bodies{4},"Mesh","./visual/link_4.STL")
addVisual(kuka.Bodies{5},"Mesh","./visual/link_5.STL")
addVisual(kuka.Bodies{6},"Mesh","./visual/link_6.STL")
addVisual(kuka.Bodies{7},"Mesh","./visual/link_7.STL")


% Set gravity and data format
kuka.Gravity = [0,0,-9.81]; 
kuka.DataFormat = 'row';

% Set the robot configuration
H = [-1.1 pi/4 0 1.3*pi -1 0 0];
    stl_path = kuka.Bodies{5}.Visuals{1};
    
  
    
% Define the point p_dc and direction vector d
p_dc = [0.1, 0.2, 0.3];  % Replace with the actual coordinates of p_dc
d = [0.1, -0.1,0.1];  % Replace with the direction vector d


%% Arrow
%EF=0.1*log10(ExternalForce+0.1)+0.1
         % Difference

         

% for l=1:length(joints)-1    
%         ForcePointApplication_saved(l,:)=ForcePointApplication_saved(l,:)+[0; 0; 0.31]';
% end
%%
rateCtrlObj = rateControl(10000);

H = [-1.1 pi/4 0 1.3*pi -1 0 0];


prova = kuka.show(H,'visuals','on','collision','off');

prova.CameraPosition = [-2,7,6]; 

for i=1:length(joints)    
    %time(i)
    prova = kuka.show(joints(i,1:7),'visuals','on','collision','off')
    
%     hold on;
%     vpa(PointOfApplication(i,1:3),2)
% 
%     p1 = vpa(PointOfApplication(i,1:3)-EF',2)
%     
%     % First Point
%     
%     % Second Point
%     dp = EF;
%     str = {'A simple plot','from 1 to 10'};
%     text(2,7,str)
%     plot3(point(1),point(2),point(3)+0.2,"pentagram")
%     hold on
%     quiver3(p1(1),p1(2),p1(3),dp(1),dp(2),dp(3));
%     hold on;
    %% TEST 2 (CIRCLE)
    
    prova.CameraPosition = [-2,7,6];    
    scatter3(task_vec(:,10),task_vec(:,11),task_vec(:,12),[8],'red');
    %% PUT IN THE PLOT ALSO THE REFERENCE TRAJECTORY
    waitfor(rateCtrlObj);
    hold off;
end

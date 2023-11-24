%% DRAW KUKA
clear;
clc;
close all;
load("plotDatas")




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

for i=1:length(joints)    
    %time(i)
    
    prova.CameraPosition = [-2,7,6]; 
    
    prova = kuka.show(Q_sampled(i,1:7),'visuals','on','collision','off');
    prova.CameraPosition = [-2,7,6]; 
    %p1 = ForcePointApplication_saved(i,:)-EF;   
    
    % First Point
    
    % Second Point
   % dp = EF;
   
    %plot3(ForcePointApplication_saved(i,1),ForcePointApplication_saved(i,2),ForcePointApplication_saved(i,3)+0.2,"pentagram")
    
    %quiver3(p1(1),p1(2),p1(3),dp(1),dp(2),dp(3));
    
    %% TEST 2 (CIRCLE)
    
       
    %scatter3(task_vec(:,10),task_vec(:,11),task_vec(:,12),[8],'red');
    %% PUT IN THE PLOT ALSO THE REFERENCE TRAJECTORY
    waitfor(rateCtrlObj);
    hold off
end
hold off
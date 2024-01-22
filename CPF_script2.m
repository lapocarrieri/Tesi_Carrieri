for linkforce=4:7

close all;
indixes=1;
num_part=20;
Niterations=10;
load(['Initialization\initializations', num2str(linkforce), '.mat'])
close all
initialize=false;
addpath 'Dynamics'
f1=figure();
addpath 'Functions'
speed=1;
hold on
index2=1;
indexT=1;
f2=figure();
f10=figure();


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

rateCtrlObj = rateControl(10000);
error3=0.8;
f3=figure();
f4=figure();
chi2=zeros(3,num_part);
tic
while (toc<60)
load(['sharedDatas\sharedData',num2str(linkforce), '.mat']);
disp("waiting for collision")
pause(1)
num_part=50;
Niterations=10;
J_w = ComputePoint_withWrenches(Q_sampled(index,:),link);
if link_collided(index) > 0
disp('CPF:')
is_initialized=false;
indixes=indixes+1;
T=QtoP(Q_sampled(index,:),link);
Points=Meshes.Points(:,1:4,link);
zero_rows = all(Points == 0, 2);
matrixes = Points(~zero_rows, :);
matrix2=(T*matrixes')';
x = matrix2(:, 1);
y = matrix2(:, 2);
z = matrix2(:, 3);
x_actualFrame = matrixes(:, 1);
y_actualFrame = matrixes(:, 2);
z_actualFrame = matrixes(:, 3);
hold on
% Plot the data
addpath('Functions')
for i=1:speed:Niterations-1
hold off
generated_points=zeros(3,num_part);
if size(Point_intersectedActualFrame,1)==1
    Point_intersectedActualFrame=Point_intersectedActualFrame';
    if size(Point_intersectedActualFrame,1)==3
        Point_intersectedActualFrame=[Point_intersectedActualFrame;1]
    end
end

[chi2, W_prime,generated_points,Festimated,f4] = cpf_RealPoint3(num_part, chi2, TauExternalForce, Point_intersectedActualFrame,link,is_initialized,Meshes,triangles,generated_points,point,i,Niterations,J_w,f4,f2);
% figure(f1);
% hold off
% prova = kuka.show(Q_sampled(index,:), 'visuals', 'on', 'collision', 'off');
% hold on
% % Adding text in the bottom right corner of the figure
% textString = sprintf('Force = [%0.3f, %0.3f, %0.3f]\npoint = [%0.3f, %0.3f, %0.3f]\nerror = %0.3f\nlink = %d', ...
% ExternalForceAppliedActualFrame(1), ExternalForceAppliedActualFrame(2), ExternalForceAppliedActualFrame(3), ...
% point(1), point(2), point(3), ...
% error3,link);
% text(+0.5, 0.1,0.1, textString, 'HorizontalAlignment', 'left', 'VerticalAlignment', 'bottom', ...
% 'FontSize', 10, 'Color', 'black');
% view(135, 69);
% camzoom(5)
% plot3(x, y, z, 'b.');
% oggettiRobot = findobj(f1, 'Type', 'patch'); 
% % Applica la trasparenza a questi oggetti
% for ii = 1:length(oggettiRobot)
% set(oggettiRobot(ii), 'FaceAlpha', 0.5); % Imposta una trasparenza del 50%
% end

% starting Niterations before the end the contact particle
% filter is iterated Niterations times until the last index.
% this is done at every time instant so if the number of
% iterations is high the code is very slow

%% CPF
% here the code for the particle filter starts, the CPF is in
% the cpf function and takes in input the joints angle, the
% residuals and the point initialized
% In particular:
% num_part: numeber of particles
% chi_prev: particle at the previous step
% q: joint angles
% gamma: estimated external torque
% estimated_cp: estimated contact point with the deterministic method used in
%                the initialization phase
% (chi are the particles in respect to the actual frame)
%figure(f1);
%hold on
is_initialized=true;
chiWorldFrames=T*[chi2;ones(1,num_part)];
Initialppoint=T*Point_intersectedActualFrame;
%scatter3(chiWorldFrames(1,:),chiWorldFrames(2,:),chiWorldFrames(3,:),'y', 'filled' ,'SizeData', 5);
% Keep the current plot
realPoint=T*[point;1];
%scatter3(realPoint(1), realPoint(2), realPoint(3), 'r', 'filled'); % Plot the point
%text(realPoint(1), realPoint(2), realPoint(3), 'Real point'); % Add a label
% Additional plotting (force vectors, etc.) can be added here
waitfor(rateCtrlObj);
CalculatedPoint2=computeBari(chi2);
%             % Assuming chi2 is a 100x3 matrix representing 100 3D vectors
%             numClusters = 2; % We want to find 2 barycenters
%
%             % Use k-means to cluster the points into two groups
%             [idx, centroids] = kmeans(chi2', numClusters);
%
%             % The centroids variable now contains the two barycenters.
%             % Each row of centroids is a barycenter of one cluster.
%
%             % Display the barycenters
%             disp('Barycenter 1:');
%             disp(centroids(1, :));
%
%             disp('Barycenter 2:');
%             disp(centroids(2, :));
%
%
%             err=norm(centroids(2, :)'-point)
%             err=norm(centroids(1, :)'-point)
CalculatedPoint=closest_point_to_triangle(triangles, CalculatedPoint2);
CalculatedPoint2
CalculatedPoint
CalculatedPointWorldFrame=T*[CalculatedPoint';1];
%scatter3(CalculatedPointWorldFrame(1), CalculatedPointWorldFrame(2), CalculatedPointWorldFrame(3), 'g', 'filled'); % Plot the point
%text(CalculatedPointWorldFrame(1), CalculatedPointWorldFrame(2), CalculatedPointWorldFrame(3), 'Calcualted point'); % Add a label
error2=norm(CalculatedPoint2(1:3)'-point);
%ErrorAfterCPF(:,ind)
save(['sharedDatas\sharedVar', num2str(linkforce)],'CalculatedPoint','Festimated');

figure(f3);
hold off
% Plotting the contact particle filter points in yellow
scatter3(chi2(1,:), chi2(2,:), chi2(3,:), 'b', 'filled', 'SizeData', 10);
hold on
% Plotting the hypothesized point before CPF in smaller blue points
%scatter3(Point_intersectedActualFrame(1), Point_intersectedActualFrame(2), Point_intersectedActualFrame(3), 'm', 'filled', 'SizeData', 5); % Reduced size here
% Plotting the real point in red
scatter3(point(1), point(2), point(3), 'r', 'filled');
% Plotting CalculatedPoint2 in black
scatter3(CalculatedPoint(1), CalculatedPoint(2), CalculatedPoint(3), 'k', 'filled');
% Plotting the reference frame
plot3(x_actualFrame, y_actualFrame, z_actualFrame, 'y.');
% Calculate the normalized direction vector for ExternalForceAppliedActualFrame
norm_ExternalForce = norm(ExternalForceAppliedActualFrame);
direction_ExternalForce = ExternalForceAppliedActualFrame / norm_ExternalForce;
scaled_length_ExternalForce = norm_ExternalForce / 100;
% Arrow from 'point' in the direction of 'ExternalForceAppliedActualFrame' with scaled length
quiver3(point(1), point(2), point(3), ...
direction_ExternalForce(1) * scaled_length_ExternalForce, ...
direction_ExternalForce(2) * scaled_length_ExternalForce, ...
direction_ExternalForce(3) * scaled_length_ExternalForce, ...
'r', 'AutoScale', 'off');
Fm=ExternalForceAppliedActualFrame;
% Calculate the normalized direction vector for Fm(1:3)
norm_Fm = norm(Fm(1:3));
direction_Fm = Fm(1:3) / norm_Fm;
scaled_length_Fm = norm_Fm / 100;
% Arrow from 'CalculatedPoint' in the direction of 'Fm(1:3)' with scaled length
quiver3(CalculatedPoint(1), CalculatedPoint(2), CalculatedPoint(3), ...
direction_Fm(1) * scaled_length_Fm, ...
direction_Fm(2) * scaled_length_Fm, ...
direction_Fm(3) * scaled_length_Fm, ...
'k', 'AutoScale', 'off');
% Adding a legend
num_part_str = ['Number of Particles: ', num2str(num_part)]; % Convert num_part to string
N_iterations_str = ['Number of Iterations: ', num2str(Niterations)]; % Convert N_iterations to string
legend('Contact Particle Filter Points', 'Real Point', 'Point Hypothesized before CPF', 'Link Points',  'Real Force','Calculated Force', num_part_str, N_iterations_str, 'Location', 'best');
% Adding a title
title('3D Scatter Plot of Points and Hypotheses');
% Optionally, you can add labels for axes
xlabel('X-axis');
ylabel('Y-axis');
zlabel('Z-axis');
% Adjust view and other plot properties as needed
view(3); % for a 3D view
grid on; % to enable grid
axis equal;

CalculatedPointSampled(index2,:)=CalculatedPoint;
RealPointSampled(index2,:)=point;
CalculatedPointBeforeSampled(index2,:)=Point_intersectedActualFrame(1:3);
errorCPF(index2,:)=CalculatedPoint'-point;
errorPinv(index2,:)=point-Point_intersectedActualFrame(1:3);

PointCPFworldframe=T*[CalculatedPoint';1];
PointRealworldframe=T*[point;1];
PointPinvworldframe=T*Point_intersectedActualFrame;

CalculatedPointSampledworldframe(index2,:)=PointCPFworldframe(1:3);
RealPointSampledworldframe(index2,:)=PointRealworldframe(1:3);
CalculatedPointBeforeSampledworldframe(index2,:)=Point_intersectedActualFrame(1:3);
index2=index2+1;
end
% ErrorAfterCPF2(:,indexT)=norm(CalculatedPoint(1:3)'-point);
% indexT=indexT+1;
% CalculatedPointSampled(index2,:)=CalculatedPoint;
% RealPointSampled(index2,:)=point;
% CalculatedPointBeforeSampled(index2,:)=Point_intersectedActualFrame(1:3);
% errorCPF(index2,:)=CalculatedPoint'-point;
% errorPinv(index2,:)=point-Point_intersectedActualFrame(1:3);
% 
% PointCPFworldframe=T*[CalculatedPoint';1];
% PointRealworldframe=T*[point;1];
% PointPinvworldframe=T*Point_intersectedActualFrame;
% 
% CalculatedPointSampledworldframe(index2,:)=PointCPFworldframe(1:3);
% RealPointSampledworldframe(index2,:)=PointRealworldframe(1:3);
% CalculatedPointBeforeSampledworldframe(index2,:)=Point_intersectedActualFrame(1:3);
% index2=index2+1;
% time=1:1000;
% % Create a new figure
% figure(f10);
% 
% % Subplot 1: Real and Calculated Point
% subplot(2, 2, 1); % This creates a 2x2 grid and places the first plot in the first cell
% plot(time(1:index2-1), CalculatedPointSampled(1:index2-1,:)', 'r', 'LineWidth', 0.5);
% hold on;
% plot(time(1:index2-1), RealPointSampled(1:index2-1,:)', 'g', 'LineWidth', 0.5);
% title('Real and Calculated Point');
% legend('Calculated Point', 'Real Point');
% hold off;
% 
% % Subplot 2: Error CPF
% subplot(2, 2, 2); % Places the second plot in the second cell
% plot(time(1:index2-1), errorCPF(1:index2-1,:)', 'r', 'LineWidth', 0.5);
% title('Error CPF');
% legend('Error CPF');
% 
% % Subplot 3: Real and Calculated Point in World Frame
% subplot(2, 2, 3); % Places the third plot in the third cell
% plot(time(1:index2-1), CalculatedPointSampledworldframe(1:index2-1,:)', 'r', 'LineWidth', 0.5);
% hold on;
% plot(time(1:index2-1), RealPointSampledworldframe(1:index2-1,:)', 'g', 'LineWidth', 0.5);
% title('Real and Calculated Point in World Frame');
% legend('Calculated Point World Frame', 'Real Point World Frame');
% hold off;
% 
% % Subplot 4: Error After All Process
% subplot(2, 2, 4); % Places the fourth plot in the fourth cell
% hold on
% plot(ErrorAfterCPF2', 'r', 'LineWidth', 0.5);
% title('Error After All Process');
% legend('Error After Process');
% 
% % Adjust layout
% sgtitle('Consolidated Data Analysis'); % Super title for the entire figure



end
end
end


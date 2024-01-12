%% Description
% in LWR_sim.m This code is for calcolare il residuo r e sigma per
% understand which link is subjected to force; ho calcolato il punto di applicazione
% using the residual for initialization in the CPF; infine ho
% realizzato the contact particle filter, which works well if the initial value is good,
% ho inoltre fatto in modo che the point calculated in the previous sample-instant is used
% in quello dopo assuming that the point is almost constant.
% Additionally, plots have been created to observe the particle behavior and the error trend
% rispetto al vero punto di applicazione.
%% INIT
clc;
close all;
clear all;
addpath 'Dynamics'
addpath 'Functions'
%% Hyperparameters to be set
f8=figure();
nn=1000;
cells0=cell(1,nn);
matrix0 = zeros(nn, 7);
matrix00 = zeros(nn, 3);
vector0=zeros(nn,1);
Residual_calculated=matrix0;
Sigma_calculated=vector0;
is_collided=vector0;
TauExtForce=matrix0;
Q_sampled=matrix0;
QD_sampled=matrix0;
QDD_sampled=matrix0;
TAU_sampled=matrix0;
B_sampled=cells0;
S_sampled= cells0;
g_sampled=cells0;
h_sampled=cells0;
% Here all the parameters to decide for the computetion
tf=10; % final time
MaxVelocity=100;
% External force applied to the link LINK in a point POINT in respect to
% the reference frame of the link
ExternalForceApplied=[1 2 3]';
ExternalForceAppliedActualFrame=[2 4 3]';
gain = 10*diag([100, 60, 160, 200, 120, 80, 125]); %gain for the residual calculation
% real link subjected to the force
radius=0.05; % radius of the circle that the End Effector(EE) of the robot must follow
%q0(1:7)=[0,0,pi/2,0,0,0,0]; % q initialized, the real q initialized is in
% parameters.m
frequency = 1; %frequency*time is the angle of the point of the circle
% in particular frequency represent the angular velocity that the EE must
% follow
threshold_Collision = 0.002; % threshold in order to calculate the residuals, if <0.1 the link is influenced by an external force
threshold_sigma= 0.014; % threshold for the sigma calculated
samples=300; % samples to calculate the residual, in the other projects I think more than 300 samples are used
% but it depends by the sampling time, since the residual calculation start
% after samples instant
%% Initialization
PlotMeshRobot; % plot of all the meshes that represent the manipulator
hold on
for link=1:7
random_index = randi([1, 556]);
T = QtoP(q0(1:7),link);
random_vector=matrix(430,:,link);
random_noise = 0.005 * (2 * rand(1, 3) - 1);
point = (random_vector(1:3))';
Point_intersected=[0 0 0];
n = 7; %number of joints
Bsampled = cell(1, 1000);
Residual_calculated=zeros(100,7);
%Gain to calculate the residual, is high enough the residuals are equal to
%the extarnal torque due to the external force
double T;
link_collided=zeros(1,1000);
NumberNonCollidedSamples=0;
fval = [];
masses = load("models\masses.txt");
CoMs = load("models\center_of_masses.txt");
gravity=9.81;
Potentialenergy=0;
sumTau = zeros(n, 1);
syms lambda
sumH = zeros(7, 1);
is_initialized = false;
sumRes = zeros(n, 1);
R = zeros(samples, n);
F_EXT_RECON = zeros(samples, 3);
TAU_FRICTION = zeros(samples, n);
Niterations=5;
speed=1;
num_part=20;
chi = zeros(3,num_part);
Jtranspose_link=zeros(7,3);
index = 1;
%save the variables with a specific name
%filename="Force"+ExternalForceApplied(1)+ExternalForceApplied(2)+ExternalForceApplied(3)+"Point"+point(1)+point(2)+point(3)+"Link"+link+".mat";
parameters;
%% Additional task to add to the null space of the principal task
% I tried to mantain the z axis of the end effector constant
% syms t QQ(t) Q1(t) Q2(t) Q3(t) Q4(t) Q5(t) Q6(t) Q7(t)
%  T = QtoP(q0(1:7),7);
%  p_0 = T(1:3,4)+[0; 0; 0.31];
% QQ(t) =[ Q1(t) Q2(t) Q3(t) Q4(t) Q5(t) Q6(t) Q7(t)];
%FinalOrientation = vpa(simplify(T(1:3,1:3)),3); % final orientation at
%every time instant in order to add the final orientation
% T_Q = QtoP(QQ(t),7);
% diff(T,t);                                                                                                                                          cos(Q6(t))*(cos(Q2(t))*cos(Q4(t)) + cos(Q3(t))*sin(Q2(t))*sin(Q4(t))) + sin(Q6(t))*(cos(Q5(t))*(cos(Q2(t))*sin(Q4(t)) - 1.0*cos(Q3(t))*cos(Q4(t))*sin(Q2(t))) + sin(Q2(t))*sin(Q3(t))*sin(Q5(t)))
% J5= jacobian(norm(-T_Q(1:3,3)),QQ(t));
triangles=zeros(3,3,size(MeshesConnectivityList{link},1));
for i=1:size(MeshesConnectivityList{link},1)
for j=1:3
triangles(:,j,i)=Meshes.Points(MeshesConnectivityList{link}(i,j),1:3,link)';
end
end
% Label the axes
[closest_point,normal] = closest_point_to_triangle3(triangles, point');
error=norm(closest_point-point');
theta = 20 * pi / 180; % Convert 20 degrees to radians
% Normalize the rotation axis (normal vector)
axis = normal' / norm(normal);
% Create a skew-symmetric matrix from axis
K = [0 -axis(3) axis(2); axis(3) 0 -axis(1); -axis(2) axis(1) 0];
% Create the rotation matrix using Rodrigues' formula
R = eye(3) + sin(theta) * K + (1 - cos(theta)) * K^2;
% Translate the point to the origin for rotation
translated_point = closest_point - normal';
% Rotate the point
rotated_point = R * translated_point';
% Adjust the norm of the rotated vector
scaled_rotated_point = 10 * rotated_point / norm(rotated_point);
% Translate the point back
ExternalForceAppliedActualFrame = -(scaled_rotated_point' + normal')';
S_fext =[0 -ExternalForceAppliedActualFrame(3) ExternalForceAppliedActualFrame(2) ; ExternalForceAppliedActualFrame(3) 0 -ExternalForceAppliedActualFrame(1) ; -ExternalForceAppliedActualFrame(2) ExternalForceAppliedActualFrame(1) 0 ];
m=double(-S_fext*point(1:3));
%% Now there is the matlab simulation of the movement
%     hold on
%     force_vector_4d=[ExternalForceAppliedActualFrame;1];
%
%
%     transformed_vector_3d = (T * force_vector_4d)/50;
%     pointWorld=T*[random_vector]';
%     pointworld2=matrix2(430,:,link);
%     scatter3(pointWorld(1), pointWorld(2), pointWorld(3),'b')
%
%
%     scatter3(pointworld2(1), pointworld2(2), pointworld2(3),'r')
%     quiver3(pointWorld(1), pointWorld(2), pointWorld(3), transformed_vector_3d(1), transformed_vector_3d(2), transformed_vector_3d(3), 'r',LineWidth=0.1);
%     %plotFrictionCone;
disp('The point in the actual frame is:')
disp(vpa(point',3));
CalculatedPoint=[0 0 0];
gainE=100;
filename = ['Initialization\initializations', num2str(link), '.mat']
save(filename)
figure;
hold on;
grid on;
axis equal;
% Iterate through each triangle and plot
for i = 1:size(triangles, 3)
% Extract the i-th triangle
triangle = triangles(:, :, i);
% Plot each edge of the triangle
% Adding the first point again at the end to close the triangle
plot3([triangle(1, 1), triangle(1, 2)], ...
[triangle(2, 1), triangle(2, 2)], ...
[triangle(3, 1), triangle(3, 2)], 'b');
plot3([triangle(1, 2), triangle(1, 3)], ...
[triangle(2, 2), triangle(2, 3)], ...
[triangle(3, 2), triangle(3, 3)], 'b');
plot3([triangle(1, 3), triangle(1, 1)], ...
[triangle(2, 3), triangle(2, 1)], ...
[triangle(3, 3), triangle(3, 1)], 'b');
end
normal_scaled=(normal./5)';
starting_point=closest_point-normal_scaled;
scatter3(closest_point(1),closest_point(2),closest_point(3))
quiver3(starting_point(1),starting_point(2),starting_point(3),normal_scaled(1),normal_scaled(2),normal_scaled(3),'r','AutoScale','off')
normal_scaled=(ExternalForceAppliedActualFrame./100)';
starting_point=closest_point-normal_scaled;
scatter3(closest_point(1),closest_point(2),closest_point(3))
quiver3(starting_point(1),starting_point(2),starting_point(3),normal_scaled(1),normal_scaled(2),normal_scaled(3),'y','AutoScale','off')
end
%plotTriangles;
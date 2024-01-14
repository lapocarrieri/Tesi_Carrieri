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
linkforce=5;
ss = 1;
Sigma=0;
n=10000;
time=zeros(n,1);
firstTime=true;
addpath 'Dynamics'
addpath 'Functions'
%% Hyperparameters to be set
%% Now there is the matlab simulation of the movement
load(['Initialization\initializations', num2str(linkforce), '.mat'])
tf=10;
disp('The point in the actual frame is:')
disp(vpa(point',3));
close all
chi2 = zeros(3,20);
%% Figures
% f1 is the EE point actual point and reference point
f1=figure();
%T0=QtoP(q0(1:7) ,link);p_0=T0(1:3,4);
p_0 = f(q0(1),q0(2),q0(3),q0(4),q0(5),q0(6))
figure(f1),scatter3(p_0(1), p_0(2), p_0(3), 'filled');
firstcollision=1;
set(f1, 'Name', 'End effector points');
axis equal
hold on
% Assuming the center of the circle is on the x-axis for simplicity
% You can choose any point at a distance of 'radius' from p_0 in the xy-plane
center_x = p_0(1) - radius;
center_y = p_0(2);
center_z = p_0(3);  % Same z-coordinate as p_0
% Create theta values
theta = linspace(0, 2*pi, 100);
% Circle in the xy-plane
x = radius * cos(theta) + center_x;
y = radius * sin(theta) + center_y;
z = ones(size(x)) * center_z;  % constant z value
% Plotting the circle in 3D
plot3(x, y, z);
xlabel('X');
ylabel('Y');
zlabel('Z');
%f2 is the link collided and the line intersection to calculate the point
%through pseudoinversion
f2=figure;
%f3 is the robot plot
f3=figure();
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
%% Parameters

rateCtrlObj = rateControl(10000);
CalculatedPoint=[0 0 0];
gainE=100;
gain=gain;
DeltaT = 0.01; % sampling time of the robot
GainInv=inv(eye(7)+gain*DeltaT) * gain ;
GainEInv=inv(eye(1)+gainE*DeltaT) * gainE ;
sumTau=0;
sumH=0;
sumRes=0;
p0=0;
initial = 1;
err_f=[0;0;0];
sumEdot=0;
sumSigma=0;
index=index-1;
acc=zeros(1,7);
%Kp = [100,0,0;0,100,0;0,0,400]*1;
%Kp = [10,0,0;0,10,0;0,0,400]*1;
Kp = [1000,0,0;0,1000,0;0,0,1000]
Kd = [10,0,0;0,10,0;0,0,50]
%Kd = [100,0,0;0,100,0;0,0,50]*1;
%Kd=0;
Point_intersected=Point_intersected';
Point_intersectedActualFrame=Point_intersected;
tic
ErrorBeforeCPF_ActualFrame=0;
F_applied=ExternalForceAppliedActualFrame;
initial_position=Point_intersected;
dp=dp_0;
d2p_ref=d2p_0;
while (toc<300)%(frequency * (t0) < 2*pi) % it ends when a circle is completed
disp(t0);
F_applied=ExternalForceAppliedActualFrame;
index = index + 1;

% figure(f3);
% hold off
% if index>1
% prova = kuka.show(Q_sampled(index-1,:), 'visuals', 'on', 'collision', 'off');
% hold on
% % Adding text in the bottom right corner of the figure
% textString = sprintf('Force = [%0.3f, %0.3f, %0.3f]\npoint = [%0.3f, %0.3f, %0.3f]\nerror = %0.3f\nlink = %d', ...
% Point_intersectedActualFrame(1), Point_intersectedActualFrame(2), Point_intersectedActualFrame(3), ...
% point(1), point(2), point(3), ...
% ErrorBeforeCPF_ActualFrame,link);
% text(+0.5, 0.1,0.1, textString, 'HorizontalAlignment', 'left', 'VerticalAlignment', 'bottom', ...
% 'FontSize', 10, 'Color', 'black');
% view(135, 69);
% camzoom(3);
% 
% end
disp('time instant:')
disp(t0)
time(index)=t0;
q0(8:14) = acc * DeltaT + q0(8:14);
q0(1:7)  = q0(1:7) + q0(8:14) * DeltaT ;
figure(f1);
%dp = J * q0(8:14)';
%d2p = dJ * q0(8:14)' + J * acc
dp_0(2)=-0.05;
%% TRAJ (CIRCLE)
p_ref(1,1) = p_0(1) -  radius * (1 - cos(frequency * (t0)));
dp_ref(1,1) = dp_0(1) - radius * frequency * sin(frequency * (t0));
d2p_ff(1,1) = d2p_0(1) - radius * frequency * frequency * cos(frequency * (t0));
p_ref(2,1) = p_0(2) -  radius * (sin(frequency * (t0)));
dp_ref(2,1) = dp_0(2) - radius * frequency * cos(frequency * (t0));
d2p_ff(2,1) = d2p_0(2) + radius * frequency * frequency * sin(frequency * (t0));
p_ref(3,1) = p_0(3);
dp_ref(3,1) = dp_0(3);
d2p_ff(3,1) = d2p_0(3);
hold on
    

p = f(q0(1),q0(2),q0(3),q0(4),q0(5),q0(6));
figure(f1),scatter3(p(1,1),p(2,1),p(3,1),'g','SizeData',20)
hold on
figure(f1),scatter3(p_ref(1,1),p_ref(2,1),p_ref(3,1),'r','SizeData',20)
%% Change in the code if we want that a certain point make a specifi trajectory and not the EE
%     p = T(1:3,4)+[0; 0; 0.31];
%
%
%     [J,Jsubs] = compute_jacobian(q0(1:7),0,7);
%
%
%
%     dJ = compute_derivative_jacobian(Jsubs,q0);
%% EE point to set the trajectory to follow
% EE point calculated with f
%now if the force is applied to the end effector a faster comutation is
%used so J_LWR

J = J_LWR(q0(1),q0(2),q0(3),q0(4),q0(5),q0(6));
    
    dJ = dJdt_LWR(q0(8),q0(9),q0(10),q0(11),q0(12),q0(13),q0(1),q0(2),q0(3),q0(4),q0(5),q0(6));%[~,S,~] = svd(J);
%sigma_trajectory = min(diag(S));
dp = J * q0(8:14)';
prefsampled(index,:)=p_ref;
    psampled(index,:)=p;
    dprefsampled(index,:)=dp_ref;
    dpsampled(index,:)=dp;
err_p = p_ref-p;
err_dp = dp_ref-dp ;
%% PD KINEMATIC
d2p_ref = Kp * err_p + Kd * err_dp + d2p_ff;%

tspan =[t0,t0+DeltaT];
% EXACT PROJECTOR:
Pj = eye(7) - pinv(J) * J;

%% TO BE FIXED
    friction =  0.1*(-A_friction .* sign(q0(8:14)) - 0.001 * q0(8:14));
friction =  0.0;

%% PSEUDOINVERSE
Uref = (pinv(J)* (d2p_ref - dJ * q0(8:14)'))';


%% to avoid breakdown due to a too high velocity impressed in the system :
%
%     for ii=8:14
%         if q0(ii)>MaxVelocity
%          fprintf("Velocity bound passed");
% %
% %         break
%          q0(ii)=MaxVelocity;
%
%         end
%     end
%% Dynamics
B=get_Bnum(q0(1:7));
g=get_gnum(q0(1:7));
S=get_Snum(q0(1:7),q0(8:14));
%another option is use get_Bnum,Snum ecc but they are slower
%          B1 = massMatrix(controller,q0(1:7));
%      Sd1= velocityProduct(controller,q0(1:7),q0(8:14));
%     g=gravityTorque(controller,q0(1:7))';
%% Force application
[J_force,~] = compute_jacobian(q0(1:7),point,link);
T= QtoP(q0(1:7),link);
RealPointIntersectedWorldFrame=T*[point;1];
J_actualframe=T(1:3,1:3)'*J_force;
J_w = ComputePoint_withWrenches(q0(1:7),link);
S_fext =skew_symmetric(F_applied);
m=double(-S_fext*point(1:3));
TauExternalForce=(J_w'*[F_applied;m])';
ExternalForceAppliedActualFrameSampled(index,:)=F_applied;
% 
% %          if index<60
% %                  TauExternalForce=[0 0 0 0 0  0 0];
% %          end
%% COMPUTED TORQUE FL - dynamic model
%TauExternalForce=[0 0 0 0 0  0 0];
Kf=0;
Kt=0.9;
scaling_factor=1;
Uref=scaling_factor*Uref;
TauFL = (g + S*q0(8:14)' + B * Uref')'-Kt*Residual_calculated(index,:);  % this is the applied torque
Tau = TauFL+TauExternalForce; % this is the real tau applied
acc = (inv(B)* (Tau' - friction - S*q0(8:14)' - g))';
t0 = t0+DeltaT;

% % Impedance
%   % Calculate Force Error
%     err_f = F_desired - force_current;
% 
% 
%     % Impedance Control Law
%     F_control = Md * d2p_ref + Bd * dq_current + Kd * q_current;
%     Tau = F_control + Kf * err_f;
%     % Impedance Control Law
%     % Compute the desired end-effector acceleration based on the dynamic impedance model
%     a = ddr_d + inv(Md) * (Dd * (dr - dr_d) + Kd * (r - r_d) + F_ext);
% 
%     % Calculate the control input 'u' using feedback linearization
%     u = Jr(q_current)' * (Mr(q_current) * a + Sr(q_current, dq_current) * dq_current + gr(q_current) - F_ext);
% 
% %Admittance
% d2q_desired = inv(Md) * (F_external - Bd * dq_current - Kd * q_current);
% EULER INTEGRATION
pk=NaN(7);
if q0(1:7)==pk
return;
end

%% save all the variables needed for the force reconstruction and the calculation of the residuals
TauExtForce(index,:)=(TauExternalForce);
Q_sampled(index,:)=q0(1:7);
QD_sampled(index,:)=q0(8:14);
QDD_sampled(index,:)=acc;
TAU_sampled(index,:)=(TauFL');
B_sampled{index}=(B);
S_sampled{index}=S;
g_sampled{index}=g;
h_sampled{index}=(S'*q0(8:14)'-g);
%% Residual Calculation and Force and point reconstruction
fprintf('Starting of the Residual calculation:\n')

%% Momentum-based isolation of collisions

t= index;
h=(S_sampled{t})'*QD_sampled(t,:)'-g_sampled{t};
sumTau = sumTau + TAU_sampled(t,:)';
sumH = sumH + h_sampled{t};
if initial == 1
p0 = B_sampled{t}*QD_sampled(t, :)';
initial=2;
r = zeros(7,1);
else
r =inv(eye(7)+gain*DeltaT) * gain * ((B_sampled{t}*QD_sampled(t, :)' - p0) - (sumTau + sumH + sumRes)*DeltaT);
sumRes = sumRes +r;
end
%% Point estimation - initialization of the contact particle filter
errorTorque=vpa(norm(r-TauExtForce(index,:)'),2);
Residual_calculated(index,:)=r;
is_collided_sigma=1;
[link_collided(index),is_collided_r] = getLink_withResidual(r,threshold_Collision);
is_collided(index)=0;
if is_collided_r==1 && is_collided_sigma==1
is_collided(index)=1;
end
if is_collided(index)==0
link_collided(index)=0;
end
LinkInCollision=link_collided(index);
if is_collided(index) == 1
%J_withwrenches = ComputePoint_withWrenches(Q_sampled(index,:),LinkInCollision);
J_withwrenches=J_w;
WeightCalculation;
W2=diag(weights);
mu=0.01;
J_withwrenchesweightedPseudoinverse5=inv(W2)*J_withwrenches*inv(J_withwrenches'*inv(W2)*J_withwrenches+mu^2*eye(7));
wrenches5=J_withwrenchesweightedPseudoinverse5*Residual_calculated(index,:)';
error5 = [F_applied;m]-wrenches5;
%% Point calculation through f and m
wrenches5=pinv(J_withwrenches')*Residual_calculated(index,:)';
error1 = [F_applied;m]-wrenches5;
%wrenches3=[ExternalForceAppliedActualFrame;m];
f_i=wrenches5(1:3);
f_i_sampled(index,:)=f_i;


m_i=wrenches5(4:6);
Sf_i=[0 -f_i(3) f_i(2) ; f_i(3) 0 -f_i(1) ; -f_i(2) f_i(1) 0 ];
p_dc=Sf_i*m_i/(norm(f_i))^2;
Lambda_coefficient=f_i/(norm(f_i))^2;
line.origin=T*[p_dc;1];
line.origin=double(line.origin(1:3));
line.direction=double(T*[Lambda_coefficient;1]);
line.direction=line.direction(1:3);
RealPointIntersected=double(T*[point;1]);
hold on
figure(f2);
hold off
Point_intersected = IntersectionPoint(line,link,Point_intersected(1:3),Meshes,T);
disp('Point_intersected')
if isempty(Point_intersected)
Point_intersected_actual_frame=closest_point_to_triangle3(triangles, p_dc');
Point_intersected=T*[Point_intersected_actual_frame';1];
disp('point initialiazation not optimal')
end
Point_intersected=double(Point_intersected(1:3)');
NumberNonCollidedSamples=0;
if firstcollision==1
    initial_position=Point_intersected;
    initial_force=f_i;
    firstcollision=0;
end
if link_collided(index)>0
err_f=f_i-initial_force;
end
if size(Point_intersected,1)==size(initial_position,1)
displacement = Point_intersected - initial_position;
else
displacement = Point_intersected' - initial_position;
end
distance = norm(displacement);
F_max = 100;
F_min=0;
if distance > 0
    direction_of_displacement = displacement / distance;
else
    direction_of_displacement = [0; 0; 0];
end
dot_product = dot(initial_force/norm(initial_force), direction_of_displacement);

F_applied = initial_force * (1 - dot_product * distance);
F_applied = max(F_min, min(norm(F_applied), F_max)) * (initial_force/norm(initial_force));
fapplieds(index,:)=F_applied;

else
NumberNonCollidedSamples=NumberNonCollidedSamples+1;



end
if NumberNonCollidedSamples>10
chi = zeros(3,num_part);
chi2 = zeros(3,num_part);
end
%% CONTACT PARTICLE FILTER
ind=1;
if size(Point_intersected,1)>1
Point_intersected=Point_intersected';
end
Point_intersectedActualFrame=double(inv(T)*[Point_intersected';1]);
ErrorBeforeCPF_ActualFrame=norm(RealPointIntersectedWorldFrame(1:3)-Point_intersected');
disp('error before CPF:')
disp(vpa(norm(ErrorBeforeCPF_ActualFrame),3))

save(['sharedDatas\sharedData',num2str(linkforce)],'point', 'link_collided','index','chi','Q_sampled','Residual_calculated','Point_intersectedActualFrame','TauExternalForce');
CalculatedPoint=Point_intersectedActualFrame(1:3)';
Rotation = T(1:3,1:3);
tran = T(1:3,4);
load(['sharedDatas\sharedVar', num2str(linkforce)])

disp(norm(CalculatedPoint(1:3)'-point));
contact_point_PF = Rotation*CalculatedPoint'+tran;
disp("error torquw")
errorTorque

%% Collaboration
end
save("plotDatas1")

figure()
% Plot the external force in red
plot(time(1:index-1), TauExtForce(1:index-1,:)', 'r', 'LineWidth', 0.5);
hold on;
plot(time(1:index-1), Residual_calculated(1:index-1,:)', 'g', 'LineWidth', 0.5);
% Plot the residuals in greenplot(time(1:index-1), Residual_calculated(1:index-1,:), 'g', 'LineWidth', 0.5);

% Adding title and axis labels
title('External Force and Residual Over Time');
xlabel('Time (s)'); % Assuming the time variable is in seconds
ylabel('Magnitude'); % Replace with a more specific label if needed

% Create

figure()
plot(time(1:index-1), prefsampled(1:index-1,:)', 'r', 'LineWidth', 0.5);
hold on;
plot(time(1:index-1), psampled(1:index-1,:), 'g', 'LineWidth', 0.5);
title('Position error');
xlabel('Time (s)'); % Assuming the time variable is in seconds
ylabel('Magnitude'); % Replace with a more specific label if needed

figure()
plot(time(1:index-1), dprefsampled(1:index-1,:)', 'r', 'LineWidth', 0.5);
hold on;
plot(time(1:index-1), dpsampled(1:index-1,:), 'g', 'LineWidth', 0.5);
title('Velocity');
xlabel('Time (s)'); % Assuming the time variable is in seconds
ylabel('Magnitude'); % Replace with a more specific label if needed

% Optionally, add a legend if it helps interpret the plots
legend('Reference Velocity', 'Actual Velocity');
figure()
plot(time(1:index-1), Q_sampled(1:index-1,:)', 'r', 'LineWidth', 0.5);
title('Joint position');
xlabel('Time (s)'); % Assuming the time variable is in seconds
ylabel('Magnitude'); % Replace with a more specific label if needed

figure()
plot(time(1:index-1), QD_sampled(1:index-1,:)', 'r', 'LineWidth', 0.5);
title('Joint Velocities');
xlabel('Time (s)'); % Assuming the time variable is in seconds
ylabel('Magnitude'); % Replace with a more specific label if needed


% Assuming time, ExternalForceAppliedActualFrameSampled, f_i_sampled, and initial_force are defined

% Total number of subplots
nSubplots = 3;

% Y-axis limits
yAxisLimits = [-20, 20];

% First subplot - Plotting first component of both f_i_sampled and ExternalForceAppliedActualFrameSampled
subplot(nSubplots, 1, 1);
plot(time(1:index-1), ExternalForceAppliedActualFrameSampled(1:index-1,1), 'r', 'LineWidth', 0.5);
hold on;
plot(time(1:index-1), f_i_sampled(1:index-1,1), 'g', 'LineWidth', 0.5);
yline(initial_force, 'b', 'LineWidth', 0.5);
ylim(yAxisLimits);
title('First Component: External Force vs. Calculated Force Over Time');
xlabel('Time (s)');
ylabel('Magnitude');
hold off;

% Second subplot - Plotting second component of both f_i_sampled and ExternalForceAppliedActualFrameSampled
subplot(nSubplots, 1, 2);
plot(time(1:index-1), ExternalForceAppliedActualFrameSampled(1:index-1,2), 'r', 'LineWidth', 0.5);
hold on;
plot(time(1:index-1), f_i_sampled(1:index-1,2), 'g', 'LineWidth', 0.5);
yline(initial_force, 'b', 'LineWidth', 0.5);
ylim(yAxisLimits);
title('Second Component: External Force vs. Calculated Force Over Time');
xlabel('Time (s)');
ylabel('Magnitude');
hold off;

% Third subplot - Plotting third component of both f_i_sampled and ExternalForceAppliedActualFrameSampled
subplot(nSubplots, 1, 3);
plot(time(1:index-1), ExternalForceAppliedActualFrameSampled(1:index-1,3), 'r', 'LineWidth', 0.5);
hold on;
plot(time(1:index-1), f_i_sampled(1:index-1,3), 'g', 'LineWidth', 0.5);
yline(initial_force, 'b', 'LineWidth', 0.5);
ylim(yAxisLimits);
title('Third Component: External Force vs. Calculated Force Over Time');
xlabel('Time (s)');
ylabel('Magnitude');
hold off;



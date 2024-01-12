%% INIT
clc;
close all;
clear all;
addpath 'Dynamics'
addpath 'Functions'
%% parameters
% Here all the parameters to decide for the computetion
tf=10; % final time
MaxVelocity=100;
acc=[0 0 0 0 0 0 0]';
% External force applied to the link LINK in a point POINT in respect to
% the reference frame of the link
ExternalForceApplied=[3 3 3]';
ExternalForceAppliedActualFrame=[1 1 1]';
S_fext =[0 -ExternalForceAppliedActualFrame(3) ExternalForceAppliedActualFrame(2) ; ExternalForceAppliedActualFrame(3) 0 -ExternalForceAppliedActualFrame(1) ; -ExternalForceAppliedActualFrame(2) ExternalForceAppliedActualFrame(1) 0 ];
if norm(ExternalForceAppliedActualFrame)>0
collision=1;
end
radius=0.05;
n = 7; %number of joints
Bsampled = cell(1, 1000);
Residual_calculated=zeros(100,7);
%Gain to calculate the residual, is high enough the residuals are equal to
%the extarnal torque due to the external force
gain = 1000*diag([100, 60, 160, 200, 120, 80, 125]);
S=zeros(7,7);
DeltaT = 0.01; % sampling time
%q0(1:7)=[0,0,0,0,0,0,0]; % initial qs
frequency = 1; %frequency*time is the angle of the point of the circle
threshold_Collision = 0.05; % threshold in order to calculate the residuals, if <0.1 the link is influenced by an external force
threshold_sigma= -0.2;
samples=20; % samples to calculate the residual
%% Initialization
masses = load("models\masses.txt");
CoMs = load("models\center_of_masses.txt");
gravity=9.81;
Potentialenergy=0;
sumTau = zeros(n, 1);
syms lambda
sumH = zeros(7, 1);
sumRes = zeros(n, 1);
R = zeros(samples, n);
TAUnew = zeros(samples,n);
F_EXT_RECON = zeros(samples, 3);
TAU_PREDICT = zeros(samples, n);
TAU_FRICTION = zeros(samples, n);
initial_idx = 1;
final_idx = 10;
speed = 1;
num_part=20;
radii = 0.05;           % radii of cyl
cnt = [0, 0,0];       % [x,y,z] center cyl
height = -0.1;      % height of cyl, is negative because the frame is on the top
[X,Y,Z] = cylinder(radii);
X = X + cnt(1) ;
Y = Y + cnt(2) ;
Z = Z * height;
chi = zeros(3,num_part);
pointZ=-0.07;
theta=pi/3;
point(3)=pointZ+ cnt(3);
point(1)=radii*cos(theta)+ cnt(1);
point(2)=radii*sin(theta) +  cnt(2);
point=point'
m=-S_fext*point(1:3)
point = [point;1]
index = 1;
%save the variables with a specific name
link=1;
filename="Force"+ExternalForceApplied(1)+ExternalForceApplied(2)+ExternalForceApplied(3)+"Point"+point(1)+point(2)+point(3)+"Link"+link+".mat";
parameters;
figure();
xlabel('X');
ylabel('Y');
zlabel('Z');
Jtranspose_link=zeros(7,3);
hold on;
plot3(p_0(1), p_0(2), p_0(3), 'bl');
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
%% Now there is the matlab simulation of the movement
nk=0;
tic
theta=pi/3;
ExternalForceApplied=[0 0 0]';
while (toc<16000)%(frequency * (t0) < 2*pi) % it ends when a circle is completed
disp(t0);
ExternalForceApplied=ExternalForceApplied+[0.01 0.02 0.03]';
link=5;
if toc<2000
link=2;
end
if (toc>2000 && toc<4000)
link=3;
end
if (toc>4000 && toc<6000)
link=4;
end
if (toc>6000 && toc<8000)
link=5;
end
if (toc>8000 && toc<12000)
link=6;
end
if (toc>12000 && toc<16000)
link=7;
end
%% TRAJ (CIRCLE)
radius = radius + DeltaT*(rand(1)-0.5);
p_ref(1,1) = p_0(1) -  radius * (1 - cos(frequency * (t0)));
dp_ref(1,1) = dp_0(1) - radius * frequency * sin(frequency * (t0));
d2p_ff(1,1) = d2p_0(1) - radius * frequency * frequency * cos(frequency * (t0));
p_ref(2,1) = p_0(2) -  radius * (sin(frequency * (t0)));
dp_ref(2,1) = dp_0(2) - radius * frequency * cos(frequency * (t0));
d2p_ff(2,1) = d2p_0(2) + radius * frequency * frequency * sin(frequency * (t0));
p_ref(3,1) = p_0(3)+DeltaT*(rand(1)-0.5) ;
dp_ref(3,1) = dp_0(3);
d2p_ff(3,1) = d2p_0(3);
%% Change in the code if we want that a certain point make a specifi trajectory and not the EE
%         T = QtoP(q0(1:7),7);
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
p = f(q0(1),q0(2),q0(3),q0(4),q0(5),q0(6));
J = J_LWR(q0(1),q0(2),q0(3),q0(4),q0(5),q0(6));
dJ = dJdt_LWR(q0(8),q0(9),q0(10),q0(11),q0(12),q0(13),q0(1),q0(2),q0(3),q0(4),q0(5),q0(6));
[~,S,~] = svd(J);
sigma = min(diag(S));
dp = J * q0(8:14)';
d2p = dJ * q0(8:14)' + J * accs(end,:)';
err_p = p_ref - p;
err_dp = dp_ref - dp;
plot3(p(1), p(2), p(3), 'ro', 'MarkerSize',2);
%% PD KINEMATIC
d2p_ref = Kp * err_p + Kd * err_dp + d2p_ff;
%d2p_ref = Kp * err_p + Kd * error_theta + d2p_ff;
tspan =[t0,t0+DeltaT];
% EXACT PROJECTOR:
Pj = eye(7) - pinv(J) * J;
% Friction calculation
%% TO BE FIXED
%     friction =  0.1*(-A_friction .* sign(q0(8:14)) - 0.001 * q0(8:14));
friction =  0.0;
%%DAMPED-LEAST SQUARE
%     A = (J' * J + damping * eye(length(q0(1:7))));
%     B = J' * (d2p_ref - dJ * q0(8:14)');
%     X1 = lsqminnorm(A,B);
%     Uref_task = X1';
%% PSEUDOINVERSE
Uref_task = (pinv(J) * (d2p_ref - dJ * q0(8:14)'))';
uopt = zeros(7,1);
%uopt = [0 0 0 0 1 1 1]';
%uopt = subs(J5,QQ(t),q0(1:7))';
Uref_redundant = (Pj * uopt)';
Uref = Uref_task + Uref_redundant;
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
%compute jacobian computes the jacobian in respect to the point
%application that is on the link-th link at a POINT distance
%from the reference frame
%dJ = compute_derivative_jacobian(Jsubs,q0);
%     ForcePointApplication = QtoP(q0(1:7),link);
%      ForcePointApplication=ForcePointApplication(1:3,4)+[0; 0; 0.31];
%     [J_link,~] = compute_jacobian(q0(1:7),[0.1,01,link);
%      Jtranspose_link(1:5,1:3) = transpose(J_link(1:3,:));
%
%TauExternalForce = (Jtranspose_link*ExternalForce)';
%size(TauExternalForce)
%external torque applied to the system
T_actualframe= QtoP(q0(1:7),link);
J_actualframe=T_actualframe(1:3,1:3)'*J_force;
%TauExternalForce = (transpose(J_actualframe) * ExternalForceAppliedActualFrame)';
TauExternalForce =(transpose(J_force)*ExternalForceApplied)';
%TauExternalForce=[0 0 0 0 0 0  0];
%% COMPUTED TORQUE FL - dynamic model
TauFL = (g + S*q0(8:14)' + B * Uref')';  % this is the applied torque
Tau = TauFL+TauExternalForce; % this is the real tau applied
acc = (inv(B) * (Tau' - friction - S*q0(8:14)' - g))'  ;
Tau_applied  = B*acc' + friction + S*q0(8:14)' + g;
accs = vertcat(accs,acc);
accs_ref = vertcat(accs_ref,[Uref,Uref_task, Uref_redundant]);
task_vec = vertcat(task_vec, [p',dp',d2p',p_ref',dp_ref',d2p_ref']);
torque_fl = vertcat(torque_fl,Tau);
singular_values = vertcat(singular_values,sigma);
% EULER INTEGRATION
q0(1:7)  = q0(1:7) + q0(8:14) * DeltaT ;
pk=NaN(7);
if q0(1:7)==pk
return;
end
q0(8:14) = acc * DeltaT + q0(8:14);
t0 = t0+DeltaT;
joints = vertcat(joints,q0);
time = vertcat(time,t0);
%% save all the variables needed for the force reconstruction and the calculation of the residuals
TauExtForce(index,:)=TauExternalForce;
Q_sampled(index,:)=q0(1:7);
PointOfApplication(index,:)=vpa(QtoP(Q_sampled(end,:),link)*point,2);
QD_sampled(index,:)=q0(8:14);
QDD_sampled(index,:)=acc;
TAU_sampled(index,:)=TauFL';
B_sampled{index}=B;
S_sampled{index}=S;
g_sampled{index}=g;
h_sampled{index}=S'*q0(8:14)'-g;
%% Residual Calculation and Force and point reconstruction
if index>samples % in order to have enough samples to calculate the residuals
sumTau=0;
sumH=0;
sumRes=0;
p0=0;
R = zeros(samples, 7);
gainE=100;
sumEdot=0;
sumSigma=0;
%% Momentum-based isolation of collisions
for tt = 1:samples+1
t=index-samples+tt-1;
h=(S_sampled{t})'*QD_sampled(t,:)'-g_sampled{t};
sumTau = sumTau + TAU_sampled(t,:)';
sumH = sumH + h;
if tt == 1
p0 = B_sampled{t}*QD_sampled(t, :)';
r = zeros(7,1);
else
r = inv(eye(7)+gain*DeltaT) * gain * ((B_sampled{t}*QD_sampled(t, :)' - p0) - (sumTau + sumH + sumRes)*DeltaT);
sumRes = sumRes + r;
R(t, :) = r';
end
end
%R = NoncausalButterworthFilter(R);
%% Energy-based detection
for tt = 1:samples+1
t=index-samples+tt-1;
sumEdot = sumEdot + QD_sampled(t,:)*(TAU_sampled(t,:)'- g_sampled{t});
if tt == 1
p0 = 1/2*QD_sampled(t, :)*B_sampled{t}*QD_sampled(t, :)';
sumSigma=0;
sigma = 0;
else
sigma = inv(eye(1)+gainE*DeltaT) * gainE * ((1/2*QD_sampled(t, :)*B_sampled{t}*QD_sampled(t, :)' - p0) - (sumEdot + sumSigma)*DeltaT);
sumSigma = sumSigma + sigma;
Sigma(t, :) = sigma;
end
end
%R = NoncausalButterworthFilter(R);
%% Point estimation - initialization of the contact particle filter
Tau_sigma = Sigma(end, :);
link_calculated(index)=link;
point_calculated(index,:)=point;
Tau_residual=R(end,:);
Residual_calculated(index,:)=Tau_residual;
Sigma_calculated(index,:)=Tau_sigma;
end
index = index + 1;
end
save('NNN1','Residual_calculated','Sigma_calculated','link_calculated','point_calculated' )
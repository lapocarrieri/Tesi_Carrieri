%% INIT
clc;
close all;
clear all;
    addpath 'C:\Users\lapoc\OneDrive\Desktop\MATLAB_Simulator\Dynamics'
    addpath 'C:\Users\lapoc\OneDrive\Desktop\MATLAB_Simulator\CPF'
parameters;
figure();
xlabel('X');
ylabel('Y');
zlabel('Z');
Jtranspose_link=zeros(7,3);
ExternalForce=[1 0 0]';
hold on;
% Plot initial point position
frequency = 3;
 plot3(p_0(1), p_0(2), p_0(3), 'bl');
 p_0;
 
 T = QtoP(q0(1:7),7);
 p_0 = T(1:3,4)+[0; 0; 0.31];
 %FinalOrientation = vpa(simplify(T(1:3,1:3)),3);
%% Additional task to add to the null space of the principal task
% I tried to mantain the z axis of the end effector constant
syms t QQ(t) Q1(t) Q2(t) Q3(t) Q4(t) Q5(t) Q6(t) Q7(t) 
QQ(t) =[ Q1(t) Q2(t) Q3(t) Q4(t) Q5(t) Q6(t) Q7(t)];
T_Q = QtoP(QQ(t),7);
diff(T,t);                                                                                                                                          cos(Q6(t))*(cos(Q2(t))*cos(Q4(t)) + cos(Q3(t))*sin(Q2(t))*sin(Q4(t))) + sin(Q6(t))*(cos(Q5(t))*(cos(Q2(t))*sin(Q4(t)) - 1.0*cos(Q3(t))*cos(Q4(t))*sin(Q2(t))) + sin(Q2(t))*sin(Q3(t))*sin(Q5(t)))
J5= jacobian(norm(-T_Q(1:3,3)),QQ(t));
 
%% Now there is the matlab simulation of the movement
while(t0 < tf)
     disp(t0);  
     

    
    %% TRAJ (CIRCLE)

    p_ref(1,1) = p_0(1) -  0.05 * (1 - cos(frequency * (t0)));
    dp_ref(1,1) = dp_0(1) - 0.05 * frequency * sin(frequency * (t0));
    d2p_ff(1,1) = d2p_0(1) - 0.05 * frequency * frequency * cos(frequency * (t0));

    p_ref(2,1) = p_0(2) -  0.05 * (sin(frequency * (t0)));
    dp_ref(2,1) = dp_0(2) - 0.05 * frequency * cos(frequency * (t0));
    d2p_ff(2,1) = d2p_0(2) + 0.05 * frequency * frequency * sin(frequency * (t0));
    
    p_ref(3,1) = p_0(3) ;
    dp_ref(3,1) = dp_0(3);
    d2p_ff(3,1) = d2p_0(3);
     
    Jold = J;

    qold = q0;
   
    T = QtoP(q0(1:7),7);       
    p = T(1:3,4)+[0; 0; 0.31];
    %p = f(q0(1),q0(2),q0(3),q0(4),q0(5),q0(6));
  
%     [J,Jsubs] = compute_jacobian(q0(1:7),0,7);
% 
%     
%     
%     dJ = compute_derivative_jacobian(Jsubs,q0);
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

    
    %% EXACT PROJECTOR
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
    link=5;
    
%     B=get_Bnum(q0(1:7));
% 
%     S=get_Snum(q0(1:7),q0(8:14));
%     g=get_gnum(q0(1:7));

    B=massMatrix(controller,q0(1:7));
    Sd=velocityProduct(controller,q0(1:7),q0(8:14))';
    g=gravityTorque(controller,q0(1:7))';

%     ForcePointApplication = QtoP(q0(1:7),link);
%      ForcePointApplication=ForcePointApplication(1:3,4)+[0; 0; 0.31];
%      [J_link,~] = compute_jacobian(q0(1:7),0,link);
%      Jtranspose_link(1:5,1:3) = transpose(J_link(1:3,:));
%     

     %TauExternalForce = (Jtranspose_link*ExternalForce)';
     %size(TauExternalForce)
    [J,~] = compute_jacobian(q0(1:7),0,link);
     TauExternalForce =(transpose(J)*ExternalForce)';
    
  
   
    TauFL = (g + Sd + B * Uref')';                
    Tau = TauFL+TauExternalForce;

    
    %% COMPUTED TORQUE FL
   

    acc = (inv(B) * (Tau' - friction - Sd - g))';      
    
    %Tau_applied  = ((massMatrix(controller,q0(1:7))*acc')' + friction + velocityProduct(controller,q0(1:7),q0(8:14)) + gravityTorque(controller,q0(1:7)));  
  
    Tau_applied  = ((B*acc')' + friction + (Sd)' + g')';  

   % TAU_reconstructed(index,:)=(B*accs')' + friction + (S)*q0(8:14)')' + g')';
%          B1 = massMatrix(controller,q0(1:7));
%       
%          B2 = get_Bnum(q0(1:7));
%          
        
% 
%      errorB =B1-B2
%      Sd1= velocityProduct(controller,q0(1:7),q0(8:14));
%      Sd2 = get_Snum(q0(1:7),q0(8:14));
%      errorSd = Sd1-Sd2



    accs = vertcat(accs,acc);
    
    accs_ref = vertcat(accs_ref,[Uref,Uref_task, Uref_redundant]);            
    
    task_vec = vertcat(task_vec, [p',dp',d2p',p_ref',dp_ref',d2p_ref']);
    
    torque_fl = vertcat(torque_fl,Tau);
    
    singular_values = vertcat(singular_values,sigma);       
    
    % EULER INTEGRATION    
    q0(1:7)  = q0(1:7) + q0(8:14) * DeltaT;    
    q0(8:14) = acc * DeltaT + q0(8:14);               

    t0 = t0+DeltaT;
    
    joints = vertcat(joints,q0);    
    
    time = vertcat(time,t0);
    TauExtForce(index,:)=TauExternalForce;
    Q(index,:)=q0(1:7);
    QD(index,:)=q0(8:14);
    QDD(index,:)=acc;
    J_c(index,:,:)=J;
    TauReal(index,:)=vpa(Tau,3);
    TAU(index,:)=Tau_applied;
    
    B=massMatrix(controller,q0(1:7));
    Sd=velocityProduct(controller,q0(1:7),q0(8:14))';
    g=gravityTorque(controller,q0(1:7))';
    
   
     %TAU_reconstructed(index,:)=vpa(B*QDD(index,:)' + friction + Sd + g)';
   

    

    %error_TAU =vpa( TAU_reconstructed(index,:)-TAU(index,:),3)
    %ForcePointApplication_saved(index,:)=ForcePointApplication;
    %Updating the step
    index = index + 1; 

 

    
end
samples=index-1;
 filename = "ExternalForce5link100.mat";
 save(filename)
 robot_plot;
 return;



main_Carrieri;

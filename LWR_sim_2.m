%% INIT
clc;
close all;
clear all;

parameters;
figure();
 plot3(-0.025, -0.17, 0.5, 'b');
hold on;
% Plot initial point position
 %plot3(p_0(1), p_0(2), 1, 'bl');
while(t0 < tf)
%     disp(t0);  

    %% TRAJ (CIRCLE)
    
    p_ref(1,1) = p_0(1) -  0.1 * (1 - cos(frequency * (t0)));
    dp_ref(1,1) = dp_0(1) - 0.1 * frequency * sin(frequency * (t0));
    d2p_ff(1,1) = d2p_0(1) - 0.1 * frequency * frequency * cos(frequency * (t0));

    p_ref(2,1) = p_0(2) -  0.1 * (sin(frequency * (t0)));
    dp_ref(2,1) = dp_0(2) - 0.1 * frequency * cos(frequency * (t0));
    d2p_ff(2,1) = d2p_0(2) + 0.1 * frequency * frequency * sin(frequency * (t0));
    
    p_ref(3,1) = p_0(3) ;
    dp_ref(3,1) = dp_0(3);
    d2p_ff(3,1) = d2p_0(3);
     
    Jold = J;

    qold = q0;
    
    p = f(q0(1),q0(2),q0(3),q0(4),q0(5),q0(6));         
    
    %J1 = J_LWR(q0(1),q0(2),q0(3),q0(4),q0(5),q0(6))
    [J,dJ] = compute_jacobian(q0(1:14),0,7);
    J=J(1:3,:);
    dJ=dJ(1:3,:);
    
    
  
    [~,S,~] = svd(J);
    
    sigma = min(diag(S));
    
    dp = J * q0(8:14)';
    
    d2p = dJ* q0(8:14)' + J  * accs(end,:)';
    err_p = p_ref - p;
  
    err_dp = dp_ref - dp;
    
    
    plot3(p(1), p(2), p(3), 'ro', 'MarkerSize',2);
    %% PD KINEMATIC  
    d2p_ref = Kp * err_p + Kd * err_dp + d2p_ff;
    
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
    Uref_task = (pinv(J(1:3,:)) * (d2p_ref - dJ(1:3,:) * q0(8:14)'))';
    
    uopt = zeros(7,1);
    
    Uref_redundant = (Pj * uopt)';
    
    Uref = Uref_task + Uref_redundant;
    Jtranspose = transpose(J);
    TauExternalForce = (Jtranspose*[0 0 10]')';
    TauFL = gravityTorque(controller,q0(1:7)) + velocityProduct(controller,q0(1:7),q0(8:14)) + (massMatrix(controller,q0(1:7)) * Uref')';                
    Tau = TauFL+TauExternalForce;
    M = massMatrix(controller,q0(1:7));
    
    n = gravityTorque(controller,q0(1:7)) + velocityProduct(controller,q0(1:7),q0(8:14));
        
    %% COMPUTED TORQUE FL
    
    acc = (inv(massMatrix(controller,q0(1:7))) * (Tau - friction - velocityProduct(controller,q0(1:7),q0(8:14)) - gravityTorque(controller,q0(1:7)))')';      
   
    %Tau_applied  = ((massMatrix(controller,q0(1:7))*acc')' + friction + velocityProduct(controller,q0(1:7),q0(8:14)) + gravityTorque(controller,q0(1:7)));  
%     B1 = massMatrix(controller,q0(1:7));
%     B2 = get_Bnum(q0(1:7));
%     errorB =B1-B2
%     Sd1= velocityProduct(controller,q0(1:7),q0(8:14));
%     Sd2 = get_Snum(q0(1:7),q0(8:14));
%     errorSd = Sd1-Sd2
    Tau_applied  = ((get_Bnum(q0(1:7))*acc') + friction + get_Snum(q0(1:7),q0(8:14))*q0(8:14)' + get_gnum(q0(1:7)))';  

    

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

    Q(index,:)=q0(1:7);
    QD(index,:)=q0(8:14);
    QDD=accs;
    J_c(index,:,:)=J;
    TAU(index,:)=Tau_applied;

    %Updating the step
    index = index + 1; 

end
filename = "test.mat";
save(filename)
robot_plot;
size(Q);
size(TAU);
xlabel('X');
ylabel('Y');
zlabel('Z');
return;
addpath 'C:\Users\lapoc\OneDrive\Desktop\MATLAB_Simulator\CPF'
main_Carrieri;

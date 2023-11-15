%% Description
% in LWR_sim.m ho scritto il codice per calcolare il residuo r e sigma per
% capire quale link è soggetto alla forza; ho calcolato il punto di applicazione
% usando il residuo per essere usato come inizializzazione per il CPF; infine ho
% realizzato il contact particle filter che funziona bene se il valore iniziale è buono, 
% ho inoltre fatto in modo che il punto calcolato nello sample-instant prima viene usato
% in quello dopo supponendo che il punto sia quasi costante.  
% Ho inoltre creato dei plot per vedere l'andamento delle particelle e l'andamento dell'errore 
% rispetto al vero punto di applicazione. 




%% INIT
clc;
close all;
clear all;

   addpath 'Dynamics'

   addpath 'Functions'
 
%% Hyperparameters to be set
LoadFile=11;

while LoadFile>0
    

 
% Here all the parameters to decide for the computetion
tf=10; % final time

MaxVelocity=100;
% External force applied to the link LINK in a point POINT in respect to
% the reference frame of the link
ExternalForceApplied=[1 2 3]'; 
ExternalForceAppliedActualFrame=[2 4 3]';
gain = 1000*diag([100, 60, 160, 200, 120, 80, 125]); %gain for the residual calculation
 % real link subjected to the force 
radius=0.05; % radius of the circle that the End Effector(EE) of the robot must follow
DeltaT = 0.001; % sampling time of the robot
%q0(1:7)=[0,0,pi/2,0,0,0,0]; % q initialized, the real q initialized is in
% parameters.m
frequency = 1; %frequency*time is the angle of the point of the circle
% in particular frequency represent the angular velocity that the EE must
% follow
threshold_Collision = 0.002; % threshold in order to calculate the residuals, if <0.1 the link is influenced by an external force
threshold_sigma= 0.014; % threshold for the sigma calculated
samples=10; % samples to calculate the residual, in the other projects I think more than 300 samples are used 
% but it depends by the sampling time, since the residual calculation start
% after samples instant




%% Initialization
    
    q0=[0 0 0 0 0 0 0 0 0 0 0 0 0 0]; %initial configuration of the robot
    PlotMeshRobot; % plot of all the meshes that represent the manipulator

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
     for i=1:size(Meshes.ConnectivityList,1)
        for j=1:3
            triangles(:,j,i)=Meshes.Points(Meshes.ConnectivityList(i,j),1:3,link);
        end
     end
     LoadFile=-1;
     
end
%% Now there is the matlab simulation of the movement



S_fext =[0 -ExternalForceAppliedActualFrame(3) ExternalForceAppliedActualFrame(2) ; ExternalForceAppliedActualFrame(3) 0 -ExternalForceAppliedActualFrame(1) ; -ExternalForceAppliedActualFrame(2) ExternalForceAppliedActualFrame(1) 0 ];
    link=5;
Pointslink = Meshes.Points(:, :,link);
    surface_points = Pointslink(Pointslink(:, 2, 1) ~= 0, :, :);
    LastPoint=size(surface_points,1);
    indecs=550;
         random_vector=Meshes.Points(indecs, :,link);
        
        point = (random_vector(1:3))';
        ExternalForceAppliedActualFrame=[[0.03 0.04 0.05]'-point]*100*rand();
        S_fext =[0 -ExternalForceAppliedActualFrame(3) ExternalForceAppliedActualFrame(2) ; ExternalForceAppliedActualFrame(3) 0 -ExternalForceAppliedActualFrame(1) ; -ExternalForceAppliedActualFrame(2) ExternalForceAppliedActualFrame(1) 0 ];
 
        m=-S_fext*point(1:3);
         p_ref(:,1) = f(q0(1),q0(2),q0(3),q0(4),q0(5),q0(6));
         dp_ref(:,1) = [0 0 0]';
         d2p_ff(:,1) =[0 0 0]';
close all
tic
while (toc<2000)%(frequency * (t0) < 2*pi) % it ends when a circle is completed
     disp(t0);  
   if mod(t0, 0.1) == 0
       indecs=indecs+1;
         random_vector=Meshes.Points(indecs, :,link);
        
        point = (random_vector(1:3))';
        
        m=-S_fext*point(1:3);
   end
   if mod(t0, 0.3) == 0
     ExternalForceAppliedActualFrame=[[0.03 0.04 0.05]'-point]*100*rand();
     S_fext =[0 -ExternalForceAppliedActualFrame(3) ExternalForceAppliedActualFrame(2) ; ExternalForceAppliedActualFrame(3) 0 -ExternalForceAppliedActualFrame(1) ; -ExternalForceAppliedActualFrame(2) ExternalForceAppliedActualFrame(1) 0 ];
 
   end
    
    
    %% TRAJ (CIRCLE)
        ranges=10*[1000 800 600 400 200 100 50];


        for i = 1:7
            q0(i) = q0(i) + 2*pi/ranges(i);
        end
       

        p_ref(:,2)=p_ref(:,1);
        p_ref(:,1) = f(q0(1),q0(2),q0(3),q0(4),q0(5),q0(6));
        dp_ref(:,2)=dp_ref(:,1);
        dp_ref(:,1) = (p_ref(:,2)-p_ref(:,1))/DeltaT;
        d2p_ff(:,1) = (dp_ref(:,1)-dp_ref(:,1))/DeltaT; 
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
    
    err_p = p_ref(:,1) - p;


    err_dp = dp_ref(:,1) - dp;
  
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
       J_withwrenches = ComputePoint_withWrenches(q0(1:7),link);
       TauExternalForce=(J_withwrenches'*[ExternalForceAppliedActualFrame;m])';
        

       %TauExternalForce=[0 0 0 0 0 0  0];

    
   
  
    
    %% COMPUTED TORQUE FL - dynamic model
     TauFL = (g + S*q0(8:14)' + B * Uref')';  % this is the applied torque 
  
    Tau = TauFL+TauExternalForce; % this is the real tau applied

    acc = (inv(B) * (Tau' - friction - S*q0(8:14)' - g))'  ;
 
    if isnan(acc)
        disp("NaN value encountered in acc. Exiting the loop.");
        break;  % Exit the loop
    end


    

    Tau_applied  = B*acc' + friction + S*q0(8:14)' + g;  
    

    

    accs = vertcat(accs,acc);
    
    accs_ref = vertcat(accs_ref,[Uref,Uref_task, Uref_redundant]);            
    
    task_vec = vertcat(task_vec, [p',dp',d2p',p_ref(:,1)',dp_ref(:,1)',d2p_ref']);
    
    torque_fl = vertcat(torque_fl,Tau);
    
    singular_values = vertcat(singular_values,sigma);       
    
    % EULER INTEGRATION    
    q0(1:7)  = q0(1:7) + q0(8:14) * DeltaT ; 
    pk=NaN(7);
    if q0(1:7)==pk
         disp("NaN value encountered in q0. Exiting the loop.");
        break;
    end

    q0(8:14) = acc * DeltaT + q0(8:14);            

    t0 = t0+DeltaT;
    
    joints = vertcat(joints,q0);    
    time = vertcat(time,t0);


    %% save all the variables needed for the force reconstruction and the calculation of the residuals
    TauExtForce(index,:)=TauExternalForce;
    Q_sampled(index,:)=q0(1:7);



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
        Js{index}=J_withwrenches;
        moments(index,:)=[ExternalForceAppliedActualFrame;m];

        Residual_calculated(index,:)=Tau_residual; 
        Sigma_calculated(index,:)=Tau_sigma; 
      


     end
index = index + 1; 

    
end
 save('PseudoinverseNN','Residual_calculated','link_calculated','point_calculated','Js' ,'moments')


 
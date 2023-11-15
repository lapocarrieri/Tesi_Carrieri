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
 ss = 1;
 Sigma=0;
currentPool = gcp('nocreate'); % Get the current pool, if it exists

if isempty(currentPool)
    % No pool is active, so create a new one
    pool = parpool(2); % Create a pool with 2 workers

    % Your parallel code here
    
    % Delete the pool when you're done with it
    
else
    % A pool is already active, display a message
    disp('A parallel pool is already active. Skipping pool creation.');
end
n=10000;
time=zeros(n,1);
firstTime=true;
   addpath 'Dynamics'

   addpath 'Functions'
 
%% Hyperparameters to be set

%% Now there is the matlab simulation of the movement
load('Initialization\initialization05.mat')
tf=0.5;
disp('The point in the actual frame is:')
disp(vpa(point',3));
    close all
    chi2 = zeros(3,20);


f1=figure;

%figure(f1),scatter3(p_0(1), p_0(2), p_0(3), 'filled');
set(f1, 'Name', 'End effector points');
xlabel('X');
ylabel('Y');
zlabel('Z');

f2=figure;
set(f2, 'Name', 'contact particle filter points');
xlabel('X');
ylabel('Y');
zlabel('Z');
f3=figure;
f4=figure;
f5=figure;
            
f6=figure;
      
            xlabel('X');
            ylabel('Y');
            zlabel('Z');
            title('3D Point Cloud');
            grid on;
            CalculatedPoint=[0 0 0];
             save('sharedVar2','CalculatedPoint');
            gainE=100;
            GainInv=inv(eye(7)+gain*DeltaT) * gain ;
            GainEInv=inv(eye(1)+gainE*DeltaT) * gainE ;


        
while (t0<tf)%(frequency * (t0) < 2*pi) % it ends when a circle is completed
     disp('time instant:')
     disp(t0);
     

%     %figure(f1);
     
    %set(f1, 'visible', 'on'); 

    
    %% TRAJ (CIRCLE)

    p_ref(1,1) = p_0(1) -  radius * (1 - cos(frequency * (t0)));
    dp_ref(1,1) = dp_0(1) - radius * frequency * sin(frequency * (t0));
    d2p_ff(1,1) = d2p_0(1) - radius * frequency * frequency * cos(frequency * (t0));

    p_ref(2,1) = p_0(2) -  radius * (sin(frequency * (t0)));
    dp_ref(2,1) = dp_0(2) - radius * frequency * cos(frequency * (t0));
    d2p_ff(2,1) = d2p_0(2) + radius * frequency * frequency * sin(frequency * (t0));
    
    p_ref(3,1) = p_0(3) ;
    dp_ref(3,1) = dp_0(3);
    d2p_ff(3,1) = d2p_0(3);
    hold on
    %figure(f1),scatter3(p_ref(1,1),p_ref(2,1),p_ref(3,1),'g','SizeData',20)
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
    
    sigma_trajectory = min(diag(S));
    
    dp = J * q0(8:14)';
    
    d2p = dJ * q0(8:14)' + J * accs(end,:)';
    
    err_p = p_ref - p;


    err_dp = dp_ref - dp;
    



    %figure(f1),plot3(p(1), p(2), p(3), 'ro', 'MarkerSize',2);
    hold on
    
    %% PD KINEMATIC  
    d2p_ref = Kp * err_p + Kd * err_dp + d2p_ff;
    %d2p_ref = Kp * err_p + Kd * error_theta + d2p_ff;
    tspan =[t0,t0+DeltaT];

    
   % EXACT PROJECTOR:
    Pj = eye(7) - pinv(J) * J;  
    
    % Friction calculation
    %% Detection 
    if link_collided(index)>0
        print('the robot is subjected to a force')
        position=p;
        d2p_ref = Kp * err_p  - Kd *dp ;
    end


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

 
       %external torque applied to the system
       
       T_actualframe= QtoP(q0(1:7),link);
       RealPointIntersectedWorldFrame=T_actualframe*[point;1];
        J_actualframe=T_actualframe(1:3,1:3)'*J_force;
        

      % TauExternalForce0 = vpa((transpose(J_actualframe) * ExternalForceAppliedActualFrame)',3)
       %TauExternalForce =(transpose(J_force)*ExternalForceApplied)';
       %%force in world frame
       J_withwrenches = ComputePoint_withWrenches(q0(1:7),link);
       TauExternalForce=(J_withwrenches'*[ExternalForceAppliedActualFrame;m])';
       %return;
       
        
%          if index<60
%                  TauExternalForce=[0 0 0 0 0  0 0];
%          end

    
   %TauExternalForce=[0 0 0 0 0  0 0];
    
    %% COMPUTED TORQUE FL - dynamic model
    TauFL = (g + S*q0(8:14)' + B * Uref')';  % this is the applied torque 
  
    Tau = TauFL+TauExternalForce; % this is the real tau applied

    acc = (inv(B)* (Tau' - friction - S*q0(8:14)' - g))'  ;
 
    
    % EULER INTEGRATION    
    q0(1:7)  = q0(1:7) + q0(8:14) * DeltaT ; 
    pk=NaN(7);
    if q0(1:7)==pk
        return;
    end

    q0(8:14) = acc * DeltaT + q0(8:14);            

    t0 = t0+DeltaT;
    
   


    %% save all the variables needed for the force reconstruction and the calculation of the residuals
    TauExtForce(index,:)=(TauExternalForce);
    Q_sampled(index,:)=q0(1:7);




    %PointOfApplication(index,:)=QtoP(Q_sampled(end,:),link)*point;

    QD_sampled(index,:)=q0(8:14);
    QDD_sampled(index,:)=(acc);


    TAU_sampled(index,:)=(TauFL');


        B_sampled{index}=(B);
        S_sampled{index}=( S);
        g_sampled{index}=(g);
        h_sampled{index}=(S'*q0(8:14)'-g);
       
        samples=50;
       
    
    %% Residual Calculation and Force and point reconstruction
    if index>samples % in order to have enough samples to calculate the residuals
        fprintf('Starting of the Residual calculation:\n')
            sumTau=0;
            sumH=0;
            sumRes=0;
            p0=0;
             
            sumEdot=0;
            sumSigma=0;
% Parameter set 1: K0 = 25 Hz (FO); S = 80 Hz2, T = 22.4Hz (SOSM);
% S 1 = 80 Hz2, T1 = 17.9 Hz, S 2 = 1600 Hz2, T2 = 80 Hz (SOSML).
% Parameter set 2: K0 = 5Hz (FO); S = 20 Hz2, T = 11.2Hz (SOSM);
% S 1 = 20 Hz2, T1 = 8.9 Hz, S 2 = 64 Hz2, T2 = 16 Hz (SOSML).
  
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
                   %r = inv(eye(7)+gain*DeltaT) * gain * ((B_sampled{t}*QD_sampled(t, :)' - p0) - (sumTau + sumH + sumRes)*DeltaT);
                    r =(inv(eye(7)+gain*DeltaT) * gain * ((B_sampled{t}*QD_sampled(t, :)' - p0) - (sumTau + sumH + sumRes)*DeltaT));
                    
                   
                   sumRes = sumRes + r;
                   
               end
        end
      
    
     
        
        %R = NoncausalButterworthFilter(R);
                %% Sliding Mode Observer SOSM
%discrete-time observer
             S1=80; T1=sqrt(S1);

            
            
            p=B_sampled{index-1}*QD_sampled(index-1, :)';
                if ss == 1
                   
                   p_hat = p;
                  
                   Sigma = zeros(7,1);
                   Dsigma= zeros(7,1);
                   ss=2;
                   
                else
                    p=p_hat-p;
                   signP=tanh(p*50);
                   dp_hat=TAU_sampled(t,:)'+S_sampled{t}'*QD_sampled(t,:)'-g_sampled{t}+sigma-T1*signP;
                   Dsigma=Dsigma-S1*signP;
                   p_hat=p_hat+DeltaT*dp_hat;
                   Sigma=Sigma+DeltaT*Dsigma;
                    
               end
           
            ExternalTauCalculatedSOSM=Sigma;
            
        %% Sliding Mode Observer SOSML

%              S1=20; T1=8.9;
%              S2=64; T2=16;
             S1=80; T1=sqrt(S1);
             S2=116 ; T2=sqrt(S2)*2;



            for tt = 1:samples+1
            
            t=index-samples+tt-1;
            p=B_sampled{t}*QD_sampled(t, :)';
                if tt == 1
                   
                   p_hat = p;
                  
                   sigma = zeros(7,1);
                   dsigma= zeros(7,1);
                   
                else
                    p=p_hat-p;
                   signP=tanh(p*50);
                   dp_hat=TAU_sampled(t,:)'+S_sampled{t}'*QD_sampled(t,:)'-g_sampled{t}+sigma-T2*p-T1*signP;
                   dsigma=dsigma-S1*signP-S2*p;
                   p_hat=p_hat+DeltaT*dp_hat;
                   sigma=sigma+DeltaT*dsigma;
                    
               end
            end
            ExternalTauCalculatedSOSML=sigma;
            
        %% Energy-based detection
        for tt = 1:samples+1

            t=index-samples+tt-1;

            sumEdot = sumEdot + QD_sampled(t,:)*(TAU_sampled(t,:)'- g_sampled{t});
           

        
               if tt == 1
                   
                   p0 = 1/2*QD_sampled(t, :)*B_sampled{t}*QD_sampled(t, :)';
                   sumSigma=0;
                   sigma0 = 0;
               else
                   sigma0 =  GainEInv*((1/2*QD_sampled(t, :)*B_sampled{t}*QD_sampled(t, :)' - p0) - (sumEdot + sumSigma)*DeltaT);
                    
                   
                   sumSigma = sumSigma + sigma0;
               end
        end
      
        
        %R = NoncausalButterworthFilter(R);
        %% Point estimation - initialization of the contact particle filter
        
        errorTorque=vpa(norm(r-TauExtForce(index,:)'),2)
        
        errorSlidingMode=vpa(norm(TauExtForce(index,:)'-ExternalTauCalculatedSOSM),2)
        
        errorSlidingModeL=vpa(norm(TauExtForce(index,:)'-ExternalTauCalculatedSOSML),2)
        Residual_calculated(index,:)=r;
        ExternalTauSOSM_calculated(index,:)=ExternalTauCalculatedSOSM;
        ExternalTauSOMSML_calculated(index,:)=ExternalTauCalculatedSOSML;
        Sigma_calculated(index)=sigma0;
        index = index + 1; 
        time(index)=t0;
        continue;
        %figure(f4),plotTorque(TauExtForce,Residual_calculated,index, 3,DeltaT)
        [is_collided_sigma] = getLink_withSigma(sigma0,threshold_sigma,QD_sampled(end, :));


        [link_collided(index),is_collided_r] = getLink_withResidual(r,threshold_Collision);
  
                is_collided(index)=0;
  
            if is_collided_r==1 && is_collided_sigma==1
                is_collided(index)=1;
            end
            if is_collided(index)==0
                link_collided(index)=0;
            end
            LinkInCollision=link_collided(index)

        %LINKK=link_collided(index)

        if is_collided(index) == 1 
                %J_withwrenches = ComputePoint_withWrenches(Q_sampled(index,:),link_collided(index));
                J_withwrenches = ComputePoint_withWrenches(Q_sampled(index,:),link);

                wrenches1=pinv(J_withwrenches')*R(end,:)';
                %ExternalForce_Real =(pinv(transpose(J_force))*Tau_residual')'
        

                
                                error1 = [ExternalForceAppliedActualFrame;m]-wrenches1;
               
                % Calculate the SVD of matrix A
                [U, S, V] = svd(J_withwrenches');
                s = diag(S);
                tol = max(size(J_withwrenches')) * eps(max(s));
                r = sum(s > tol);
                U_r = U(:, 1:r);
                V_r = V(:, 1:r);
                S_inv = diag(1./s(1:r));
                wrenches2 = V_r * S_inv * U_r' * R(end,:)';
               
                error2 = [ExternalForceAppliedActualFrame;m]-wrenches2;
               % DLS method
%                it=1;
%                
%                for lambda=0:0.001:100
%                       % Damping parameter (you can adjust this value)
%                     wrenches4 = (J_withwrenches*J_withwrenches' + lambda^2*eye(size(J_withwrenches', 2))) \ (J_withwrenches*R(end,:)');
%                     
%                     error4 = [ExternalForceAppliedActualFrame;m]-wrenches4;
%                     normerror4(it)=norm(error4);
%                     it=it+1;
%                end
%               plot(normerror4)
                lambda = 0.1;
               wrenches4 = (J_withwrenches*J_withwrenches' + lambda^2*eye(size(J_withwrenches', 2))) \ (J_withwrenches*R(end,:)');
                    
                    error4 = [ExternalForceAppliedActualFrame;m]-wrenches4;
                    
%                % weighted pseudoinverse
%                it=1;
%                 for gg=0.001:0.001:1
%                    W = gg*diag(rand(6, 1));
%                    J_withwrenchesweightedPseudoinverse=W^(-1/2)*pinv(J_withwrenches'*W^(-1/2));
%                    wrenches3=J_withwrenchesweightedPseudoinverse*R(end,:)';
%                    error3 = [ExternalForceAppliedActualFrame;m]-wrenches3;
%                    normerror3(it)=norm(error3);
%                    it=it+1;
%                 end
%                 for it=0:49
%                     for a=1:19
%                         normerror(it+1)=normerror3(it*20+1)+normerror3(it*20+a+1);
%                     end
%                 end
                    W=eye(6);
                   W(1,1)=60;
                   W(2,2)=45;
                   W(3,3)=44;
                   W(4,4)=55;
                   W(5,5)=52;
                   W(6,6)=53;
                   
                J_withwrenchesweightedPseudoinverse=W^(-1/2)*pinv(J_withwrenches'*W^(-1/2));
                       wrenches3=J_withwrenchesweightedPseudoinverse*R(end,:)';
                        error3 = [ExternalForceAppliedActualFrame;m]-wrenches3;

%                 % quadratic programming method
%                 % Define the matrices A and C, and vectors b and D
%                 A = [1, 2, 3; 4, 5, 6; 7, 8, 9];
%                 b = [1; 2; 3];
%                 C = [1, 0, 0; 0, 1, 0];
%                 D = [3; 4];
%                 
%                 % Solve the quadratic programming problem
%                 x = quadprog(A' * A, -A' * b, C, -D);
%                 
%                 % Display the solution
%                 disp('Solution:')
%                 disp(x); 
            %PreviousPoint
                %WeightCalculation;
                %W=diag(weights)
                
                J_withwrenchesweightedPseudoinverse=W^(-1/2)*pinv(J_withwrenches'*W^(-1/2));
                       wrenches5=J_withwrenchesweightedPseudoinverse*R(end,:)';
                           
                            
                        error5 = [ExternalForceAppliedActualFrame;m]-wrenches5;
                        norm(error5)
               
               % null-space method
%                norm(error1)
%                norm(error2)
%                norm(error3)
%                norm(error4) 
%                norm(error5)
               
               %wrenches3=[ExternalForceAppliedActualFrame;m];
              f_i=wrenches5(1:3);
                m_i=wrenches5(4:6);
                
               
                
                Sf_i=[0 -f_i(3) f_i(2) ; f_i(3) 0 -f_i(1) ; -f_i(2) f_i(1) 0 ];
                p_dc=Sf_i*m_i/(norm(f_i))^2;

%                 radii = 0.05;   
%                  x = radii*cos(pi/3)+ cnt(1);
%                  y = radii*sin(pi/5) +  cnt(2);
%                  z=height;
%                  p_dc=[x, y,z]'


                
               
               % p_dc=pinv(-Sf_i)*m_i
              
               T_actualframe= QtoP(q0(1:7),link);
                % T= QtoP(Q_sampled(end,:),link_collided(end));
                
                Lambda_coefficient=f_i/(norm(f_i))^2;
                line.origin=T_actualframe*[p_dc;1];
                line.origin=line.origin(1:3);
                line.direction=T_actualframe*[Lambda_coefficient;1];
                line.direction=line.direction(1:3);
                RealPointIntersected=double(T_actualframe*[point;1]);
                disp(vpa(RealPointIntersectedWorldFrame,2))
                figure(f3),plot3( RealPointIntersected(1), RealPointIntersected(2),RealPointIntersected(3), 'o', 'MarkerSize', 4, 'MarkerFaceColor', 'r', 'LineWidth', 2);
                hold on
                %Point_intersected = IntersectionPoint(line,link,Q_sampled(index,:),RealPointIntersected(1:3),Meshes,f3);
                Point_intersected=Point_intersected+[0.15,-0.1,0.07]
                if Point_intersected==[0 0 0]'
                    Point_intersected_actual_frame=closest_point_to_triangle(triangles, p_dc');
                    Point_intersected=T_actualframe*[Point_intersected_actual_frame';1];
                    disp('point initialiazation not optimal')
                end
                Point_intersected=(Point_intersected(1:3))';
            
                disp(vpa(Point_intersected,2))

                
%                      raggio1=  sqrt(PointRetrieved(1)^2+PointRetrieved(2)^2)
%              raggio2=  sqrt(PointRetrievedWithFunction(1)^2+PointRetrievedWithFunction(2)^2)
%                 

                %fprintf('the point calculated with the residuals is: %d\n',PointRetrieved)
%                 m=-S_fext*PointOnTheCylinder1
                 %m_calculated=-S_fext*p_dc;
                 %error_momentum=abs(m-m_calculated)  ;
                
%                
%     
%                 error_initialization=abs(point(1:3)-PointRetrieved)
                error_initialization=(vpa(norm(RealPointIntersected(1:3)-Point_intersected(1,:)'),3))

                
               
%                 ForcePoint=T_actualframe*[Point_intersected;1] ;%point in frame 0
%                 %ForcePointApplication(index,:)=ForcePoint(1:3);
%                 
%                 Force=T_actualframe*[f_ext;1]; %force in frame 0 
        

                NumberNonCollidedSamples=0;
        else
            NumberNonCollidedSamples=NumberNonCollidedSamples+1;
                
        end
        if NumberNonCollidedSamples>10
            chi = zeros(3,num_part);
            chi2 = zeros(3,num_part);
        end
%           if Point_intersected==[ 0 0 0]
%                 break;
%           end
        

        %lambda = -1:0.01:1;
        
        
        %estimated_cp=p_dc+lambda*f_ext/(norm(f_ext));
 
        
        
   

       %% CONTACT PARTICLE FILTER
        
        ind=1;
        %is_initialized=false;
                T= QtoP(Q_sampled(index,:),link);
            Rotation = T(1:3,1:3);
             tran = T(1:3,4);

         Point_intersectedActualFrame=double(inv(T)*[Point_intersected;1]);
        disp('Point_intersectedActualFrame:')
        disp(vpa(Point_intersectedActualFrame(1:3)',2))
           ErrorBeforeCPF_ActualFrame=abs(Point_intersectedActualFrame(1:3)-point);
          disp('error before CPF:')
           disp(vpa(ErrorBeforeCPF_ActualFrame,3))
              figure(f6),scatter3(point(1),point(2),point(3),'h', 'filled' ,'SizeData', 50);
            % Add a text label
            hold on
                figure(f6),text(point(1),point(2),point(3), 'Real Point', 'FontSize', 6, 'HorizontalAlignment', 'left');
                save('sharedData.mat', 'point', 'link_collided','index','chi','Q_sampled','Residual_calculated','Point_intersectedActualFrame','f6');
               if firstTime
                   parfeval( @CPF_script, 1); % 0 means no output needed
                   disp('poolattivato')
                   firstTime=false;
               end
                
                
                load('sharedVar2')
                
             contact_point_PF = Rotation*CalculatedPoint'+tran;
             disp('error Contact point calculated after CPF:')
             disp(vpa(norm(RealPointIntersectedWorldFrame(1:3)-contact_point_PF),4))
             figure(f6),scatter3(CalculatedPoint(1),CalculatedPoint(2),CalculatedPoint(3),'p', 'filled' ,'SizeData', 50);
            % Add a text label
            textname="CalculatedPoint 5-th iteration";
                figure(f6),text(CalculatedPoint(1),CalculatedPoint(2),CalculatedPoint(3), textname, 'FontSize', 6, 'HorizontalAlignment', 'left');
             %disp(vpa(contact_point_PF',4))
             
         %    ErrorAfterCPFWorldFrame(:)=abs(RealPointIntersectedWorldFrame(1:3)-contact_point_PF);

               % disp(vpa(abs(RealPointIntersectedWorldFrame(1:3)-contact_point_PF),4))
             %[fval(end+1)] = observationModel(eye(7), Q_sampled(i,:),Residual_calculated(i,:),[ForcePointApplication(i,:); 1]',link_collided(i));
              % here is plotteed the error of between the point calculated
              % after i iteration of the CPF in respect to the real point
              % calculated
            

        %    drawnow() 
    

        
        %disp('Error After CPF World Frame:')
        %disp(vpa(ErrorAfterCPFWorldFrame,2));
      %  figure(f5),plot(1:size(ErrorAfterCPF,2),norm(ErrorAfterCPF));
       % hold on
        %figure(f5),plot(1:size(ErrorAfterCPFWorldFrame,2),norm(ErrorAfterCPFWorldFrame));
        %plot the cylinder
       % figure(f2),h2 = surf(double(X_prime),double(Y_prime),double(Z_prime));
        %xlabel('x');
        %ylabel('y');
        %zlabel('z');
        %title('link collided with point particles')
       % h2.EdgeColor = 'none';

        % Set the transparency of the figure 
         %   alpha(h2, 0.1);
        
        %plot the dh reference frame
        
        hold on
%         figure(f2),plotRF(Rotation, tran)
%         figure(f2),plot3(tran(1),tran(2),tran(3))
%         figure(f2),plotParticles(chi, Rotation, tran);
%          figure(f2),plot3(real_point(1),real_point(2),real_point(3),'b');
%          hold off
     end
index = index + 1; 
time(index)=t0;



%% Collaboration
    
end
%Residual_calculated
%ExternalTauSOSM_calculated
%ExternalTauSOSML_calculated
%TauExtForce
samples2=samples;
figure()
plot(time(samples2:index), TauExtForce(samples2:index,:)', 'r', 'LineWidth', 2);
hold on;
plot(time(samples2+1:index-1), Residual_calculated(samples2+1:index-1,:), 'g', 'LineWidth', 2);
hold off
figure()
plot(time(samples2:index), TauExtForce(samples2:index,:)', 'r', 'LineWidth', 2);
hold on;
plot(time(samples2+1:index-1), ExternalTauSOSM_calculated(samples2+1:index-1,:), 'g', 'LineWidth', 2);
hold off
figure()
plot(time(samples2:index), TauExtForce(samples2:index,:)', 'r', 'LineWidth', 2);
hold on;
plot(time(samples2+1:index-1), ExternalTauSOMSML_calculated(samples2+1:index-1,:), 'g', 'LineWidth', 2);
hold off

return;
 robot_plot;
 
main_Carrieri;



% function plotForces(out, dofs, lineWidth)
%     for i=1:3
%         plot(out.tout, out.contact_force.Data(:,i), "lineWidth", lineWidth);
%         hold on
%         plot(out.tout, out.estimated_force.Data(:,i), "lineWidth", lineWidth);
%     end
%     legend("real F_x", "estimated F_x", "real F_y", "estimated F_y" , "real F_z", "estimated F_z");
%     grid on
% end
% 
% function plotForces2(time,Fc, lineWidth)
% 
%     for i=1:3
%         plot(time(1:length(Fc(i,:))), Fc(i,:), "lineWidth", lineWidth);
%         hold on 
%     end
% 
%     grid on
% end
% function plotTorque(TauExtForce,Residual_calculated,index, lineWidth,DeltaT)
%     legend_torque = [];
%     
%     for i=1:7
%         plot(i*DeltaT, Residual_calculated(index,i), "lineWidth", lineWidth);
%         hold on
%         plot(i*DeltaT, TauExtForce(index,i), "lineWidth", lineWidth);
%         hold on
%         real_contact_str = "real tauk_"+i;
%         estimated_contact_str = "estimated tauk_"+i;
%         legend_torque = [legend_torque, [real_contact_str, estimated_contact_str]];
%     end
%     legend(legend_torque);
%     grid on
% end
% 
% 
% 
% 
% function plotArrow(point, direction, arrow_color, arrow_size)
%         %this plot the arrow at the beginning of the line <---
%         last_point1 = point + 0.01*(direction+tan(60));
%         last_point2 = point + 0.01*(direction+-tan(60));
%         cf_plot1 = line([point(1), last_point1(1)], [point(2), last_point1(2)], [point(3), last_point1(3)],"color", arrow_color, "lineWidth", arrow_size);
%         cf_plot2 = line([point(1), last_point2(1)], [point(2), last_point2(2)], [point(3), last_point2(3)],"color", arrow_color, "lineWidth", arrow_size);
% end
% 
% %plot num_part particlces on the cylinder sourface where
% %r is the cylinder radius h is its height and c its center 
% function plotParticles(chi,R, t)
%     chi = R*chi(:,:)+t;
%     for i=1:length(chi)
%         hold on 
%         plot3(chi(1,i),chi(2, i), chi(3, i),".", "Color", "red");
%     end
%    
% end


%plot the RF in a generic point with a generic orientation. The lenght of
%the axis are fixed but eventually can be passed as a parameters



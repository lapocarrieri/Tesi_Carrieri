%% Description
% in LWR_sim.m ho scritto il codice per calcolare il residuo r e sigma per
% capire quale link è soggetto alla forza; ho calcolato il punto di applicazione
% usando il residuo per essere usato come inizializzazione per il CPF; infine ho
% realizzato il contact particle filter che funziona bene se il valore iniziale è buono, 
% ho inoltre fatto in modo che il punto calcolato nello sample-instant prima viene usato
% in quello dopo supponendo che il punto sia quasi costante.  
% Ho inoltre creato dei plot per vedere l'andamento delle particelle e l'andamento dell'errore 
% rispetto al vero punto di applicazione. 

%calculate the mean
% calculate
 


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
load('initialization_externalForce_link6_force=0DeltaT0012.mat')
tf=20;
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

index=index-1;
initializaton=0;
CASE = 0;
FirstTouch=1;
        DISTANCES={};
        initial_position=[0 0 0];
        initial_force = inv(T_actualframe(1:3,1:3))*ExternalForceApplied;
while (t0<tf)%(freiquency * (t0) < 2*pi) % it ends when a circle is completed
     disp('time instant:')
     disp(t0);
     index = index + 1; 
     time(index)=t0;
     

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
       
       T= QtoP(q0(1:7),link);
       RealPointIntersectedWorldFrame=T*[point;1];
        J_actualframe=T(1:3,1:3)'*J_force;
        

      % TauExternalForce0 = vpa((transpose(J_actualframe) * ExternalForceAppliedActualFrame)',3)
       %TauExternalForce =(transpose(J_force)*ExternalForceApplied)';
       %%force in world frame
       %TauExternalForce=(J_withwrenches'*[ExternalForceAppliedActualFrame;m])';

       displacement = Point_intersected - initial_position;

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
       F_applied = max(F_min, min(norm(F_applied), F_max)) * (initial_force/norm(initial_force))
       fapplieds{index}=F_applied;

       ExternalForceAppliedActualFrame = inv(T_actualframe(1:3,1:3))*ExternalForceApplied;
       S_fext=skew_symmetric(F_applied);
       m=double(-S_fext*point(1:3));
       TauExternalForce=(J_withwrenches'*[F_applied;m])';
        
%          if index<60
%                  TauExternalForce=[0 0 0 0 0  0 0];
%          end

    
   %TauExternalForce=[0 0 0 0 0  0 0];
    
    %% COMPUTED TORQUE FL - dynamic model
    TauFL = (g + S*q0(8:14)' + B * Uref')';  % this is the applied torque 
  
     % this is the real tau applied
    %% Reaction Strategies
    
        CASE=1  ;
    
    switch CASE
        case 0
            %do nothing
        case 1
            % Stop the robot
            TauFL = -r; % Assuming 'q' is the vector of joint positions
            
        case 2
            % Compensate gravity only
            TauFL = g; % Assuming 'g(q)' computes the gravity compensation torque
            
        case 3
            % Reflex torque reaction
            TauFL = Kr * r + g; % 'Kr' is the reflex gain, 'r' is the reflex signal
            
        case 4
            % Exponential decay of contact
            TauFL = g + Kr * exp(-t * tc); % 'tc' is the decay constant
            
        case 5
            % Admittance mode reaction
            TauFL = g; % Gravity compensation
            thetadot = Kr * r; % 'thetadot' is the joint velocity command
            
        case 6
            % Time scaling
            % This strategy might involve rescaling time in some way, so this is just a placeholder
            %TauFL = ...; % Define the time scaling control law
            
        case 7
            % Cartesian task preservation
            % This strategy involves projecting the torque into a null space
            % 'P' is the projection matrix, 'tau_task' is the torque related to the task
            %P = ...; % Define the projection matrix
            %tau_task = ...; % Define the task-related torque
            TauFL = P * tau_task;
            
        case 8
            % Admittance mode + floating reaction
            % Combining admittance control with some floating base control, placeholder
            %TauFL = ...; % Define the combined control law
        case 9
            J_withwrenchesss = ComputePoint_withWrenches(q0(1:7),LinkInCollision);
            Kf=1;
            %disp("force error")
            force_error = F_initial - f_i %F in global frameJ_withwrenchesss = ComputePoint_withWrenches(q0(1:7),LinkInCollision);
            forcerrors{index}=force_error;
            mi=-skew_symmetric(force_error)*Point_intersected_Actualframe(1:3);

            %tau1 = J_withwrenchesss'*[ExternalForceAppliedActualFrame;mi]
            tau_force_control = J_withwrenchesss' * (Kf * [force_error;mi]);
            TauFL = TauFL + tau_force_control';
            
           
            
        otherwise
            error('Invalid CASE number');
    end
%%
    % EULER INTEGRATION   
    Tau = TauFL+TauExternalForce;
    acc = (inv(B)* (Tau' - friction - S*q0(8:14)' - g))'  ;
    q0(1:7)  = q0(1:7) + q0(8:14) * DeltaT ; 
    pk=NaN(7);
    if q0(1:7)==pk
        return;
    end
    if CASE== 10
        acc = sign(q0(8:14))*10;
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
       
        samples=300;
       
    
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
               R(end,:)=r;
        end
      
    
     
        
        R = NoncausalButterworthFilter(R);
        r=R(end,:);      
            
     
        %% Point estimation - initialization of the contact particle filter
        
        errorTorque=vpa(norm(r-TauExtForce(index,:)'),2)
        
       Residual_calculated(index,:)=r;

        %figure(f4),plotTorque(TauExtForce,Residual_calculated,index, 3,DeltaT)
      is_collided_sigma=1;

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
                mu=0.01;
                
%% Point calculation through f and m
               J_withwrenches = ComputePoint_withWrenches(Q_sampled(index,:),link);  % Code (explanation to be added)
                wrenches1=pinv(J_withwrenches')*Residual_calculated(index,:)';  % Code (explanation to be added)
               %wrenches3=[ExternalForceAppliedActualFrame;m];

              f_i=wrenches1(1:3);
            
               m_i=wrenches1(4:6);
                Sf_i=[0 -f_i(3) f_i(2) ; f_i(3) 0 -f_i(1) ; -f_i(2) f_i(1) 0 ];
                p_dc=Sf_i*m_i/(norm(f_i))^2;

%                 radii = 0.05;   
%                  x = radii*cos(pi/3)+ cnt(1);
%                  y = radii*sin(pi/5) +  cnt(2);
%                  z=height;
%                  p_dc=[x, y,z]'
            T= QtoP(Q_sampled(index,:),link);
            Rotation = T(1:3,1:3);
             tran = T(1:3,4);

                
               
               % p_dc=pinv(-Sf_i)*m_i
              
               
                % T= QtoP(Q_sampled(end,:),link_collided(end));
                
                Lambda_coefficient=f_i/(norm(f_i))^2;
                line.origin=T*[p_dc;1];
                line.origin=double(line.origin(1:3));
                line.direction=double(T*[Lambda_coefficient;1]);
                line.direction=line.direction(1:3);
                RealPointIntersected=double(T*[point;1]);
                %disp(vpa(RealPointIntersectedWorldFrame,2))
                figure(f3),plot3( RealPointIntersected(1), RealPointIntersected(2),RealPointIntersected(3), 'o', 'MarkerSize', 4, 'MarkerFaceColor', 'r', 'LineWidth', 2);
                hold on
                Point_intersected = IntersectionPoint(line,link,Point_intersected(1:3),Meshes,f3,T);
                if size(Point_intersected,2)==3
                    Point_intersected=Point_intersected';
                end
                Point_intersected_Actualframe = inv(T)*[Point_intersected;1];
                %disp('Point_intersected')
                if Point_intersected==[0 0 0]'
                    Point_intersected_actual_frame=closest_point_to_triangle(triangles, p_dc');
                    Point_intersected=T*[Point_intersected_actual_frame';1];
                    disp('point initialiazation not optimal')
                end
                Point_intersected=double(Point_intersected(1:3)')
                if  Point_intersected ~= [0 0 0]'
                    if FirstTouch==1
                        initial_position = Point_intersected;
                        initial_force = f_i;
                        FirstTouch=0;
                    end
                end
            

                
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
                error_initialization=(vpa(norm(RealPointIntersected(1:3)-Point_intersected(1,:)'),3));

                
               
%                 ForcePoint=T*[Point_intersected;1] ;%point in frame 0
%                 %ForcePointApplication(index,:)=ForcePoint(1:3);
%                 
%                 Force=T*[f_ext;1]; %force in frame 0 
        

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

        %% Reaction strategy
        % Possible options:
        % collaboration: mantain the initial force in that direction
        % not expected: stop immediately
        % mantain the force while continuining the task
        % collaborative 
        % Assuming f_i, Frelax, Fabort, qd, q, xdot, J, Jc, K0, Kf, Kn, and qc are already defined in your workspace
        if initializaton == 0
            F_initial=f_i;
            initializaton=1;
        end
        CASE=1;
        % Check if the force is less than Frelax
       continue;
        
        
   

       %% CONTACT PARTICLE FILTER
        
        ind=1;
        %is_initialized=false;

         if size(Point_intersected,1)>1
             Point_intersected=Point_intersected';
         end
         Point_intersectedActualFrame=double(inv(T)*[Point_intersected';1]);
        disp('Point_intersectedActualFrame:')
        disp(vpa(Point_intersectedActualFrame(1:3)',2))
           ErrorBeforeCPF_ActualFrame=abs(Point_intersectedActualFrame(1:3)-point);
          disp('error before CPF:')
           disp(vpa(ErrorBeforeCPF_ActualFrame,3))
              figure(f6),scatter3(point(1),point(2),point(3),'h', 'filled' ,'SizeData', 50);
            % Add a text label
            hold on
                figure(f6),text(point(1),point(2),point(3), 'Real Point', 'FontSize', 6, 'HorizontalAlignment', 'left');
                if mod(t0,10)
                    save('sharedData.mat', 'point', 'link_collided','index','chi','Q_sampled','Residual_calculated','Point_intersectedActualFrame','f6');
                end
                if firstTime
                   parfeval( @CPF_script, 1); % 0 means no output needed
                   disp('poolattivato')
                   firstTime=false;
               end
                
                
                load('sharedVar2')
                CalculatedPoint=Point_intersected;
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


        

%% Collaboration
    
end
%Residual_calculated
%ExternalTauSOSM_calculated
%ExternalTauSOSML_calculated
%TauExtForce
samples2=samples;
figure()
plot(time(samples2:index-20), TauExtForce(samples2:index-20,:)', 'r', 'LineWidth', 2);
hold on;
plot(time(samples2+1:index-1-20), Residual_calculated(samples2+1:index-1-20,:), 'g', 'LineWidth', 2);
hold off
plots;

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


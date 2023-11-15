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

% Here all the parameters to decide for the computetion
tf=10; % final time
MaxVelocity=100;
% External force applied to the link LINK in a point POINT in respect to
% the reference frame of the link
ExternalForceApplied=[1 2 3]'; 
ExternalForceAppliedActualFrame=[3 4 3]';
point = [0.01;-0.04;0.02;1];  
      pointZ=-0.07;
    theta=pi/3; 
gain = 1000*diag([100, 60, 160, 200, 120, 80, 125]); %gain for the residual calculation
link=3; % real link subjected to the force 
radius=0.05; % radius of the circle that the End Effector(EE) of the robot must follow
DeltaT = 0.001; % sampling time of the robot
%q0(1:7)=[0,0,pi/2,0,0,0,0]; % q initialized, the real q initialized is in
% parameters.m
frequency = 1; %frequency*time is the angle of the point of the circle
% in particular frequency represent the angular velocity that the EE must
% follow
threshold_Collision = 0.002; % threshold in order to calculate the residuals, if <0.1 the link is influenced by an external force
threshold_sigma= 0.014; % threshold for the sigma calculated
samples=20; % samples to calculate the residual, in the other projects I think more than 300 samples are used 
% but it depends by the sampling time, since the residual calculation start
% after samples instant




%% Initialization


n = 7; %number of joints
Bsampled = cell(1, 1000);
Residual_calculated=zeros(100,7);
%Gain to calculate the residual, is high enough the residuals are equal to
%the extarnal torque due to the external force

S=zeros(7,7);
S_fext =[0 -ExternalForceAppliedActualFrame(3) ExternalForceAppliedActualFrame(2) ; ExternalForceAppliedActualFrame(3) 0 -ExternalForceAppliedActualFrame(1) ; -ExternalForceAppliedActualFrame(2) ExternalForceAppliedActualFrame(1) 0 ];
m=-S_fext*point(1:3);
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
TAUnew = zeros(samples,n);
F_EXT_RECON = zeros(samples, 3);
TAU_PREDICT = zeros(samples, n);
TAU_FRICTION = zeros(samples, n);


Niterations=10;
speed=1;
num_part=20;
chi = zeros(3,num_part);
    
Jtranspose_link=zeros(7,3);

index = 1;
%save the variables with a specific name
filename="Force"+ExternalForceApplied(1)+ExternalForceApplied(2)+ExternalForceApplied(3)+"Point"+point(1)+point(2)+point(3)+"Link"+link+".mat";
parameters;


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
  

  %% cylinder

  
  radii = 0.05;           % radii of cyl
    cnt = [0, 0,0];       % [x,y,z] center cyl
    height = -0.1;      % height of cyl, is negative because the frame is on the top 

    [X,Y,Z] = cylinder(radii);
    X = X + cnt(1) ;
    Y = Y + cnt(2) ;
    Z = Z * height;
    chi = zeros(3,num_part);
    point(3)=pointZ+ cnt(3);
    point(1)=radii*cos(theta)+ cnt(1);
    point(2)=radii*sin(theta) +  cnt(2);
    point
    m=-S_fext*point(1:3)
    

    



%% Additional task to add to the null space of the principal task
% I tried to mantain the z axis of the end effector constant

% syms t QQ(t) Q1(t) Q2(t) Q3(t) Q4(t) Q5(t) Q6(t) Q7(t) 
%  T = QtoP(q0(1:7),7);
%  p_0 = T(1:3,4)+[0; 0; 0.31];
 
% QQ(t) =[ Q1(t) Q2(t) Q3(t) Q4(t) Q5(t) Q6(t) Q7(t)];
%FinalOrientation = vpa(simplify(T(1:3,1:3)),3); % final orientation at
%every time instant in order to add the final orientation
% T_Q = QtoP(QQ(t),7);
% diff(T,t);cos(Q6(t))*(cos(Q2(t))*cos(Q4(t)) + cos(Q3(t))*sin(Q2(t))*sin(Q4(t))) + sin(Q6(t))*(cos(Q5(t))*(cos(Q2(t))*sin(Q4(t)) - 1.0*cos(Q3(t))*cos(Q4(t))*sin(Q2(t))) + sin(Q2(t))*sin(Q3(t))*sin(Q5(t)))
% J5= jacobian(norm(-T_Q(1:3,3)),QQ(t));
 
%% Now there is the matlab simulation of the movement

while (t0<tf)%(frequency * (t0) < 2*pi) % it ends when a circle is completed
     disp(t0);  
    %figure(f1);
     
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
    
  %  figure(f1),scatter3(p_ref(1,1),p_ref(2,1),p_ref(3,1),'g','SizeData',20)
    hold on
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
        printf('the robot is subjected to a force')
        position=p;
        d2p_ref = Kp * err_p  - Kd *dp ;
    end
    q0(8:14);


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
        J_actualframe=T_actualframe(1:3,1:3)'*J_force;
        

      % TauExternalForce0 = vpa((transpose(J_actualframe) * ExternalForceAppliedActualFrame)',3)
       TauExternalForce =(transpose(J_force)*ExternalForceApplied)';
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




    PointOfApplication(index,:)=QtoP(Q_sampled(end,:),link)*point;

    QD_sampled(index,:)=q0(8:14);
    QDD_sampled(index,:)=acc;


    TAU_sampled(index,:)=TauFL';


        B_sampled{index}=B;
        S_sampled{index}= S;
        g_sampled{index}=g;
        h_sampled{index}=S'*q0(8:14)'-g;

        
        
    
    %% Residual Calculation and Force and point reconstruction
    if index>samples % in order to have enough samples to calculate the residuals
        fprintf('Starting of the Residual calculation:\n')
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
        r=R(end,:);
        %errorTorque=abs(R(end,:)-TauExternalForce)
        Residual_calculated(index,:)=R(end,:);
        Sigma_calculated(index)=Sigma(end);
        %figure(f4),plotTorque(TauExtForce,Residual_calculated,index, 3,DeltaT)
        [is_collided_sigma] = getLink_withSigma(Sigma(end, :),threshold_sigma,QD_sampled(end, :));


        [link_collided(index),is_collided_r] = getLink_withResidual(R(end,:),threshold_Collision);
  
                is_collided(index)=0;
  
            if is_collided_r==1 && is_collided_sigma==1
                is_collided(index)=1;
            end
            if is_collided(index)==0
                link_collided(index)=0;
            end
            link_collided(index)

        %LINKK=link_collided(index)

        if is_collided(index) == 1
                %J_withwrenches = ComputePoint_withWrenches(Q_sampled(index,:),link_collided(index));
                J_withwrenches = ComputePoint_withWrenches(Q_sampled(index,:),link);
                
                wrenches=pinv(J_withwrenches')*R(end,:)';



                %ExternalForce_Real =(pinv(transpose(J_force))*Tau_residual')'
        
                f_i=wrenches(1:3);
                m_i=wrenches(4:6);

                error1 = [ExternalForceAppliedActualFrame;m]-wrenches
               
                % Calculate the SVD of matrix A
                [U, S, V] = svd(J_withwrenches');
                s = diag(S);
                tol = max(size(J_withwrenches')) * eps(max(s));
                r = sum(s > tol);
                U_r = U(:, 1:r);
                V_r = V(:, 1:r);
                S_inv = diag(1./s(1:r));
                wrenches2 = V_r * S_inv * U_r' * R(end,:)';
               
                error2 = [ExternalForceAppliedActualFrame;m]-wrenches2
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
                    W = 300*0.001*diag(rand(6, 1));
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


               % null-space method
               norm(error1)
               norm(error2)
               norm(error3)
               norm(error4) 
               return;
               % Linear-Quadratic Optimization
               % non so come usarla
               % prova col quadratic programming usando inequality
               % costraints.



            
%% now intersect the line of points that generate the torque m with a cylinder posed
% in the center of the reference fram, this is a simplified version since
% the real link is not cylindrical
          
                
              Sf_i=[0 -f_i(3) f_i(2) ; f_i(3) 0 -f_i(1) ; -f_i(2) f_i(1) 0 ];
%                 radii = 0.05;   
%                  x = radii*cos(pi/3)+ cnt(1);
%                  y = radii*sin(pi/5) +  cnt(2);
%                  z=height;
%                  p_dc=[x, y,z]'

                radii=0.01;
                p_dc=Sf_i*m_i/(norm(f_i))^2
                p_dc=[0;0.05;-0.05];
               
               % p_dc=pinv(-Sf_i)*m_i
                
                Lambda_coefficient=f_i/(norm(f_i))^2;
                H=([0;0;height]-cnt')/norm(p_dc-cnt);
                w=p_dc-cnt';
                p_dc(3)'*p_dc(3)
            (p_dc(1:2)'*p_dc(1:2))-(p_dc'*H)^2
            radii.^2
                A=Lambda_coefficient'*Lambda_coefficient-(Lambda_coefficient'*H)^2;
                B=2*((Lambda_coefficient'*w)-(Lambda_coefficient'*H));
                C=(w'*w)-(w'*H)^2-radii.^2;
                lambdas = roots([A, B, C]);
                PointOnTheCylinder(:,1)=p_dc+lambdas(1)*Lambda_coefficient;
                PointOnTheCylinder(:,2)=p_dc+lambdas(2)*Lambda_coefficient;
                if PointOnTheCylinder(3,1) >height && PointOnTheCylinder(3,1)<0
                    PointRetrieved=PointOnTheCylinder(:,1);
                elseif PointOnTheCylinder(3,2)>height && PointOnTheCylinder(3,2)<0
                    PointRetrieved=PointOnTheCylinder(:,2);
                end
             raggio=  sqrt(PointRetrieved(1)^2+PointRetrieved(2)^2)
                %fprintf('the point calculated with the residuals is: %d\n',PointRetrieved)
%                 m=-S_fext*PointOnTheCylinder1
%                 m=-S_fext*PointOnTheCylinder2

                

    
                error_initialization=abs(point(1:3)-PointRetrieved)
               
                figure(f5);
                plot3(point(1),point(2),point(3),'b');
                hold on 
                plot3(PointRetrieved(1),PointRetrieved(2),PointRetrieved(3),'b');

                
                % T= QtoP(Q_sampled(end,:),link_collided(end));
                T= QtoP(Q_sampled(end,:),link);
                ForcePoint=T*[PointRetrieved;1] ;%point in frame 0
                ForcePointApplication(index,:)=ForcePoint(1:3);
                
                Force=T*[f_ext;1]; %force in frame 0 

                NumberNonCollidedSamples=0;
        else
            NumberNonCollidedSamples=NumberNonCollidedSamples+1;
                
        end
        if NumberNonCollidedSamples>10
            chi = zeros(3,num_part);
        end

        

        %lambda = -1:0.01:1;
        
        
        %estimated_cp=p_dc+lambda*f_ext/(norm(f_ext));
 
        
        
   

       %% CONTACT PARTICLE FILTER
        
        ind=1;
        %is_initialized=false;
        for i=index-Niterations:speed:index
            break;
            %is_collided
            % starting Niterations before the end the contact particle
            % filter is iterated Niterations times until the last index.
            % this is done at every time instant so if the number of
            % iterations is high the code is very slow 
        
       
           
           
           

            %% plot the cylinder
            %here the cylinder representing the link collided is
            %represented in a 3D plane
            %transformation of both the circumference upper and lower using
            %R*p+t
                    
                    if link_collided(i) == 0
                        
                        continue;
                        
                    end

            
                  T= QtoP(Q_sampled(i,:),7);
            Rotation = T(1:3,1:3);
             tran = T(1:3,4);


        hom1 = Rotation*[X(1,:);Y(1,:);Z(1,:)]+tran;
        hom2 = Rotation*[X(2,:);Y(2,:);Z(2,:)]+tran;
        X_prime(1,:) = hom1(1, :);
        X_prime(2,:) = hom2(1, :);

        Y_prime(1,:) = hom1(2, :);
        Y_prime(2,:) = hom2(2, :);

        Z_prime(1,:) = hom1(3, :);
        Z_prime(2,:) = hom2(3, :);

        

        
            % estimated_contat_point_prime is the point calculated in the initialization phase
            % in respect of the world frame

            
            estimated_contat_point_prime = Rotation*PointRetrieved+ tran;
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
           
            [chi, W_prime] = cpf(num_part, chi, Q_sampled(i,:), Residual_calculated(i,:), [PointRetrieved' 1]',link_collided(i),is_initialized);
            is_initialized = true; 
            
            
           hold on
           chi;
            
            
             real_point=computeBari(chi);
             
             e2(:,ind)=abs(point(1:3)-real_point');
             ind=ind+1;
             contact_point_PF = computeBari(Rotation*chi+tran);

             [fval(end+1)] = observationModel(eye(7), Q_sampled(i,:),Residual_calculated(i,:),[ForcePointApplication(i,:) 1]',link_collided(i));
              % here is plotteed the error of between the point calculated
              % after i iteration of the CPF in respect to the real point
              % calculated
            figure(f4),plot(1:length(e2),e2);

        %    drawnow() 



        end
        
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




%% Collaboration
    
end
 save(filename)
 save('Residuals_calculatedlink7','Residual_calculated')
return;
 robot_plot;
 



main_Carrieri;



function plotForces(out, dofs, lineWidth)
    for i=1:3
        plot(out.tout, out.contact_force.Data(:,i), "lineWidth", lineWidth);
        hold on
        plot(out.tout, out.estimated_force.Data(:,i), "lineWidth", lineWidth);
    end
    legend("real F_x", "estimated F_x", "real F_y", "estimated F_y" , "real F_z", "estimated F_z");
    grid on
end

function plotForces2(time,Fc, lineWidth)

    for i=1:3
        plot(time(1:length(Fc(i,:))), Fc(i,:), "lineWidth", lineWidth);
        hold on 
    end

    grid on
end
function plotTorque(TauExtForce,Residual_calculated,index, lineWidth,DeltaT)
    legend_torque = [];
    
    for i=1:7
        plot(i*DeltaT, Residual_calculated(index,i), "lineWidth", lineWidth);
        hold on
        plot(i*DeltaT, TauExtForce(index,i), "lineWidth", lineWidth);
        hold on
        real_contact_str = "real tauk_"+i;
        estimated_contact_str = "estimated tauk_"+i;
        legend_torque = [legend_torque, [real_contact_str, estimated_contact_str]];
    end
    legend(legend_torque);
    grid on
end




function plotArrow(point, direction, arrow_color, arrow_size)
        %this plot the arrow at the beginning of the line <---
        last_point1 = point + 0.01*(direction+tan(60));
        last_point2 = point + 0.01*(direction+-tan(60));
        cf_plot1 = line([point(1), last_point1(1)], [point(2), last_point1(2)], [point(3), last_point1(3)],"color", arrow_color, "lineWidth", arrow_size);
        cf_plot2 = line([point(1), last_point2(1)], [point(2), last_point2(2)], [point(3), last_point2(3)],"color", arrow_color, "lineWidth", arrow_size);
end

%plot num_part particlces on the cylinder sourface where
%r is the cylinder radius h is its height and c its center 
function plotParticles(chi,R, t)
    chi = R*chi(:,:)+t;
    for i=1:length(chi)
        hold on 
        plot3(chi(1,i),chi(2, i), chi(3, i),".", "Color", "red");
    end
   
end


%plot the RF in a generic point with a generic orientation. The lenght of
%the axis are fixed but eventually can be passed as a parameters
function plotRF(R, tran)

    origin = tran;
    length_axis = 0.05;
    x = R*[length_axis 0 0]'+tran;
    y = R*[0 length_axis 0]'+tran;
    z = R*[0 0 length_axis]'+tran;

    line([origin(1) x(1)], [origin(2) x(2)], [origin(3) x(3)],"color", "r", "lineWidth", 1);
    hold on 
    line([origin(1) y(1)], [origin(2) y(2)], [origin(3) y(3)],"color", "g", "lineWidth", 1);
    hold on 
    line([origin(1) z(1)], [origin(2) z(2)], [origin(3) z(3)],"color", "b", "lineWidth", 1);
    
end


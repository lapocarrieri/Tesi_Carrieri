load("test2 .mat");


addpath 'C:\Users\lapoc\OneDrive\Desktop\MATLAB_Simulator\Dynamics'
addpath 'C:\Users\lapoc\OneDrive\Desktop\MATLAB_Simulator\CPF'
% If the joint friction is taken into account, open forceReconstruction and
% uncomment line 22
gain = 1000*diag([100, 60, 160, 200, 120, 80, 125]);

n = 7; %number of joints



Q = Q(1:samples, 1:n);
%Q = NoncausalButterworthFilter(Q);

TAU = TAU(1:samples, 1:7);
%TAU = NoncausalButterworthFilter(TAU);

estimated_cp = QtoP(Q(samples,link));


%QD = TimeDerivative(Q, DeltaT);
%QDD = TimeDerivative(QD, DeltaT);



% P = zeros(samples,1);
% PD = zeros(samples,1);
% 
% for t=1:samples
%     q = Q(t, :)';
%     qd = QD(t, :)';
%     
%     T = get_transform(q,needle_length);
%     P(t) = T{7}(3,4)';
% 
%     J = Geometric_Jacobian(q, needle_length, n);
%     PD(t) = J(3,:)*qd;
% end
% P = NoncausalButterworthFilter(P);
% PD = NoncausalButterworthFilter(PD);


[R, F_EXT_RECON, TAU_PREDICT,TAU_FRICTION] = forceReconstruction(gain, samples, n, DeltaT, Q, QD, QDD, TAU);
TAU_error = TAU_PREDICT(end-20:end,:) -TAU(end-20:end,:)

Residual_error = (R(end,:)-TauExtForce(end,:))



threshold_Collision = 0.05;
threshold_Residual = 0.1;
initial_idx = 1;
final_idx = 10;
speed = 1;  
num_part=10;
    
    r = 0.045;           % radii of cyl
    cnt = [0,0,0];       % [x,y,z] center cyl
    height = -0.05;      % height of cyl, is negative because the frame is on the top 
    color = [128 128 128]/255;    % color of cyl
    nSides = 100;        % number of "sides" of the cyl
    hold on
    
    [X,Y,Z] = cylinder(r);
    X = X + cnt(1); 
    Y = Y + cnt(2); 
    Z = Z * height;
    chi = zeros(3,num_part);
    F_EXT_RECON;
    ACTUAL_RESIDUAL = R(end,:);
    TauExternalForce;
    error = ACTUAL_RESIDUAL - TauExternalForce;
   
    link=getLink(R(end,:),threshold_Residual);
    for i=initial_idx:speed:final_idx
        
        [chi, W_prime] = cpf(num_part, chi, Q(end,:), R(end,:), estimated_cp,link)
        contact_point_PF = computeBari(J_c(end,1:3,1:3)*chi+J_c(end,1:3,4));
        [~,fval(end+1)] = observationModel(eye(7)*7, Q(end,:),R(end,:),contact_point_PF);
        if norm(R(end,:)>threshold_Collision)
            is_collided
        end
         if(is_collided)      
            hold on 
            p_est_cont =  plot3(estimated_contat_point_prime(1), estimated_contat_point_prime(2), estimated_contat_point_prime(3), "c.", "lineWidth", 15);
            hold on 
            p_cont = plot3(contact_point_prime(1), contact_point_prime(2), contact_point_prime(3), "g.", "lineWidth", 15);
            hold on 
            
            plot3(contact_point_PF(1), contact_point_PF(2), contact_point_PF(3), "bx", "lineWidth", 3);
         else
             legend off
         end


    end

    

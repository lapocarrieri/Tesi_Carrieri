  % Sigma: standard devation of the measurements
% q: current joint angle
% gamma: estimated external torque
% chi: particles at the current time
%function [x,fval] = observationModel(Sigma, q, gamma,chi,link)
function [fval] = observationModel2( q, gamma,chi,link)
    
    %to test it observationModel(eye(4)*1,[0 -pi/3 pi/4 pi/2 0 0 0],[1 3 2
    %0 0 0 0],[1;1;1;1],4)  n
    
   % [Jc,~] = compute_jacobian(q,chi,link);
    Jc=ComputePoint_withWrenches(q,link)
     
     

     %H_opt = Jc*Sigma*Jc';
     %f_opt = gamma' *Sigma*Jc';
     
     %in case you want to include the friction cone constraints
%      %H matrix---------------------------
%      H_opt_prime = Jc*Sigma*Jc';
%      H_tr = zeros(3,4);     %H top right
%      H_bl = zeros(4,3);     %H bottom left
%      H_br = zeros(4,4);     %H bottom right
%      H_opt = zeros(7,7);
%      H_opt(1:3,1:3) = H_opt_prime;
%      H_opt(1:3,4:7) = H_tr;
%      H_opt(4:7,1:3) = H_bl;
%      H_opt(4:7,4:7) = H_br;
%      %-----------------------------------
%      %f vector---------------------------
%      f_opt_prime = gamma*Sigma*Jc';
%      f_opt = zeros(7,1);
%      f_opt(1:3,1) = f_opt_prime;
%      f_opt(4:7,1) = zeros(4,1);
%      %-----------------------------------
     
%      lb = [-100 -100 -100 -100 0 0 0 0]';
%      Aeq = [eye(3), F];
%      beq = zeros(3,1);
     
%      options =  optimset('Display','off');
%      [x, fval] = quadprog(H_opt,f_opt,[],[],[],[],[],[],[], options);
     
     
     
        %if instead of the optimization problem you want to use the classical 
        %technique to compute the cost function
     %Fc = pinv(Jc')*gamma'
   % Fc = pinv(Jc')*gamma;

     W=pinv(Jc')*gamma';
    f_i=W(1:3);
     Sf_i=[0 -f_i(3) f_i(2) ; f_i(3) 0 -f_i(1) ; -f_i(2) f_i(1) 0 ];

     
     m=-Sf_i*chi(1:3);
     disp(W(4:6))
     m
     W(4:6)=m;
     
      %Fc = (Sigma*Jc'*inv(Jc*Sigma*Jc'))'*gamma';%weighted pinv
     fval = (gamma'-Jc'*W)'*(gamma'-Jc'*W);    

end
% num_part: numeber of particles
% chi_prev: particle at the previous step
% q: joint angles
% gamma: estimated external torque
% estimated_cp: estimated contact point with the deterministic method used in
%                the initialization phase
function [chi, W_prime] = cpf(num_part, chi_prev, q, gamma, estimated_cp,link)

    W_prime = zeros(1, num_part); %cancellare
    theta_hat = atan2(estimated_cp(2),estimated_cp(1)); 
    global is_initialized;

    
    r = 0.045;           % radii of cyl
    cnt = [0,0,0];       % [x,y,z] center cyl
    height = -0.05;      % height of cyl, is negative because the frame is on the top 
    C = [0 0 0]';        %center of bottom circle
    H = [0 0 height]';   %center of top circle
    Sigma = eye(6)*1;
    theta = zeros(1, num_part);
    if(~is_initialized)
        disp("Initialization");
        chi = chi_prev;
        for i=1:num_part
            %theta_v =theta_hat +pi*rand();         %uniform distr on one part of the cylinder
            theta_v = normrnd(theta_hat+pi/2, 0.5); %gaussian distribution around the estimated contact point
            x = r*cos(theta_v)+ cnt(1);
            y = r*sin(theta_v) +  cnt(2);
            z = rand()*height + cnt(3);
            theta(1,i) = theta_v;
            chi(:, i) = [x y z];
        end
        is_initialized = true;
    else
        %random walk of each particles
        m=randi(2,1)-1;
        m(~m)=-1;                                        %m is random 1 -1 
         %chi = chi_prev + m .* rand(3,1)*0.001;            %random walk model
        
         
        
        
        
        X = zeros(3, num_part);                           %to store the points on the cylinder line   
        W = zeros(1, num_part);                           %to store the weigths
        W_prime = W; 
        for i = 1:num_part
            
            for j=1:3
                chi(j,i) = chi_prev(j,i)  + m .*  0.001*randn();
            end
            %I project the particle on the closet point on the cylinder axis
            lambda = (dot((chi(:,i)-C),(H-C)))/dot(H-C,H-C);
            X(:,i) = C+lambda*(H-C); 
            chi(:,i) = compueIntersaction(X(:,i), chi(:,i))
            Sigma
            q
            link
            gamma
            [fval] = observationModel(Sigma, q, gamma, chi(:,i),link);
            fval = fval + gamma*Sigma*gamma';
                          
             W(1,i) = exp(-0.5*fval);

             W_prime = W;
             
        end

          
     W = W./sum(W); % normalization 
     new_indeces = resample( num_part, W); %resampling
     chi = chi(:, new_indeces);            %maintain the best particles

    end
end
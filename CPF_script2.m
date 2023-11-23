clc;
clear all;
close all;
indixes=1;
           num_part=50;
           Niterations=10;
           load('initialization5.mat','Meshes','triangles','point','link')
               initialize=false;
                  addpath 'Dynamics'
                  f6=figure;

   addpath 'Functions'
  figure(f6);
            scatter3(point(1),point(2),point(3),'h', 'filled' ,'SizeData', 50);
            hold on
            
                text(point(1),point(2),point(3), 'Real Point', 'FontSize', 6, 'HorizontalAlignment', 'left');
             
       speed=1;
while true

            load('sharedData7.mat');
            normBefore=norm(Point_intersectedActualFrame(1:3)-point)
            J_w = ComputePoint_withWrenches(Q_sampled(index,:),link);
       if link_collided(index) > 0
           disp('CPF:')
           is_initialized=false;
           indixes=indixes+1;
           
             % Add a text label
            
       for i=1:speed:Niterations-1
            
            %disp(i+'-th iteration for the CPF');
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
                    
                    
                        
                     

            
          

        

        
            % estimated_contat_point_prime is the point calculated in the initialization phase
            % in respect of the world frame

            
            
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
     
            generated_points=zeros(3,num_part);
            
            [chi,chi2, W_prime,generated_points] = cpf_RealPoint2(num_part, chi, Q_sampled(index-10:index,:), Residual_calculated(index-10:index,:), Point_intersectedActualFrame,link,is_initialized,Meshes,triangles,generated_points,point,i,Niterations,J_w);
            
            
            
            is_initialized=true;

            figure(f6),scatter3(chi(1,:),chi(2,:),chi(3,:),'y', 'filled' ,'SizeData', 25);
            
            
             CalculatedPoint=computeBari(chi);
             CalculatedPoint2=computeBari(chi2);
              error1=norm(CalculatedPoint(1:3)'-point);
              error2=norm(CalculatedPoint2(1:3)'-point);
             %ErrorAfterCPF(:,ind)

             
             save('sharedVar3','CalculatedPoint');
       
            ErrorAfterCPF1(:,i)=norm(CalculatedPoint(1:3)'-point)
             ErrorAfterCPF2(:,i)=norm(CalculatedPoint2(1:3)'-point)
             
  

        pause(0.1);

       end
               % Continuous execution code
              
        % ...
           %figure(f6),scatter3(CalculatedPoint(1),CalculatedPoint(2),CalculatedPoint(3),'p', 'filled' ,'SizeData', 50);
            % Add a text label
            textname="CalculatedPoint 5-th iteration";
            %figure(f6),text(CalculatedPoint(1),CalculatedPoint(2),CalculatedPoint(3), textname, 'FontSize', 6, 'HorizontalAlignment', 'left');
            
            end
        pause(0.1); % Prevents MATLAB from freezing
            
end
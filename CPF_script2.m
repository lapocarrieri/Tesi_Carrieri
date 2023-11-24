clc;
clear all;
close all;
indixes=1;
           num_part=100;
           Niterations=20;
           load('initializations5.mat','Meshes','triangles','point','link')
               initialize=false;
                  addpath 'Dynamics'
                  f6=figure;

   addpath 'Functions'
  figure(f6);
            scatter3(point(1),point(2),point(3),'h', 'filled' ,'SizeData', 50);
            hold on
            
                text(point(1),point(2),point(3), 'Real Point', 'FontSize', 6, 'HorizontalAlignment', 'left');
             
       speed=1;
       load("plotDatas")




         hold on

kuka = importrobot('./models/kuka_lwr.urdf');

addVisual(kuka.Base,"Mesh","./visual/base.STL")
addVisual(kuka.Bodies{1},"Mesh","./visual/link_1.STL")
addVisual(kuka.Bodies{2},"Mesh","./visual/link_2.STL")
addVisual(kuka.Bodies{3},"Mesh","./visual/link_3.STL")
addVisual(kuka.Bodies{4},"Mesh","./visual/link_4.STL")
addVisual(kuka.Bodies{5},"Mesh","./visual/link_5.STL")
addVisual(kuka.Bodies{6},"Mesh","./visual/link_6.STL")
addVisual(kuka.Bodies{7},"Mesh","./visual/link_7.STL")

kuka.Gravity = [0,0,-9.81];
kuka.DataFormat = 'row';

%EF=0.1*log10(ExternalForce+0.1)+0.1;
         % Difference

rateCtrlObj = rateControl(10000);

H = [-1.1 pi/4 0 1.3*pi -1 0 0];

prova = kuka.show(H,'visuals','on','collision','off');
prova.CameraPosition = [-2,7,6]; 
joints=Q_sampled;
chi3=zeros(3,num_part);
while true

            load('sharedData7.mat');
            Niterations=20;
            num_part=100;
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
        
            [chi,chi2,chi3, W_prime,generated_points] = cpf_RealPoint3(num_part, chi3, Residual_calculated(index,:), Point_intersectedActualFrame,link,is_initialized,Meshes,triangles,generated_points,point,i,Niterations,J_w);
            
           Q_sampled(index,:)=[0 0 0 0 0 0 0];
            
            is_initialized=true;
            T=QtoP(Q_sampled(index,:),link);
            chiWorldFrames=T*[chi3;ones(1,num_part)];
            figure(f6);
                                    Points=matrix(:,1:4,link);
             
                        
                        matrix=(T*Points')';       
                        x = matrix(:, 1);
                    y = matrix(:, 2);
                    z = matrix(:, 3);
                    
                    % Plot the data
                    addpath('Functions')
                    plot3(x, y, z, 'b.');
                    title('3D Plot');
                    xlabel('X');
                    ylabel('Y');
                    zlabel('Z');
                    axis equal;
                    grid on;
                    hold on
            scatter3(chiWorldFrames(1,:),chiWorldFrames(2,:),chiWorldFrames(3,:),'y', 'filled' ,'SizeData', 25);
            hold on; % Keep the current plot
            realPoint=T*[point;1];
            scatter3(realPoint(1), realPoint(2), realPoint(3), 'r', 'filled'); % Plot the point
            text(realPoint(1), realPoint(2), realPoint(3), 'Real point'); % Add a label
            prova = kuka.show(Q_sampled(index,:), 'visuals', 'on', 'collision', 'off');
            prova.CameraPosition = [-2, 7, 6]; 
        
            % Additional plotting (force vectors, etc.) can be added here
        
            waitfor(rateCtrlObj);
                    
             CalculatedPoint=computeBari(chi);
             CalculatedPoint2=computeBari(chi2);
             CalculatedPoint3=computeBari(chi3);
             CalculatedPointWorldFrame=T*[CalculatedPoint3';1];
             scatter3(CalculatedPointWorldFrame(1), CalculatedPointWorldFrame(2), CalculatedPointWorldFrame(3), 'g', 'filled'); % Plot the point
            text(CalculatedPointWorldFrame(1), CalculatedPointWorldFrame(2), CalculatedPointWorldFrame(3), 'Calcualted point'); % Add a label
              error1=norm(CalculatedPoint(1:3)'-point);
              error2=norm(CalculatedPoint2(1:3)'-point);
              error3=norm(CalculatedPoint3(1:3)'-point);
             %ErrorAfterCPF(:,ind)
            
             
             save('sharedVar3','CalculatedPoint');
       
            ErrorAfterCPF1(:,i)=norm(CalculatedPoint(1:3)'-point);
             ErrorAfterCPF2(:,i)=norm(CalculatedPoint2(1:3)'-point);
             ErrorAfterCPF3(:,i)=norm(CalculatedPoint3(1:3)'-point);
             
  

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
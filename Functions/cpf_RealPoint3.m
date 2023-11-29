% num_part: numeber of particles
% chi_prev: particle at the previous step
% q: joint angles
% gamma: estimated external torque
% estimated_cp: estimated contact point with the deterministic method used in
%                the initialization phase
function [chi,chi2,chi3, W_prime,generated_points,Festimated] = cpf_RealPoint3(num_part, chi_prev,  gamma, estimated_cp,link,is_initialized,Meshes,triangles,generated_points,point,iteration,Niterations,J_w)
Festimated=1;    
Sigma = eye(7)*1;
    num_part_multiplicator=5;
    matrix = Meshes.Points(:,1:3,link);
    nonZeroRows = any(matrix, 2);
    matrix = matrix(nonZeroRows, :);
    surface_points = matrix(matrix(:, 2, 1) ~= 0, :, :);
    
   syms q1 q2 q3 q4 q5 q6 q7
q = [q1;q2;q3;q4;q5;q6;q7];
J = zeros(4,7);
Jsubs= zeros(4,7);
A = cell(1, link);
C1=cos(q1);
S1=sin(q1);
C2=cos(q2);
S2=sin(q2);
C3=cos(q3);
S3=sin(q3);
C4=cos(q4);
S4=sin(q4);
C5=cos(q5);
S5=sin(q5);
C6=cos(q6);
S6=sin(q6);
C7=cos(q7);
S7=sin(q7);
L = 0.4;
l5=0.39;


%  l1=0.0;     % for real robot where the base frame is on the second joint
%  l2=0.4;
%  l3=0.39;
  l4=0.078;   % EE in the tip of KUKA without auxiliary addition
M = cell(1, 7);
M{1} = [C1,0,S1,0; S1,0,-C1,0; 0,1,0,0; 0,0,0,1];
M{2} = [C2,0,-S2,0; S2,0,C2,0; 0,-1,0,0; 0,0,0,1];
M{3} = [C3,0,-S3,0; S3,0,C3,0; 0,-1,0,L; 0,0,0,1];
M{4} = [C4,0,S4,0; S4,0,-C4,0; 0,1,0,0; 0,0,0,1];
M{5} = [C5,0,S5,0; S5,0,-C5,0; 0,1,0,l5; 0,0,0,1];
M{6} = [C6,0,-S6,0; S6,0,C6,0; 0,-1,0,0; 0,0,0,1];
M{7} = [C7,-S7,0,0; S7,C7,0,0; 0,0,1,l4; 0,0,0,1];


A{1}=M{1};
%through the premultiplication find the 7 transformation matrixes that
%transform the point in the actual frame to the point in the world frame
for j=2:link
    A{j}=A{j-1}*M{j};
end





   % point_on_surface=estimated_cp(1:3);
    
 
    
    
%     figure(figure),scatter3(point_on_surface(1),point_on_surface(2),point_on_surface(3),'g', 'filled' ,'SizeData', 100);
%     hold on
    n = size(surface_points, 1);  % Number of surface points
  
    
    W_prime = zeros(1, num_part); %cancellare

    if(~is_initialized)
            disp("Initialization contact particle filter");
        
            %generated_points = generate_points_nearby( point_on_surface', surface_points, num_part);
            %disp(generated_points);
            
            % Generate random data points
           


            

           
            for i=1:num_part
                    
                    closest_point = estimated_cp(1:3) +  normrnd(0, 0.5,3,1)*0.1;
                    if isempty( closest_point_to_triangle2(matrix, closest_point'))
                            generated_points(:,i)=triangles(:,1,33);
                           
                    else
                        generated_points(:,i) = (closest_point_to_triangle2(matrix, closest_point'))';
                    end
                    
            end
        
             chi(:,:) = generated_points;
              chi2(:,:) = generated_points;
              chi3(:,:) = generated_points;
      
            %scatter3(generated_points(:,1),generated_points(:,2),generated_points(:,3),'b', 'filled' ,'SizeData', 20);
       
       
        
    else
        %random walk of each particles
                                                %m is random 1 -1 
 
        

         %pointt=[ 0.0479, 0.0455,-0.0362];
          
        
        
        X = zeros(3, num_part);                           %to store the points on the cylinder line   
        W = zeros(1, num_part);                           %to store the weigths
        W_prime = W;
        
        [Fm]=pinv(J_w')*gamma';
        for i = 1:num_part

            for j=1:num_part_multiplicator
                    m=randi([0, 1]) * 2 - 1;
                    closest_point(:,j) = chi_prev(:,i) + m .* rand(3,1)* 0.01*(Niterations-iteration);
                        if isempty( closest_point_to_triangle2(matrix, closest_point(:,j)'))
                            Particles(:,num_part_multiplicator*(i-1)+j) = closest_point_to_triangle2(matrix, closest_point(:,j)');
                     
                        else
                        
                         Particles(:,num_part_multiplicator*(i-1)+j) = closest_point_to_triangle2(matrix, closest_point(:,j)');
                        end
                        
                        

                             
                   % Particles(:,num_part_multiplicator*(i-1)+j)=point;
                         
                   
                        
                        fval = (skew_symmetric(Particles(:,num_part_multiplicator*(i-1)+j))*Fm(1:3)-Fm(4:6))'*(skew_symmetric(Particles(:,num_part_multiplicator*(i-1)+j))*Fm(1:3)-Fm(4:6));








                        

                    
                    %disp([fval,Particles(:,num_part_multiplicator*(i-1)+j)'])
                                  
                     W(1,num_part_multiplicator*(i-1)+j) = exp(-0.5*fval);
                     %disp(vpa(norm([0.0479, 0.0455, -0.0362]-Particles(:,num_part_multiplicator*i+j)'),3))
                    %disp( vpa((exp(-0.5*fval))',3))
                    hold on

                    %plot(norm(point-Particles(:,num_part_multiplicator*(i-1)+j)'),(exp(-0.5*fval))','--rs','LineWidth',2,'MarkerEdgeColor','k','MarkerFaceColor','g','MarkerSize',10)
                     W_prime = W;
            end
             
        end
        %scatter3(Particles(1,:),Particles(2,:),Particles(3,:))
        W = W./sum(W);
%         figure();
%                 hold on
%         for i = 1:size(Particles,2)
%             diffVector = Particles(:,i) - point;
%             normDifferences(i) = norm(diffVector);
%         end
% 
%         %Plot the results
%         plot( W,normDifferences, 'o-');
%         xlabel('W');
%         ylabel('Norm of Differences');
%         title('Norm of Differences between Particles and W');
%         grid on;
     new_indeces=resample(num_part, W,num_part); %resampling
    new_indeces2=resample2(num_part, W); %resampling
    new_indeces3=resample3(num_part, W); %resampling
      chi = Particles(:, new_indeces);%maintain the best particles
     chi2 = Particles(:, new_indeces2);
     chi3= Particles(:, new_indeces3);
     

    end
                %scatter3(surface_points(:,1),surface_points(:,2),surface_points(:,3),'r', 'filled' ,'SizeData', 10);
           
            %scatter3(point_on_surface(1),point_on_surface(2),point_on_surface(3),'g', 'filled' ,'SizeData', 40);
            
            
            %disp('size particles')
            %size(chi,2)
end




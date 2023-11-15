% num_part: numeber of particles
% chi_prev: particle at the previous step
% q: joint angles
% gamma: estimated external torque
% estimated_cp: estimated contact point with the deterministic method used in
%                the initialization phase
function [chi, W_prime,generated_points,num_part] = cpf_RealPoint(num_part,next_num_part, chi_prev, q, gamma, estimated_cp,link,is_initialized,Meshes,triangles,f6,generated_points)
    Sigma = eye(7)*1;
    num_part_multiplicator=5;
    matrix = Meshes.Points(:,1:3,link);
    surface_points = matrix(matrix(:, 2, 1) ~= 0, :, :);
    
    point_on_surface=estimated_cp(1:3);
    
    hold off
    
    
    
    
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
                    
                    closest_point = estimated_cp(1:3) +  normrnd(0, 0.5,3,1)*0.05;
                    generated_points(:,i) = (closest_point_to_triangle(triangles, closest_point'))';
            end
        
             chi(:,:) = generated_points;
      
            scatter3(generated_points(:,1),generated_points(:,2),generated_points(:,3),'b', 'filled' ,'SizeData', 20);
       
       
        
    else
        %random walk of each particles
                                                %m is random 1 -1 
 
        

         %pointt=[ 0.0479, 0.0455,-0.0362];
          
        
        
        X = zeros(3, num_part);                           %to store the points on the cylinder line   
        W = zeros(1, num_part);                           %to store the weigths
        W_prime = W;
        figure();
        for i = 1:num_part

            for j=1:num_part_multiplicator
                    m=randi([0, 1]) * 2 - 1;
                    closest_point(:,j) = chi_prev(:,i) + m .* rand(3,1)* 0.07;
                        if isempty( closest_point_to_triangle(triangles, closest_point(:,j)'))
                            continue;
                        end
                        
                     Particles(:,num_part_multiplicator*(i-1)+j) = closest_point_to_triangle(triangles, closest_point(:,j)');
        
                  
                    
                    
                    [fval] = observationModel(Sigma, q, gamma,  Particles(:,num_part_multiplicator*(i-1)+j),link);
                    
                    fval = fval + gamma*Sigma*gamma';
                                  
                     W(1,num_part_multiplicator*(i-1)+j) = exp(-0.5*fval);
                     %disp(vpa(norm([0.0479, 0.0455, -0.0362]-Particles(:,num_part_multiplicator*i+j)'),3))
                    %disp( vpa((exp(-0.5*fval))',3))
                    hold on

                    plot(norm([-0.0438, -0.106, -0.048]-Particles(:,num_part_multiplicator*(i-1)+j)'),(exp(-0.5*fval))','--rs','LineWidth',2,'MarkerEdgeColor','k','MarkerFaceColor','g','MarkerSize',10)
                     W_prime = W;
            end
             
        end
        figure(f6);
        %scatter3(Particles(1,:),Particles(2,:),Particles(3,:))
        hold on
        selected_indices=[];
    for i = 1:size(Particles, 2)
        column = Particles(:, i);
        if ~all(column == [0; 0; 0])
            selected_indices = [selected_indices, i];
        end
    end
     Particles_processed = Particles(:, selected_indices);
     W = W./sum(W); % normalization 
     W_processed = W(:, ceil(selected_indices / 3));
     new_indeces=resample(next_num_part, W_processed,size(Particles_processed,2)); %resampling
     chi = Particles_processed(:, new_indeces);          %maintain the best particles
     
     

    end
               % scatter3(surface_points(:,1),surface_points(:,2),surface_points(:,3),'r', 'filled' ,'SizeData', 10);
            hold on
            %scatter3(point_on_surface(1),point_on_surface(2),point_on_surface(3),'g', 'filled' ,'SizeData', 40);
            num_part=next_num_part;
            
            disp('size particles')
            size(chi,2)
end




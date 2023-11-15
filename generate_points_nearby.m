function generated_points = generate_points_nearby(point_on_surface, surface_points, b)
    
   % distances = vecnorm(surface_points - point_on_surface, 2, 2);
    
   
    sigma = 2;

   distances=  surface_points - point_on_surface ;
    min_distance = min(distances);
    max_distance = max(distances);
    
    generated_points = zeros(b, 3);
  
   
    for i = 1:b
        
        
        distance = generate_random_3d_vector(max_distance, min_distance,sigma)*0.;
        distances=  surface_points - point_on_surface -distance ;
        norms = vecnorm(distances, 2, 2);
       %dist(1:10,:)
       
        

        
        

        [~, idx] = min(norms);
        surface_points(idx, :);
        generated_points(i, :) = surface_points(idx, :);
        surface_points(idx,:)=[];
    end
end
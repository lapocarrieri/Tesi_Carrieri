function [is_collided_sigma] = getLink_withSigma(sigma,threshold_sigma,qd)
    
   
    
    
    is_collided_sigma=1;
    if norm(qd)> 0.02
        is_collided_sigma=0;
       if  abs(sigma) > threshold_sigma
           is_collided_sigma=1;
       end
    end 
end
    

%Computes the intersection between the line of action of the contact force
%and the cylinder that approximated the last link and returns the point on
%the link surface that corresponds to the pushing point
%X, P are points to construct the line
function [p1,p2] = compueIntersaction(d, P)

    r = 0.045;          %cylinder radius
    height = -0.05;     %cylinder height, is negative because the reference
                        %frame is on the top face 
    C = [0 0 0]';       %center of bottom circle
    H = [0 0 height]';  %center of top circle
    
    
   
    % intersection 
    
    w = P - C;
    h = H-C; 
    h_hat = h/norm(h);   
    
    
    a = dot(d,d)-dot(d,h_hat)^2;
    b = 2*(dot(d,w)-dot(d,h_hat)*dot(w,h_hat));
    c = dot(w,w)-dot(w,h_hat)^2-r^2;
    if((b^2-4*a*c)>=0)
        lambda1 = (-b+sqrt(b^2-4*a*c))/(2*a);    
        lambda2 = (-b-sqrt(b^2-4*a*c))/(2*a);

        p1 = P + lambda1*d;
        p2 = P + lambda2*d;
        

       
    else
        p1 = [0 0 0]';
        p2 = [0 0 0]';
        p = [0 0 0]';
    end
    
   
        
end
    

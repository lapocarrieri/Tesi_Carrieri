function [link,is_collided] = getLink_withResidual(R,threshold_Collision)
    link = 0;
    R=abs(R);
   
    is_collided=0;

    for i=1:7
        

        if R(i)>threshold_Collision
            is_collided =1;
            link=i;
        end
    end
end
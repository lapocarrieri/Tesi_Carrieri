function [link] = getLink(R,Maxthreshold,MinThreshold)
    link = 0;
    influenced=1;
    for i=1:7
        norm= norm(R(1:i));
        if R(i)>0.5 
            influenced =1;
        else 
            influenced =0;
        end
        if R(i)<norm/10 && R(i)<threshold
            link = i;
        end

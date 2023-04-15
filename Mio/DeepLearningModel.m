%Create a deep learning model to have a function that given a residual gives as output the link interested
clc
r = zeros(7,1);
addpath 'C:\Users\lapoc\OneDrive\Desktop\MATLAB_Simulator'
addpath 'C:\Users\lapoc\OneDrive\Desktop\MATLAB_Simulator\CPF'
maxForce=100;
linklenghts = 0.2;
k=2;
index=1;
tic



s.Tau = zeros(1:7);
s.link = 0;
s.p = zeros(1:3);
X = repmat(s,1,100000);
syms q q1 q2 q3 q4 q5 q6 q7

    for f=0:maxForce/k:maxForce
        for q1=0:2*pi/k:2*pi
            for q2=0:2*pi/k:2*pi
                for q3=0:2*pi/k:2*pi
                    for q4=0:2*pi/k:2*pi
                        for q5=0:2*pi/k:2*pi
                            for q6=0:2*pi/k:2*pi
                                for q7=0:2*pi/k:2*pi
                                    for link=1:7
                                        q=[q1,q2,q3,q4,q5,q6,q7];
                                       TReal = QtoP(q,link);
                                        for j=0:linklenghts/k:linklenghts
                                            p=TReal*[0 0 j 1]';
                                            p=p(1:3);

                                            [J,~] = compute_jacobian(q,p,link);
                                            TauExternalForce =(transpose(J)*f)';
% 
%                                         qd(index,:)=(q(index,:)-q(index-1,:))/deltaT
%                                         qdd(index,:)=(qd(index,:)-qd(index-1,:))/deltaT
%                                         B=massMatrix(controller,q(1:7));
%                                         Sd=velocityProduct(controller,q(1:7),qd(1:7))';
%                                         g=gravityTorque(controller,q(1:7))';
%                                         Tau_applied(index)  = ((B*qdd')' + friction + (Sd)' + g')';
%                                         index=index+1;
%                                     
                                    
                                           X(index).Tau = TauExternalForce;
                                           X(index).link = link;
                                           X(index).p=p;
                                           X
                                           
                                           if toc>1200
                                               save("Residuals.mat","X")
                                               return;
                                           end

                                        end
                                
 

   
                                    end
                               
                                end
                            end

                        end
                    end
                end
            end
        end
    end
save("Residuals.mat","X")
% q(index,1:7)
% 
% 
%                                                                     %non solo il link ma ancje il punto
%                                     
%                                     
%                                     
%                                     
%                                 TReal = QtoP(q,link);
%                                     for j=0:linklenghts/k:linklenghts
%                                         p=Treal*[0 0 j]';
%     
%                                 Residual = QtoP
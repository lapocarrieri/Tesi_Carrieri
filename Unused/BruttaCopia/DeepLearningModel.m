%Create a deep learning model to have a function that given a residual gives as output the link interested
clc
clear all
r = zeros(7,1);
addpath 'C:\Users\lapoc\OneDrive\Desktop\MATLAB_Simulator'
addpath 'C:\Users\lapoc\OneDrive\Desktop\MATLAB_Simulator\Functions'
maxForce=10;
linklenghts = 0.2;
k=100;
index=1;
tic



samples = 1;

syms q q1 q2 q3 q4 q5 q6 q7
   
    f1= maxForce.*rand(1000000,1);

    f2= maxForce.*rand(1000000,1);
    f3= maxForce.*rand(1000000,1);
    q1 = 2*pi.*rand(1000000,1);
    q2 = 2*pi.*rand(1000000,1);
    q3 = 2*pi.*rand(1000000,1);
    q4 = 2*pi.*rand(1000000,1);
    q5 = 2*pi.*rand(1000000,1);
    q6 = 2*pi.*rand(1000000,1);
    q7 = 2*pi.*rand(1000000,1);
   
   distance1=linklenghts.*rand(1000000,1);
   distance2=linklenghts.*rand(1000000,1);
   distance3=linklenghts.*rand(1000000,1);
   N = 10; % specify desired length of vector
    link = randi([1 7], 1, 1000000)';
    i=1;
while(i<5.6192e+05)
                                        

                                  
                        
                                        q=[q1(i),q2(i),q3(i),q4(i),q5(i),q6(i),q7(i)];
                                       TReal = QtoP(q,link(i));
                                        link(i)
                                        
                                            p=TReal*[distance1(i) distance2(i) distance3(i) 1]';
                                            
                                     
                                         
                                            [J,~] = compute_jacobian(q,p,link(i));
                                            
                                            TauExternalForce =(transpose(J)*[f1(i) f2(i) f3(i)]')'

% 
%                                         qd(index,:)=(q(index,:)-q(index-1,:))/deltaT
%                                         qdd(index,:)=(qd(index,:)-qd(index-1,:))/deltaT
%                                         B=massMatrix(controller,q(1:7));
%                                         Sd=velocityProduct(controller,q(1:7),qd(1:7))';
%                                         g=gravityTorque(controller,q(1:7))';
%                                         Tau_applied(index)  = ((B*qdd')' + friction + (Sd)' + g')';
%                                         index=index+1;
%                                     
                                    
                                    
                                    
                                         
                                            Tau_calculated(i,link(i),1:7) = TauExternalForce;
                                           
                                           p_calculated(i,:)=p;
                                           
                                           
%                                            if toc>2400
%                                                save("Residuals.mat","X")
%                                                return;
%                                            end
                                    
                                                                  
    

   i=i+1;
   
end
toc
   Tau_calculated=vpa(Tau_calculated,3);
   p_calculated=vpa(p_calculated,3);
save("Residuals3.mat","Tau_calculated","link_calculated")
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
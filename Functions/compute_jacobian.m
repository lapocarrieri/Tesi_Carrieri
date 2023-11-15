function [J,Jsubs] = compute_jacobian(qq,point,link)
    %d0 d1 d2 d3 real to add when known
    if size(point,1)==3
        point=[point;1];
    end
    %example of its use is: [J,Jsubs]=compute_jacobian([1 2 0 0 1 1 3 0 0.2 0.3 0.4 0.3 0 0],[1;2;2],7)
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

% M1 = [C1,0,S1,0; S1,0,-C1,0; 0,1,0,l1; 0,0,0,1];
% M2 = [C2,0,-S2,0; S2,0,C2,0; 0,-1,0,0; 0,0,0,1];
% M3 = [C3,0,-S3,0; S3,0,C3,0; 0,-1,0,l2; 0,0,0,1];
% M4 = [C4,0,S4,0; S4,0,-C4,0; 0,1,0,0; 0,0,0,1];
% M5 = [C5,0,S5,0; S5,0,-C5,0; 0,1,0,l3; 0,0,0,1];
% M6 = [C6,0,-S6,0; S6,0,C6,0; 0,-1,0,0; 0,0,0,1];
% M7 = [C7,-S7,0,0; S7,C7,0,0; 0,0,1,l4; 0,0,0,1];
% now define the 2 7D vectorscontaining the homogeneus transformation
% matrixes

A{1}=M{1};
%through the premultiplication find the 7 transformation matrixes that
%transform the point in the actual frame to the point in the world frame
for j=2:link
    A{j}=A{j-1}*M{j};
end

% A{7} = (((((M1*M2)*M3)*M4)*M5)*M6)*M7;
% A{6}= ((((M1*M2)*M3)*M4)*M5)*M6;
% A{5} = (((M1*M2)*M3)*M4)*M5;
% A{4} = ((M1*M2)*M3)*M4;
% A{3} = (M1*M2)*M3;
% A{2} = M1*M2;
% A{1} = M1;

%% Point calculation
% now, given the position of the point application that is the result of
% the residual.m file we transform the point from the world frame to the
% actual frame and then apply it to the homegeneus matrix of the
% correspondent link in order to find the jacobian of a generic point in
% the link "link"


%     for i=1:link
%         A_numeric{i}=subs(A{i},{q1,q2,q3,q4,q5,q6,q7},{qq(1),qq(2),qq(3),qq(4),qq(5),qq(6),qq(7)});
%     end
% 
% 
%     for i=1:link
%        invA_numeric{i}=inv(A_numeric{i});
%     end
% 
%         if point == 0
%             
%                 point_inActualFrame =[0 0 1 1]';
%                 
%         else
%             point = [point ; 1];
%             point_inActualFrame=invA_numeric{link}*point;
%             
%             
%         end
 %PointEstimation = A{link}*point_inActualFrame;

 PointEstimation = A{link}*point;

    

    JJ = jacobian(PointEstimation,q);
    %Jsubs = subs(JJ,{q1,q2,q3,q4,q5,q6,q7},{Q1(t),Q2(t),Q3(t),Q4(t),Q5(t),Q6(t),Q7(t)});
    JJ=JJ(1:4,1:link);
    J(:,1:link) = subs(JJ,{q1,q2,q3,q4,q5,q6,q7},{qq(1),qq(2),qq(3),qq(4),qq(5),qq(6),qq(7)});
    
    J = J(1:3,1:7);
    %sizeJ = size(J);

end






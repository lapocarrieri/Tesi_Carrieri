clc
clear
close all

%% Symbolic variables
syms point % the point obtained through the residuals, it is px py pz
syms link % the link where the force is applied
point = [1;1;1;1];

syms q1 q2 q3 q4 q5 q6 q7 l1 l2 l3 l4

q = [q1;q2;q3;q4;q5;q6;q7];

%% Direct kinematics

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
C7=cos(0.0);
S7=sin(0.0);

% l1=0.0;     % for real robot where the base frame is on the second joint
% l2=0.4;
% l3=0.39;
% l4=0.078;   % EE in the tip of KUKA without auxiliary addition

M1 = [C1,0,S1,0; S1,0,-C1,0; 0,1,0,l1; 0,0,0,1];
M2 = [C2,0,-S2,0; S2,0,C2,0; 0,-1,0,0; 0,0,0,1];
M3 = [C3,0,-S3,0; S3,0,C3,0; 0,-1,0,l2; 0,0,0,1];
M4 = [C4,0,S4,0; S4,0,-C4,0; 0,1,0,0; 0,0,0,1];
M5 = [C5,0,S5,0; S5,0,-C5,0; 0,1,0,l3; 0,0,0,1];
M6 = [C6,0,-S6,0; S6,0,C6,0; 0,-1,0,0; 0,0,0,1];
M7 = [C7,-S7,0,0; S7,C7,0,0; 0,0,1,l4; 0,0,0,1];
% now define the 2 7D vectorscontaining the homogeneus transformation
% matrixes
A = cell(1, 7);
invA_numeric = cell(1, 7);

%through the premultiplication find the 7 transformation matrixes that
%transform the point in the actual frame to the point in the world frame
A{7} = (((((M1*M2)*M3)*M4)*M5)*M6)*M7;
A{6}= ((((M1*M2)*M3)*M4)*M5)*M6;
A{5} = (((M1*M2)*M3)*M4)*M5;
A{4} = ((M1*M2)*M3)*M4;
A{3} = (M1*M2)*M3;
A{2} = M1*M2;
A{1} = M1;
%% Point calculation
% now, given the position of the point application that is the result of
% the residual.m file we transform the point from the world frame to the
% actual frame and then apply it to the homegeneus matrix of the
% correspondent link in order to find the jacobian of a generic point in
% the link "link"
for i=1:7
    A_numeric{i}=subs(A{i},{q1,q2,q3,q4,q5,q6,q7},{pi/4,0,pi,pi/2,pi,0,0});
end
for i=1:7
   invA_numeric{i}=inv(A_numeric{i});
end

link=7;

point_inActualFrame=invA_numeric{link}*point;





PointEstimation = simplify(A{link}*point_inActualFrame)

J_intermediate = jacobian(PointEstimation,q)
J_interest = J(:,1:link)
size( J_interest)












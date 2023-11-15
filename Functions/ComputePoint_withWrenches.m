function [J] = ComputePoint_withWrenches(qq,link)
        %example of its use is: J=ComputePoint_withWrenches([2*pi 0 pi/3 pi/8 -pi/3 0 pi/4],2)
    syms q1 q2 q3 q4 q5 q6 q7

J = zeros(6,7);
JJw=zeros(3,link);
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
% now define the 2 7D vectors containing the homogeneus transformation
% matrixes

A{1}=M{1};
%through the premultiplication find the 7 transformation matrixes that
%transform the point in the actual frame to the point in the world frame
JJw(1:3,1)=[0 0 1]';

for j=2:link
    A{j}=A{j-1}*M{j};
end
for j=1:link-1
      JJw(1:3,j+1)=subs(A{j}(1:3,3),{q1,q2,q3,q4,q5,q6,q7},{qq(1),qq(2),qq(3),qq(4),qq(5),qq(6),qq(7)}); 
end
%JJ = simplify(jacobian(A{link}(1:3,4),q));

JJJ(1:3,1)=cross([0;0;1],A{link}(1:3,4));

for j=1:link-1
    JJJ(1:3,j+1)=cross(subs(A{j}(1:3,3),{q1,q2,q3,q4,q5,q6,q7},{qq(1),qq(2),qq(3),qq(4),qq(5),qq(6),qq(7)}),(A{link}(1:3,4)-A{j}(1:3,4))');
end
JJJ = vpa(subs(JJJ,{q1,q2,q3,q4,q5,q6,q7},{qq(1),qq(2),qq(3),qq(4),qq(5),qq(6),qq(7)}),3);
%JJ = vpa(subs(JJ,{q1,q2,q3,q4,q5,q6,q7},{qq(1),qq(2),qq(3),qq(4),qq(5),qq(6),qq(7)}),3)
J(:,1:link)=[JJJ;JJw];



end
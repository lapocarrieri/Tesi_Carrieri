function [X] = ComputePoint_fromTau(qq,tau,link)
        %example of its use is: ComputePoint_fromTau([2*pi 0 0 0 0 0 pi/4],[1,1,1,1,1,1,1],7)
syms q1 q2 q3 q4 q5 q6 q7
    syms t
    syms Q(t) Q1(t) Q2(t) Q3(t) Q4(t) Q5(t) Q6(t) Q7(t)

    syms p_x p_y p_z
    syms fx fy fz
q = [q1;q2;q3;q4;q5;q6;q7];
J = zeros(4,7);
Jsubs= zeros(4,7);
A = cell(1, link);
invA_numeric = cell(1, link);
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


EQN=[];
 point = [p_x p_y p_z 1]';
 PointEstimation = A{link}*point;
variables = [fx,fy,fz,p_x,p_y,p_z];
%variables = variables(3:6);


    JJ = jacobian(PointEstimation,q);
    JJ = subs(JJ,{q1,q2,q3,q4,q5,q6,q7},{qq(1),qq(2),qq(3),qq(4),qq(5),qq(6),qq(7)});

    
    JJ=JJ(1:4,1:link);
    
    tauUnknown=simplify(JJ'*[fx;fy;fz;1])

    
    for i=1:link
          eqn(i) = tauUnknown(i)==tau(i);
          EQN = [EQN,eqn(i)]

    end
  X=  solve(EQN, variables, 'Real', true);
  x = quadprog(H, f, A, b, Aeq, beq);
    return;
% Convert symbolic equations to function handles
eqn1 = matlabFunction(EQN(1));
eqn2 = matlabFunction(EQN(2));
eqn3 = matlabFunction(EQN(3));
eqn4 = matlabFunction(EQN(4));
eqn5 = matlabFunction(EQN(5));
eqn6 = matlabFunction(EQN(6));
eqn7 = matlabFunction(EQN(7));
% Create a function handle for all equations
eqn = @(x) [eqn1(x); eqn2(x); eqn3(x); eqn4(x); eqn5(x); eqn6(x); eqn7(x)]

% Initial guess for the variables
x0 = [1; 1; 1; 1; 1; 1]; % Adjust the initial guess as needed

% Solve the system using lsqnonlin
x = lsqnonlin(eqn, x0);
%    % [A,B] = equationsToMatrix(EQN, variables);
%    X = solve(EQN, variables)
% 
%    % X = vpa(simplify(pinv(A)*B),3);


end
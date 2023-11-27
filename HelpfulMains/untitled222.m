qq=qqs(end,:);

for jj=1:100
    link=4;
figure();

J_w = ComputePoint_withWrenches(Q_sampled(index,:),link);
F=[0.8236;0.1750;0.1636];
matrix=Meshes.Points(:,1:3,link);
point=matrix(jj*16,:)';
m=-skew_symmetric(F)*point;
gamma=(J_w'*[F;m])';
for i = 1:size(matrix,1)
POINTT=matrix(i,:)';


[Fm]=pinv(J_w')*gamma';

fval = (skew_symmetric(POINTT)*Fm(1:3)-Fm(4:6))'*(skew_symmetric(POINTT)*Fm(1:3)-Fm(4:6))

W(1,i) = exp(-2*fval);
diffVector = POINTT - point;
normDifferences(i) = norm(diffVector);
end
% Plot the results
plot( W,normDifferences, 'o');
xlabel('W');
ylabel('Norm of Differences');
title('Norm of Differences between Particles and W');
grid on;
end
for link=5:7

figure();


for i = 1:size(matrix,1)
POINTT=matrix(i,:)';
JJ = jacobian(A{link}*[POINTT;1],q);
                            %Jsubs = subs(JJ,{q1,q2,q3,q4,q5,q6,q7},{Q1(t),Q2(t),Q3(t),Q4(t),Q5(t),Q6(t),Q7(t)});
                            JJ=JJ(1:4,1:link);
                            J(:,1:link) = subs(JJ,{q1,q2,q3,q4,q5,q6,q7},{qq(1),qq(2),qq(3),qq(4),qq(5),qq(6),qq(7)});
                            
                            Jc = J(1:3,1:7);
                            %sizeJ = size(J);
     
    
                         Fc = (Sigma*Jc'*inv(Jc*Sigma*Jc'))'*gamma';%weighted pinv
                         fval = (gamma'-Jc'*Fc)'*(gamma'-Jc'*Fc);



W(1,i) = exp(-2*fval);
diffVector = POINTT - point;
normDifferences(i) = norm(diffVector);
end
% Plot the results
plot( W,normDifferences, 'o');
xlabel('W');
ylabel('Norm of Differences');
title('Norm of Differences between Particles and W');
grid on;
end


for link=5:7
figure();
J_w = ComputePoint_withWrenches(qqs,link);
F=[3;3;3];
matrix=Meshes.Points(:,1:3,link);
point=matrix(452,:)';
m=-skew_symmetric(F)*point;
gamma=(J_w'*[F;m]);
for i = 1:size(matrix,1)
POINTT=matrix(i,:)';

[POINTT,normal] = closest_point_to_triangle(triangles, POINTT');


mu = tan(deg2rad(30));


objectiveFunction = @(x) norm(gamma - J_w'*x)^2;


constraints = @(x) deal([], [norm(x(1:2)) - mu*dot(x(1:3), normal),-norm(x(1:3)) + 8,norm(x(1:3)) - 10]);

 options = optimoptions('fmincon', 'Algorithm', 'sqp');


x0 = pinv(J_w') * gamma; % Starting point

Fm = fmincon(objectiveFunction, x0, [],[],[], [],[], [], constraints, options)


fval = (skew_symmetric(POINTT)*Fm(1:3)-Fm(4:6))'*(skew_symmetric(POINTT)*Fm(1:3)-Fm(4:6));








W(1,i) = exp(-2*fval);
diffVector = POINTT - point;
normDifferences(i) = norm(diffVector);
end
% Plot the results
plot( W,normDifferences, 'o');
xlabel('W');
ylabel('Norm of Differences');
title('Norm of Differences between Particles and W');
grid on;
end



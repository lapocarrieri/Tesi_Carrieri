
for j=1:100
    figure();
J_w = ComputePoint_withWrenches(Q_sampled(j,:),link);
[Fm]=pinv(J_w')*Residual_calculated(j,:)';
for i = 1:size(matrix,1)
POINTT=matrix(i,1:3,link)';




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
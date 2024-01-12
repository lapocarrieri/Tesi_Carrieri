       figure(f4);


       hold off
        
        matrix=Meshes.Points(:,1:3,link);
        
        m=-skew_symmetric([12.700401450564340;0;-15.449912718022170])*point;
        gamma=(J_w'*[[12.700401450564340;0;-15.449912718022170];m])';
        for i = 1:size(matrix,1)
        POINTT=matrix(i,:)';
        
        
        [Fm]=pinv(J_w')*gamma';
        
        fval = (skew_symmetric(POINTT)*Fm(1:3)-Fm(4:6))'*(skew_symmetric(POINTT)*Fm(1:3)-Fm(4:6));
        
        W2(1,i) = exp(-2*fval);
        diffVector = POINTT - point;
        normDifferences(i) = norm(diffVector);
        end
        % Plot the results
   
        plot( W2,normDifferences, 'o');
        xlabel('W');
        ylabel('Norm of Differences');
        title('Norm of Differences between Particles and W');
        grid on;

        fval = (skew_symmetric(point)*Fm(1:3)-Fm(4:6))'*(skew_symmetric(point)*Fm(1:3)-Fm(4:6));
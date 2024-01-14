       
         
        for k=1:30
       figure;
         scatter3(matrix(:,1),matrix(:,2),matrix(:,3),'y')
         hold on
        F_applied = 10*rand(1, 3)';
        fsampled(k,:)=F_applied;
        matrix=Meshes.Points(:,1:3,link);
        
        m=-skew_symmetric([F_applied])*point;
        gamma=(J_w'*[[F_applied];m])';
        for i = 1:size(matrix,1)
        POINTT=matrix(i,:)';
        
        
        [Fm]=pinv(J_w')*gamma';
        
        fval = (skew_symmetric(POINTT)*Fm(1:3)-Fm(4:6))'*(skew_symmetric(POINTT)*Fm(1:3)-Fm(4:6));
        
        W2(1,i) = exp(-2*fval);
        diffVector = POINTT - point;
        normDifferences(i) = norm(diffVector);
        
        end
        % Plot the results
         
        %plot( W2,normDifferences, 'o');
        
        vector = [W2]; % Replace with your actual vector
        [~, indices] = sort(vector, 'descend'); % Sorts the vector in descending order and gets indices
        top_20_indices = indices(1:100); % Selects the indices of the first 20 elements
        for j=1:20
        scatter3( matrix(j,1), matrix(j,2),matrix(j,3),'r')
        end
        scatter3(point(1),point(2),point(3),'r')

        end

        

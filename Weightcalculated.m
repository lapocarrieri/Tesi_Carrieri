%                it=1;
%                mu=0.01;
%                normerror3=zeros(20000-1,1);
%                wrenches=zeros(20000-1,6);
%                 while it<20000
%                    W = diag(rand(6, 1));
%                    J_withwrenchesweightedPseudoinverse=inv(W)*J_withwrenches*inv(J_withwrenches'*inv(W)*J_withwrenches+mu^2*eye(7));
%                    
%                    wrenches3=J_withwrenchesweightedPseudoinverse*r;
%                    wrenches(it,:)=wrenches3;
%                    error3 = [ExternalForceAppliedActualFrame;m]-wrenches3;
%                    normerror3(it)=norm(error3);
%                    it=it+1;
%                 end
%                 [minValue, minIndex] = min(normerror3);
%                 bestwrenches(index,:)=wrenches(minIndex,:);
%                 % Display the results
%                 fprintf('Minimum value: %f\n', minValue);
%                 fprintf('Position of the minimum: %d\n', minIndex);
                if link==1
                    W=diag([0.01 ,        0.01    ,  0.01   ,  0.01    ,  0.01 , -0.0526]);
                end
                if link==2
                    W=diag([0.01     ,   0.01  ,   0.01  ,-0.0330   ,-0.0000  , -0.0837]);
                end
                if link==3
                    W=diag([   -1.2556   ,-0.0890  ,  4.1134 ,   0.1823 ,  -0.1856  ,  0.0511]);
                end
                if link==4
                    W=diag([ 0.7614  , 0.0245  , -2.8593 ,  -0.0731  ,  0.0678  ,  0.0304]);
                end
                if link==5
                    W=diag( [1.5219  ,  6.3337  ,  4.0377,   -0.1557 ,  -0.0451  ,  0.1272]);
                end
                if link==6
                    W=diag([-0.5817 ,  -0.2504  ,  3.2473 ,   0.1427,   -0.1285 ,   0.0134]);
                end
                if link==7
                    W=diag([0.7671  ,  2.9953 ,   1.3766,   -0.1098,    0.0056   , 0.0548]);
                end
                W_calculated=W;

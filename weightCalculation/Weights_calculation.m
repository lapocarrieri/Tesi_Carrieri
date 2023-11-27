                    
                clear all
                load('PseudoinverseNN')
              
                W=eye(6);
                    index=1;
                    for indecs=11:size(Residual_calculated,1)
                
                        norms=[];
                        
                        for k=1:1000
                            
                            for i=1:6
                                W(i,i)=rand();
                            end
                            
                            J = cell2mat(Js(indecs));

                           J_withwrenchesweightedPseudoinverse=W^(-1/2)*pinv(J'*W^(-1/2));
                           wrenches3=J_withwrenchesweightedPseudoinverse*Residual_calculated(indecs,:)';
                            error3 = moments(indecs)-wrenches3;
                            norms(k)=norm(error3);
                            weights{k}=W;
                            norm(error3);
                          
                        end
                        size(norms);
                        [~,sorted_indices]=sort(norms);
                        for p=1:10  
                                vector(10*indecs+p,:) = diag(cell2mat(weights(sorted_indices(p))));
                        end
                    end
            Finalweight=vecnorm(vector,2,1)
               
        S_fext =skew_symmetric(F_applied);
        m=double(-S_fext*point(1:3));
        TauExternalForce=(J_w'*[F_applied;m])';

        Fm_unconstrained = pinv(J_w') * gamma';
        mu=0.1;
        % Objective function
        objFunc = @(Fm) norm(Fm - Fm_unconstrained)^2;
        
        
        
        % Initial guess (can be the unconstrained solution or another appropriate starting point)
        initial_guess = Fm_unconstrained;
        
        % Optimization options, such as algorithm selection, can be set here
        % Solver options with relaxed tolerances
        options = optimoptions('fmincon', 'Algorithm', 'sqp', ...
                               'MaxFunctionEvaluations', 1e4, ...
                               'StepTolerance', 1e-8, ...
                               'OptimalityTolerance', 1e-4, ... % Relaxed optimality tolerance
                               'ConstraintTolerance', 1e-4,'Display', 'off');   % Relaxed constraint tolerance


       hold off
        
        matrix=Meshes.Points(:,1:3,link);
        
        m=-skew_symmetric([1.2246;   -1.8115;    8.7318])*point;
        gamma=(J_w'*[[1.2246;   -1.8115;    8.7318];m])';
        for i = 1:size(matrix,1)
        POINTT=matrix(i,:)';
        desired_Fm = [1.2246;   -1.8115;    8.7318];

% Tolerance for each component of Fm(1:3)
tolerance = [0.5; 0.5; 0.5]; % Adjust this as needed

% Non-linear constraints
nonlcon = @(Fm) deal( ...
    norm(Fm(1:3) - dot(Fm(1:3), normal_vector) * normal_vector) - mu * abs(dot(Fm(1:3), normal_vector)), ...
    max(abs(Fm(1:3) - desired_Fm) - tolerance, 0) ...
);
                    % Attach the custom output function to the options
                options.OutputFcn = @customOutputFunction;
                
                % Solve the optimization problem
                [Fm, ~, ~, ~, ~, best_solution] = fmincon(objFunc, initial_guess, [], [], [], [], [], [], nonlcon, options);

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
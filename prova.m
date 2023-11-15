M = [5, 5, 5];  % Maximum distance vector
m = [-5,-5,-5];  % Minimum distance vector

for i=1:10000
    random_vector = generate_random_3d_vector(M, m);
    scatter3(random_vector(1),random_vector(2),random_vector(3));
    hold on
end

disp("Random 3D Vector:");
disp(random_vector);
function random_vector = generate_random_3d_vector(M, m)
    mean = [0, 0, 0];
    cov = diag([0.1, 0.1, 0.1]);  % Identity covariance matrix

    while true
        vector = mvnrnd(mean, cov);
        if all(vector < M) && all(vector > m)
            random_vector = vector;
            return
        end
    end
end
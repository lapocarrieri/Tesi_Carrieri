function random_vector = generate_random_3d_vector(M, m,sigma)
    mean = [0, 0, 0];
    cov = diag([sigma, sigma,sigma]);  % Identity covariance matrix

    while true
        vector = mvnrnd(mean, cov);
        if all(vector < M) && all(vector > m)
            random_vector = vector;
            return
        end
    end
end
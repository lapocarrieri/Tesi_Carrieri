function CalculatedPoint2 = calculatePoint(chi2)
    % Parameters for outlier detection
    num_std_dev = 2; % Number of standard deviations for outlier threshold

    % Calculate the mean and standard deviation
    mean_points = mean(chi2, 2);
    std_dev = std(chi2, 0, 2);

    % Find outliers (points beyond num_std_dev standard deviations from the mean)
    outlier_mask = any(abs(chi2 - mean_points) > num_std_dev * std_dev, 1);

    % Exclude outliers
    filtered_points = chi2(:, ~outlier_mask);

    % Calculate CalculatedPoint2 (mean of the remaining points)
    CalculatedPoint2 = mean(filtered_points, 2);
end

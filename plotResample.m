% Assume chi2(:,1) are the x-coordinates of the particles
% W are the weights of the particles

% Normalize weights for visualization
normalized_weights = W / max(W);

% Plot each particle as a vertical line
for i = 1:length(W)
    line([chi2(i,1), chi2(i,1)], [0, normalized_weights(i)], 'Color', 'cyan');
    hold on; % Keep the plot active to overlay more data
end

% Fit a Gaussian distribution to the x-coordinates of the particles
pd = fitdist(chi2(:,1), 'Normal');

% Generate values for the PDF and CDF
x_values = linspace(min(chi2(:,1)), max(chi2(:,1)), 1000);
pdf_values = pdf(pd, x_values);
cdf_values = cdf(pd, x_values);

% Overlay the PDF
plot(x_values, pdf_values, 'b-', 'LineWidth', 2);

% Overlay the CDF on a second y-axis
yyaxis right;
plot(x_values, cdf_values, 'r-', 'LineWidth', 2);

% Label the axes
xlabel('x');
yyaxis left; % Switch back to left y-axis for the PDF
ylabel('Probability Density');
yyaxis right; % Switch to right y-axis for the CDF
ylabel('Cumulative Distribution');

% Add a title and grid
title('Particle Filter Distribution');
grid on;

% Finish up the plot
hold off;

% Assuming Residual_calculated, TauExtForce, and LinkInCollision are already defined
% n is the number of samples
n = size(Residual_calculated, 1);

% Time vector
time = linspace(0, (n-1) * 0.01, n-29);

% Creating a new figure
figure;

% Plotting each column of Residual_calculated in a unique color
hold on; % Holds the current plot so that new plots do not delete existing ones
colors = lines(7); % Generate 7 distinct colors
for i = 1:7
    plot(time, Residual_calculated(30:index-1, i), 'Color', colors(i,:));
end

hold on
% Plotting each column of TauExtForce with dashed lines
for i = 1:7
    plot(time, TauExtForce(30:index-1, i), '--', 'Color', colors(i,:));
end

% Plotting the LinkInCollision as a black line
plot(time, link_collided(30:index-1), 'k-', 'LineWidth', 0.4);
yline(linkforce, 'r-', 'LineWidth', 0.2);
% Adding labels and a legend
xlabel('Time (seconds)');
ylabel('Values');
title('Residual_calculated and TauExtForce Curves with LinkInCollision Line');

grid on;

hold off; % Release the hold on the current figure

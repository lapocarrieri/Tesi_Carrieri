% Define original 3D points
x = [1, 2, 3, 4];  % x coordinates
y = [1, 2, 3, 4];  % y coordinates
z = [0.2, 0.5, 1.0, 0.8];  % z coordinates

% Define higher resolution grid
[X, Y] = meshgrid(1:0.1:4, 1:0.1:4);  % increase resolution by 10x

% Interpolate surface
Z = griddata(x, y, z, X, Y);

% Plot original and interpolated surfaces
figure;
subplot(1, 2, 1);
scatter3(x, y, z, 'filled');
title('Original Surface');
xlabel('x');
ylabel('y');
zlabel('z');
size(X)
size(Y)
size(Z)

subplot(1, 2, 2);
surf(X, Y, Z);
title('Interpolated Surface');
xlabel('x');
ylabel('y');
zlabel('z');

% Assuming 'triangles' is your 3x3x1098 array

figure;
hold on;
axis equal;
grid on;
xlabel('X');
ylabel('Y');
zlabel('Z');

for i = 1:size(triangles, 3)
    % Extract the vertices of the ith triangle
    vertices = T(1:3,1:3)*triangles(:, :, i)+T(1:3,4);

    % Plot the triangle
    patch(vertices(1, :), vertices(2, :), vertices(3, :), 'b'); % Color set to blue
end

hold off;
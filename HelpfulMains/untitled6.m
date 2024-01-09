figure;
hold on;
grid on;
axis equal;

% Iterate through each triangle and plot
for i = 1:size(triangles, 3)
    % Extract the i-th triangle
    triangle = triangles(:, :, i);
    
    % Plot each edge of the triangle
    % Adding the first point again at the end to close the triangle
    plot3([triangle(1, 1), triangle(1, 2)], ...
          [triangle(2, 1), triangle(2, 2)], ...
          [triangle(3, 1), triangle(3, 2)], 'b');
    plot3([triangle(1, 2), triangle(1, 3)], ...
          [triangle(2, 2), triangle(2, 3)], ...
          [triangle(3, 2), triangle(3, 3)], 'b');
    plot3([triangle(1, 3), triangle(1, 1)], ...
          [triangle(2, 3), triangle(2, 1)], ...
          [triangle(3, 3), triangle(3, 1)], 'b');
end
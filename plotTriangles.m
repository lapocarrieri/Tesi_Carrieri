figure;
% Assuming Meshes.Points and MeshesConnectivityList are available
% Replace with your actual data structures
% For demonstration, these are placeholders

% Iterate over each link
for link = 1:7
    subplot(3, 3, link);  % Adjust the subplot grid if necessary
    hold on;
    grid on;
    axis equal;

    % Extract triangles for the current link
    triangles = [];
    for i = 1:size(MeshesConnectivityList{link}, 1)
        for j = 1:3
            triangles(:, j, i) = Meshes.Points(MeshesConnectivityList{link}(i, j), :, link)';
        end
    end

    % Iterate through each triangle and plot
    for i = 1:size(triangles, 3)
        % Extract the i-th triangle
        triangle = triangles(:, :, i);
        
        % Plot each edge of the triangle
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

    % Set titles and axes labels if needed
    title(sprintf('Link %d', link));
    xlabel('X-axis');
    ylabel('Y-axis');
    zlabel('Z-axis');
    view(3); % Set the view to 3D
end

% Adjust spacing between subplots
sgtitle('3D Mesh Plots for Links 1 to 7');
% Assuming you have a matrix named 'triangles' of size 3x3x1098
link=5;
        triangles = [];
        for i = 1:size(MeshesConnectivityList{link}, 1)
            for j = 1:3
                triangles(:, j, i) = Meshes.Points(MeshesConnectivityList{link}(i, j), :, link)';
            end
        end
% Create a figure and an axes for 3D plotting
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

% Label the axes


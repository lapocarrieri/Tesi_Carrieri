figure();
x = matrix(:, 1,link);
    y = matrix(:, 2,link);
    z = matrix(:, 3,link);

    % Plot the data
    
    plot3(x, y, z, 'b.');
hold on

for i = 1:size(triangles, 3)
    % Extract the vertices of the ith triangle
    for j=1:3
        vertices = triangles(:, :, i);
        pointt=vertices(:,j);

    % Plot the triangle
    scatter3(pointt(1), pointt(2), pointt(3), 'r'); % Color set to blue
    hold on
end 
end
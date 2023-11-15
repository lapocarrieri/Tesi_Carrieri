    ranges = [2*pi/100, 2*pi/66, 2*pi/73, 2*pi/121, 2*pi/85, 2*pi/105, 2*pi/94];
    q0=[0 0 0 0 0 0 0];
    DeltaT=0.01;
       addpath 'Dynamics'

   addpath 'Functions'
    % Add random numbers to each element
    for a=1:10000
        
        for i = 1:7
            q0(i) = q0(i) + rand() * ranges(i)*DeltaT*10;
        end
        p_ref(:,a) = f(q0(1),q0(2),q0(3),q0(4),q0(5),q0(6));
    end
    % Example data: 3xN matrix where each column represents a 3D vector


n = size(p_ref, 2);  % Number of vectors

% Plotting the vectors
figure;
plot3(0.1,0.1,0.1)
hold on;
grid on;

for i = 1:n
    vector = p_ref(:, i);
    quiver3(0, 0, 0, vector(1), vector(2), vector(3), 'LineWidth', 1.5);
end

axis equal;
xlabel('X');
ylabel('Y');
zlabel('Z');
title('3D Vectors');
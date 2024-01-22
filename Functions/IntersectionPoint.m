function [Point_intersected] = IntersectionPoint(line,link,PreviousPoint,matrix,T)
        addpath 'Functions'
        figure();
        t = linspace(-0.2,0.2, 200);
        x = line.origin(1) + line.direction(1)*t;
        y = line.origin(2) + line.direction(2)*t;
        z = line.origin(3) + line.direction(3)*t;
       
        % Plot the 3D line
        view(135, 69);
        plot3(x, y, z, 'b', 'LineWidth', 0.2);  
        hold on
       plot3(line.origin(1),line.origin(2),line.origin(3), 'o', 'MarkerSize', 2, 'MarkerFaceColor', 'g', 'LineWidth', 2);



        plotRF(T(1:3,1:3), T(1:3,4));
        Vertices.Points=double((T*matrix.Points(:,:,link)')');

        Vertices.ConnectivityList=matrix.ConnectivityList{link};

                       
                       
                    
                       
                       x=Vertices.Points(:,1);
                        y=Vertices.Points(:,2);
                         z=Vertices.Points(:,3);
                    mask = z > 0.1;
                    filtered_x = x(mask);
                    filtered_y = y(mask);
                    filtered_z = z(mask);
                    
                    % Plotting only the points where z > 0.1
                    plot3(filtered_x, filtered_y, filtered_z, 'g.', 'LineWidth', 0.05);
                    title('3D Plot');
                    xlabel('X');
                    ylabel('Y');
                    zlabel('Z');
                    axis equal;
                    grid on;
                    
                    Vertices.points=matrix.Points(1:3,:);

            Point_intersected = checkIntersection(Vertices, line,PreviousPoint);
            if isempty(Point_intersected) 
                plot3(line.origin(1),line.origin(2),line.origin(3), 'o', 'MarkerSize', 2, 'MarkerFaceColor', 'g', 'LineWidth', 2);

            elseif Point_intersected==[0 0 0]
                plot3(line.origin(1),line.origin(2),line.origin(3), 'o', 'MarkerSize', 2, 'MarkerFaceColor', 'g', 'LineWidth', 2);

            else
                plot3(Point_intersected(1),Point_intersected(2),Point_intersected(3), 'o', 'MarkerSize', 3, 'MarkerFaceColor', 'b', 'LineWidth', 4);

            end
          

end
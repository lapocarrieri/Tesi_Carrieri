function [Point_intersected] = IntersectionPoint(line,link,PreviousPoint,matrix,f3,T)
        addpath 'Functions'
        
        t = linspace(-0.2,0.2, 200);
        x = line.origin(1) + line.direction(1)*t;
        y = line.origin(2) + line.direction(2)*t;
        z = line.origin(3) + line.direction(3)*t;
       
        % Plot the 3D line
        figure(f3),plot3(x, y, z, 'b', 'LineWidth', 0.2);  
        hold on

       %figure(f3),plot3(line.origin(1),line.origin(2),line.origin(3), 'o', 'MarkerSize', 2, 'MarkerFaceColor', 'g', 'LineWidth', 2);
              
        
        
        figure(f3),plotRF(T(1:3,1:3), T(1:3,4));
        Vertices.Points=double((T*matrix.Points(:,:,link)')');

        Vertices.ConnectivityList=matrix.ConnectivityList;

                       
                       
                    
                       
                       x=Vertices.Points(:,1);
                        y=Vertices.Points(:,2);
                         z=Vertices.Points(:,3);
                %        figure(f3),plot3(x, y, z, 'g.','LineWidth', 0.2);
                    title('3D Plot');
                    xlabel('X');
                    ylabel('Y');
                    zlabel('Z');
                    axis equal;
                    grid on;
                    hold off
                    Vertices.points=matrix.Points(1:3,:);

            Point_intersected = checkIntersection(Vertices, line,PreviousPoint);
            
   

end
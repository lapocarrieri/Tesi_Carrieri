function [outputArg1,outputArg2] = LineCylinderFunction(inputArg1,inputArg2)
    % Input parameters
x0 = 0;         % x-coordinate of the point on the line
y0 = 0;         % y-coordinate of the point on the line
z0 = 0;         % z-coordinate of the point on the line
direction = [1, 1, 1];   % 3D angular coefficient (direction) of the line
r = 1;          % radius of the cylinder
h = 5;          % height of the cylinder

% Normalize the direction vector
direction = direction / norm(direction);

% Calculate coefficients of the quadratic equation
a = direction(1)^2 + direction(2)^2;
b = 2 * (direction(1) * x0 + direction(2) * y0);
c = x0^2 + y0^2 - r^2;

% Solve the quadratic equation
delta = b^2 - 4 * a * c;
if delta < 0
    disp('No intersection points')
else
    t1 = (-b + sqrt(delta)) / (2 * a);
    t2 = (-b - sqrt(delta)) / (2 * a);

    % Calculate the intersection points
    X1 = x0 + direction(1) * t1;
    Y1 = y0 + direction(2) * t1;
    Z1 = z0 + direction(3) * t1;

    X2 = x0 + direction(1) * t2;
    Y2 = y0 + direction(2) * t2;
    Z2 = z0 + direction(3) * t2;

    % Check if the intersection points are within the cylinder height
    if Z1 >= 0 && Z1 <= h
        disp('Intersection Point 1:')
        disp([X1, Y1, Z1])
    else
        disp('Intersection Point 1 is outside the cylinder height')
    end

    if Z2 >= 0 && Z2 <= h
        disp('Intersection Point 2:')
        disp([X2, Y2, Z2])
    else
        disp('Intersection Point 2 is outside the cylinder height')
    end
end

end
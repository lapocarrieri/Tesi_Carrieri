% Define the force vector (example)
F = (T * force_vector_4d)/100; % Replace with your force vector
F=F(1:3);
% Define the Point where the apex of the cone will be (example)
Point = pointWorld; % Replace with your point
Point=Point(1:3);
% Define the maximum angle of inclination in degrees
max_angle = 60; % Replace with your maximum angle

% Convert the maximum angle to radians for calculations
max_angle_rad = deg2rad(max_angle);

% Define the number of points for plotting
num_points = 50;

% Determine the length of the force vector
force_length = norm(F);

% Calculate the radius of the cone's base
base_radius = force_length * tan(max_angle_rad);

% Generate theta and z values for the cone surface
[theta, z] = meshgrid(linspace(0, 2*pi, num_points), linspace(0, force_length, num_points));

% Cartesian coordinates for the cone surface
x = base_radius * (1 - z / force_length) .* cos(theta);
y = base_radius * (1 - z / force_length) .* sin(theta);
z = -z; % Invert the cone

% Rotate the cone to align with the force vector
% Normalize the force vector
force_unit = F / force_length;

% Compute rotation axis and angle
axis_rot = cross([0 0 1], force_unit);
if norm(axis_rot) ~= 0
    axis_rot = axis_rot / norm(axis_rot);
    angle_rot = acos(dot(force_unit, [0 0 1]));
    rot_matrix = axang2rotm([axis_rot angle_rot]);

    % Apply rotation to the cone coordinates
    for i = 1:numel(x)
        v = [x(i); y(i); z(i)];
        v_rot = rot_matrix * v;
        x(i) = v_rot(1);
        y(i) = v_rot(2);
        z(i) = v_rot(3);
    end
end

% Translate the cone so that its apex is at Point
x = x + Point(1);
y = y + Point(2);
z = z + Point(3);

% Plot the cone

surf(x, y, z);
hold on;

% Plot the force vector (origin to Point)



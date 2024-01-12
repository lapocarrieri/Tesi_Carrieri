friction_coefficient=0.01;
angle_of_cone = atan(friction_coefficient);

% Generate a random force vector within the friction cone
% This is a simple approach for demonstration purposes
theta = rand() * 2 * pi; % Random angle in the plane perpendicular to the normal
phi = rand() * angle_of_cone; % Random angle within the friction cone
force_magnitude = 5; % Random force magnitude

% Convert spherical coordinates to Cartesian coordinates
ExternalForceAppliedActualFrame = -force_magnitude *normal_vector';% [sin(phi) * cos(theta); sin(phi) * sin(theta); cos(phi)];

% Adjust the force vector to align with the friction cone
% Rotate the force vector to align its 'up' direction with the normal vector
normal_vector=normal_vector';

% Transform force_vector_alignedthe force vector and point of application to the world frame
T=QtoP(q0(1:7),link);
force_vector_world = T(1:3, 1:3) * ExternalForceAppliedActualFrame*0.01; % Apply only the rotation
application_point_world = T * [point; 1]; % Apply full transformation

arrow_start_point_world = application_point_world(1:3) - force_vector_world;

% Plot the force vector as an arrow in 3D in the world frame

quiver3(arrow_start_point_world(1), arrow_start_point_world(2), arrow_start_point_world(3), ...
        force_vector_world(1), force_vector_world(2), force_vector_world(3), ...
        'MaxHeadSize', 0.5, 'Color', 'r');
scatter3(application_point_world(1),application_point_world(2),application_point_world(3),'r','filled')
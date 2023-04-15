clear all
clc

%% Define robot parameters
% Number of links
n_links = 6;

% Link lengths
L = [1, 1, 1, 1, 1, 1];

% Joint angles (in radians)
q = [0, 0, 0, 0, 0, 0];

% End-effector position (in meters)
Pc = [1.5; 0.5; 1];

% Radius of the cylindrical link (in meters)
r = 0.2;

% Rotation matrix from the world frame to the cylinder frame
theta = pi/4;
Rc = [cos(theta) 0 sin(theta);
      0 1 0;
      -sin(theta) 0 cos(theta)];

%% Compute the Jacobian
% Initialize the Jacobian matrix
J = zeros(6, n_links);

% Loop over the links and compute the Jacobian for each one
for i = 1:n_links
    % Compute the transformation matrix from the world frame to link i
    T = eye(4);
    for j = 1:i-1
        T = T * DH(L(j), q(j));
    end
    
    % Compute the position and orientation of the end-effector in link i's frame
    Pe_i = T\[Pc; 1];
    Re_i = T(1:3, 1:3)\Rc;
    
    % Compute the Jacobian for link i
    J(:, i) = [cross(Re_i(:, 1), Pe_i(1:3));
               cross(Re_i(:, 2), Pe_i(1:3));
               cross(Re_i(:, 3), Pe_i(1:3));
               Re_i(:, 1);
               Re_i(:, 2);
               Re_i(:, 3)];
end

%% Compute the pseudoinverse of the Jacobian
J_pinv = pinv(J);

%% Compute the external wrench acting on the end-effector
F_x = 0; % Force in the x direction
F_y = 0; % Force in the y direction
F_z = 10; % Force in the z direction
M_x = 0; % Torque around the x axis
M_y = 0; % Torque around the y axis
M_z = 0; % Torque around the z axis
wrench = [F_x; F_y; F_z; M_x; M_y; M_z];
p = 4; % Index of the link we're interested in
J_pinv_p = J_pinv((p-1)*6+1:p*6, :); % Extract the relevant rows of the pseudoinverse
ext_wrench = J_pinv_p * wrench;

%% Compute the line described by the lambda parameter
lambda = 2;
line = Pc + lambda*wrench(1:3)/norm(wrench(1:3));

%% Compute the intersection of the line with the cylinder
% Transform the line to the cylinder frame
line_c = inv(Rc) * (line - Pc);

% Solve for the two possible values of lambda
a = line_c(1)^2 + line_c(2)^2;
b = 2*line_c(1)*line_c(3);
c = line_c(3)^2 - r^2;
delta = b^2 - 4*a*c;
if delta >= 0
    lambda1 = (-b + sqrt(delta)) / (2*a);
    lambda2 = (-b - sqrt(delta)) / (2*a);
    % Compute the two intersection
end
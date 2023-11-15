

% Define the robot's parameters and state
L1 = 1; % Length of link 1
L2 = 1; % Length of link 2
L3 = 1; % Length of link 3
q1 = pi/4; % Joint angle 1
q2 = pi/3; % Joint angle 2
q3 = pi/6; % Joint angle 3
%This block of code defines the robot's parameters and state. The lengths of the three links are stored in L1, L2, and L3, and the joint angles are stored in q1, q2, and q3.
% Define the desired end-effector pose
x_d = 1; % Desired x position
y_d = 2; % Desired y position
z_d = 0; % Desired z position
theta_x = pi/6; % Desired orientation around x axis
theta_y = pi/4; % Desired orientation around y axis
theta_z = pi/3; % Desired orientation around z axis
%This block of code defines the desired end-effector pose. The desired position is stored in x_d, y_d, and z_d, and the desired orientation is stored in theta_x, theta_y, and theta_z.
% Define the cylinder parameters for the p-th link

p = 3; % Index of the link of interest
r = 0.1; % Radius of the cylinder
L = L2; % Length of the cylinder
    % This function computes the end effector position of a planar robot given its joint angles and link lengths
    % Compute the coordinates of the first joint
    x1 = 0;
    y1 = 0;
    
    % Compute the coordinates of the second joint
    x2 = L1*cos(q1);
    y2 = L1*sin(q1);
    
    % Compute the coordinates of the third joint
    x3 = x2 + L2*cos(q1+q2);
    y3 = y2 + L2*sin(q1+q2);
    
    % Compute the coordinates of the end effector
    x = x3 + L3*cos(q1+q2+q3);
    y = y3 + L3*sin(q1+q2+q3);
 Pc=[x,y]% Center of the cylinder
        % This function computes the rotation matrix of the end effector of a planar robot given its joint angles
    
    % Compute the rotation matrix of the first joint
    R1 = [cos(q1) -sin(q1); sin(q1) cos(q1)];
    
    % Compute the rotation matrix of the second joint
    R2 = [cos(q1+q2) -sin(q1+q2); sin(q1+q2) cos(q1+q2)];
    
    % Compute the rotation matrix of the third joint
    R3 = [cos(q1+q2+q3) -sin(q1+q2+q3); sin(q1+q2+q3) cos(q1+q2+q3)];
    
    % Compute the rotation matrix of the end effector
Rc = R1 * R2 * R3;% Rotation matrix of the cylinder
%This block of code defines the parameters for the p-th link cylinder. The index of the link of interest is stored in p, the radius of the cylinder is stored in r, and the length of the cylinder is stored in L. The center of the cylinder is computed using the function computeEndEffectorPosition(), which takes the joint angles and link lengths as inputs and returns the position of the end-effector. The rotation matrix of the cylinder is computed using the function computeEndEffectorRotation(), which takes the joint angles as inputs and returns the rotation matrix of the end-effector.
% Compute the robot's Jacobian
% Compute the Jacobian matrix J for a 3-link planar manipulator
% with joint angles q1, q2, and q3 and link lengths L1, L2, and L3.

% Forward kinematics
x = L1*cos(q1) + L2*cos(q1+q2) + L3*cos(q1+q2+q3);
y = L1*sin(q1) + L2*sin(q1+q2) + L3*sin(q1+q2+q3);

% Partial derivatives
dx_dq1 = -L1*sin(q1) - L2*sin(q1+q2) - L3*sin(q1+q2+q3);
dx_dq2 = -L2*sin(q1+q2) - L3*sin(q1+q2+q3);
dx_dq3 = -L3*sin(q1+q2+q3);
dy_dq1 = L1*cos(q1) + L2*cos(q1+q2) + L3*cos(q1+q2+q3);
dy_dq2 = L2*cos(q1+q2) + L3*cos(q1+q2+q3);
dy_dq3 = L3*cos(q1+q2+q3);

% Jacobian matrix
J = [dx_dq1, dx_dq2, dx_dq3; dy_dq1, dy_dq2, dy_dq3];
%This block of code computes the robot's Jacobian matrix using the function computeJacobian(), which takes the joint angles and link lengths as inputs and returns the Jacobian matrix.
% Compute the pseudoinverse of the transpose of the Jacobian
J_pinv = pinv(J');
% Compute the external wrench acting on the end-effector
%This block of code computes the external wrench acting on the end-effector of the robot, multiplies it by the pseudoinverse of the transpose of the Jacobian for the p-th link, and computes a line described by the lambda parameter. It then computes the intersection of this line with the cylinder that represents the p-th link of the robot.

F_x = 10; % Force in the x direction
F_y = 0; % Force in the y direction

M_x = 0; % Torque around the x axis
M_y = 0; % Torque around the y axis

wrench = [F_x; F_y; M_x; M_y];
%This code sets up the external wrench acting on the end-effector of the robot. In this case, there is only a force acting in the positive z direction.
J_pinv
J_pinv_p = J_pinv((p-1)*6+1:p*6, :); % Extract the relevant rows of the pseudoinverse
ext_wrench = J_pinv_p * wrench;
%This code extracts the relevant rows of the pseudoinverse of the transpose of the Jacobian for the p-th link of the robot, multiplies it by the external wrench, and stores the result in ext_wrench.

% Compute the line described by the lambda parameter

line = Pc + lambda*wrench(1:3)/norm(wrench(1:3));

% Compute the intersection of the line with the cylinder
% Transform the line to the cylinder frame
%This code sets up the line described by the lambda parameter. In this case, lambda is set to 2, and the direction of the line is the normalized force vector in the x, y, and z directions stored in wrench.

line_c = inv(Rc) * (line - Pc);
% Solve for the two possible values of lambda
a = line_c(1)^2 + line_c(2)^2;
b = 2*line_c(1)*line_c(3);
c = line_c(3)^2 - r^2;
delta = b^2 - 4*a*c;
if delta >= 0
    lambda1 = (-b + sqrt(delta)) / (2*a);
    lambda2 = (-b - sqrt(delta)) / (2*a);
    % Compute the two intersection points in the cylinder frame
    int1_c = [line_c(1) * lambda1; line_c(2) * lambda1; lambda1];
    int2_c = [line_c(1) * lambda2; line_c(2) * lambda2; lambda2];
end
    % Transform the intersection points back to the world frame
%This code computes the intersection of the line with the cylinder that represents the p-th link of the robot. It first transforms the line from the world frame to the frame of the cylinder using the inverse of the rotation matrix Rc and the center of the cylinder Pc. It then computes the two possible values of the lambda parameter using the quadratic formula, and computes the corresponding intersection points in the cylinder frame. Finally, it transforms the intersection points back to the world frame.

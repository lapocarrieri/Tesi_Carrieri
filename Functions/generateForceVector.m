function ExternalForceAppliedActualFrame = generateForceVector(normal, alpha, norm_force)
    % Normalize the normal vector
    normal = normal / norm(normal);

    % Find a vector perpendicular to the normal
    % For example, if normal is not parallel to [1; 0; 0], we can use it to find a perpendicular vector
    if abs(dot(normal, [1; 0; 0])) < 1
        perpVector = cross(normal, [1; 0; 0]);
    else
        perpVector = cross(normal, [0; 1; 0]);
    end

    % Normalize the perpendicular vector
    perpVector = perpVector / norm(perpVector);

    % Rotate the perpendicular vector by alpha degrees around the normal
    % Create a rotation matrix using the Rodrigues rotation formula
    alpha_rad = deg2rad(alpha); % Convert alpha to radians
    K = [0, -normal(3), normal(2); normal(3), 0, -normal(1); -normal(2), normal(1), 0]; % Skew-symmetric matrix
    R = eye(3) + sin(alpha_rad) * K + (1 - cos(alpha_rad)) * (K^2); % Rotation matrix

    % Apply the rotation
    rotatedVector = R * perpVector;

    % Scale the rotated vector to have the specified norm
    ExternalForceAppliedActualFrame = rotatedVector * norm_force;
end

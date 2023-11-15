
function S = skew_symmetric(v)
    % This function takes a 3D vector 'v' and returns the skew-symmetric matrix 'S' associated with it.

    % Validate input is a 3D vector
    if length(v) ~= 3
        error('Input must be a 3D vector');
    end

    % Define the skew-symmetric matrix S for vector v
    S = [  0     -v(3)   v(2);
          v(3)    0     -v(1);
         -v(2)   v(1)    0  ];
end
displacement = Point_intersected - initial_position;

distance = norm(displacement);
F_max = 100;
F_min=0;
if distance > 0
    direction_of_displacement = displacement / distance;
else
    direction_of_displacement = [0; 0; 0];
end
dot_product = dot(initial_force/norm(initial_force), direction_of_displacement);

F_applied = initial_force * (1 - dot_product * distance);
F_applied = max(F_min, min(norm(F_applied), F_max)) * (initial_force/norm(initial_force))
fapplieds{index}=F_applied;

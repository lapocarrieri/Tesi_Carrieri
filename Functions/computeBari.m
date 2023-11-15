%compute the baricenter given a set of points
function baricenter = computeBari(points)

    N = length(points);
    x = sum(points(1,:))/N;
    y = sum(points(2,:))/N;
    z = sum(points(3,:))/N;
    
    baricenter = [x y z];

end
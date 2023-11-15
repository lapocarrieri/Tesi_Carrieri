function plotRF(R, tran)

    origin = tran;
    length_axis = 0.05;
    x = R*[length_axis 0 0]'+tran;
    y = R*[0 length_axis 0]'+tran;
    z = R*[0 0 length_axis]'+tran;

    line([origin(1) x(1)], [origin(2) x(2)], [origin(3) x(3)],"color", "r", "lineWidth", 1);
    hold on 
    line([origin(1) y(1)], [origin(2) y(2)], [origin(3) y(3)],"color", "g", "lineWidth", 1);
    hold on 
    line([origin(1) z(1)], [origin(2) z(2)], [origin(3) z(3)],"color", "b", "lineWidth", 1);
    
end
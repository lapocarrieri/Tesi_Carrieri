
function plotForces(out, dofs, lineWidth)
    for i=1:3
        plot(out.tout, out.contact_force.Data(:,i), "lineWidth", lineWidth);
        hold on
        plot(out.tout, out.estimated_force.Data(:,i), "lineWidth", lineWidth);
    end
    legend("real F_x", "estimated F_x", "real F_y", "estimated F_y" , "real F_z", "estimated F_z");
    grid on
end

function plotForces2(time,Fc, lineWidth)

    for i=1:3
        plot(time(1:length(Fc(i,:))), Fc(i,:), "lineWidth", lineWidth);
        hold on 
    end

    grid on
end

function plotTorque(out, dofs, lineWidth)
    legend_torque = [];
    for i=1:dofs
        plot(out.tout, out.real_contat_torques.Data(:,i), "lineWidth", lineWidth);
        hold on
        plot(out.tout, out.r.Data(:,i), "lineWidth", lineWidth);
        real_contact_str = "real tauk_"+i;
        estimated_contact_str = "estimated tauk_"+i;
        legend_torque = [legend_torque, [real_contact_str, estimated_contact_str]];
    end
    legend(legend_torque);
    grid on
end

function plotObjectiveFunction(time, val, lineWidth)
    plot(time(1:length(val)), val, "lineWidth", lineWidth);
    grid on 
end


function plotArrow(point, direction, arrow_color, arrow_size)
        %this plot the arrow at the beginning of the line <---
        last_point1 = point + 0.01*(direction+tan(60));
        last_point2 = point + 0.01*(direction+-tan(60));
        cf_plot1 = line([point(1), last_point1(1)], [point(2), last_point1(2)], [point(3), last_point1(3)],"color", arrow_color, "lineWidth", arrow_size);
        cf_plot2 = line([point(1), last_point2(1)], [point(2), last_point2(2)], [point(3), last_point2(3)],"color", arrow_color, "lineWidth", arrow_size);
end

%plot num_part particlces on the cylinder sourface where
%r is the cylinder radius h is its height and c its center 
function plotParticles(chi,R, t)
    chi = R*chi(:,:)+t;
    for i=1:length(chi)
        hold on 
        plot3(chi(1,i),chi(2, i), chi(3, i),".", "Color", "red");
    end
   
end


%plot the RF in a generic point with a generic orientation. The lenght of
%the axis are fixed but eventually can be passed as a parameters
function plotRF(R, tran)

    origin = tran;
    length_axis = 0.05;
    x = R*[length_axis 0 0]'+tran;
    y = R*[0 length_axis 0]'+tran;
    z = R*[0 0 length_axis]'+tran;

    line([origin(1) x(1)], [origin(2) x(2)], [origin(3) x(3)],"color", "r", "lineWidth", 3);
    hold on 
    line([origin(1) y(1)], [origin(2) y(2)], [origin(3) y(3)],"color", "g", "lineWidth", 3);
    hold on 
    line([origin(1) z(1)], [origin(2) z(2)], [origin(3) z(3)],"color", "b", "lineWidth", 3);
    
end
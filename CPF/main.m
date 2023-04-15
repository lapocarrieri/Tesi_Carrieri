%plots the cylinder that approximate the last link, particles, estimated
%external force, actual external force.
close all
clear initialization

global is_initialized;
is_initialized = false;

num_part = 50;
chi = zeros(3,num_part);
%use estimated_contact_point function

dofs = 6;
lineWidth = 1.5;
plot_pos_contact = false; %plot position ee and position contact point
sp_rows = 2;              %rows subplot
sp_cols = 2;              %cols subplot
is_recording = false;
file_name = 'contact_040722_5.avi';
frame_rate = 80;

fval = [];
Fc = [];

lambda = -1:0.01:1;

%if you want to modify the initial simulation time change initial_idx, same
%for final_idx
initial_idx = 2550;
final_idx = 5550;
speed = 10;                     %change the execution speed

max_force = 120 % max(max(out.estimated_force(:,:)));
min_force = 20 %min(min(out.estimated_force(:,:)));

max_torque = 120%max(max(out.r(:,:)));
min_torque = 10 %min(min(out.r(:,:)));



if(plot_pos_contact)
    f1 = figure(1);
    %( estimated_contact_point, "lineWidth", lineWidth);%plot( out.estimated_contact_point, "lineWidth", lineWidth);
    legend("x_c", "y_c", "z_c")
    title("Contact Point");
    grid on

    f2 = figure(2);
    plot(out.tout, squeeze(out.T1.data(1,4,:)));
    hold on
    plot(out.tout, squeeze(out.T1.data(2,4,:)));
    hold on
    plot(out.tout, squeeze(out.T1.data(3,4,:)));
    legend("x", "y", "z");
    title("ee position");
    grid on
    
    f3 = figure(3);
    plotForces(out, dofs, lineWidth);
else

    if(is_recording)
        %set up the video
        v = VideoWriter(strcat('../energy_momentum/lara5/data/videos/', file_name),'Uncompressed AVI');
        v.FrameRate = frame_rate;
        open(v)
    end

    h = figure('units','normalized','outerposition',[0 0 1 1]);
    text(0.5,0.5,'Explanation'); axis off;
    subplot(sp_rows,sp_cols,1);
    plotForces(out, dofs, lineWidth);

    subplot(sp_rows,sp_cols,3);
    plotTorque(out, dofs, lineWidth)


    ll = subplot(sp_rows,sp_cols,[2 4]);
    
    r = 0.045;           % radii of cyl
    cnt = [0,0,0];       % [x,y,z] center cyl
    height = -0.05;      % height of cyl, is negative because the frame is on the top 
    color = [128 128 128]/255;    % color of cyl
    nSides = 100;        % number of "sides" of the cyl
    hold on
  

    [X,Y,Z] = cylinder(r);
    X = X + cnt(1); 
    Y = Y + cnt(2); 
    Z = Z * height;


    for i=initial_idx:speed:final_idx
    
        is_collided = out.is_collided.data(i);
        
        ll =  subplot(sp_rows,sp_rows,[2 4]);
        ll = cla(ll); %i have to clear each time otherwise overlaps the cylinder
        hold on
        
        %% link
        %transformation of both the circumference upper and lower using
        %R*p+t
        R = out.T1.data(1:3,1:3,i);
        tran = out.T1.data(1:3,4,i);
        hom1 = R*[X(1,:);Y(1,:);Z(1,:)]+tran;
        hom2 = R*[X(2,:);Y(2,:);Z(2,:)]+tran;
        X_prime(1,:) = hom1(1, :);
        X_prime(2,:) = hom2(1, :);

        Y_prime(1,:) = hom1(2, :);
        Y_prime(2,:) = hom2(2, :);

        Z_prime(1,:) = hom1(3, :);
        Z_prime(2,:) = hom2(3, :);
        
        %plot the cylinder
        h1 = surf(X_prime,Y_prime,Z_prime,'facecolor',color,'LineStyle','none','FaceAlpha','0.5');
        h2 = fill3(X_prime(1,:),Y_prime(1,:),Z_prime(1,:),color);
        h3 = fill3(X_prime(2,:),Y_prime(2,:),Z_prime(2,:),color);  
        
        %plot the dh reference frame
        plotRF(R, tran)
        
        estimated_contat_point_prime = R*out.estimated_contact_point_prime.Data(i,1:3)' + tran;
        [chi, W] = cpf(num_part,chi, out.q.data(i,:), out.r.Data(i,:), estimated_contat_point_prime);
        plotParticles(chi, R, tran);
        
         contact_point_PF = computeBari(R*chi+tran);
%          [F1, F2, F3, F4] = computeFrictionCone(contact_point_PF', 0, R);
%          F = [F1, F2, F3, F4] + contact_point_PF'
         [~,fval(end+1)] = observationModel(eye(6)*7, out.q.data(i,:), out.r.Data(i,:),contact_point_PF);
%          line([F1(1) F2(1)], [F1(2) F2(2)], [F1(3) F2(3)]);
%          line([F2(1) F4(1)], [F2(2) F4(2)], [F2(3) F4(3)]);
%          line([F3(1) F4(1)], [F3(2) F4(2)], [F3(3) F4(3)]);
%          line([F3(1) F1(1)], [F3(2) F1(2)], [F3(3) F1(3)]); 
        
        z_max = R*[0 0 0.02]'+tran;
        z_min = R*[0 0 -0.1]'+tran;
            
        %contact_point_prime = R*contact_point' + tran;
        contact_point_prime = contact_point' + tran ;
        
       
        %% contact point estimation
        if(is_collided)      
            hold on 
            p_est_cont =  plot3(estimated_contat_point_prime(1), estimated_contat_point_prime(2), estimated_contat_point_prime(3), "c.", "lineWidth", 15);
            hold on 
            p_cont = plot3(contact_point_prime(1), contact_point_prime(2), contact_point_prime(3), "g.", "lineWidth", 15);
            hold on 
            
            plot3(contact_point_PF(1), contact_point_PF(2), contact_point_PF(3), "bx", "lineWidth", 3);

            %p(lambda) = estimated_contant_point + lambda*(estimated force direction)
            
%             estimated_contact_point2 = out.estimated_contact_point2.Data(i,:);
%             action_line = estimated_contat_point_prime + lambda.*(out.estimated_force.Data(i,:)/norm(out.estimated_force.Data(i,:)))';
%             hold on
%             action_line_plot = line([action_line(1,1) action_line(1,end)], [action_line(2,1) action_line(2,end)], [action_line(3,1) action_line(3,end)],'LineStyle','--', "color", '#EDB120', "lineWidth", 1.5);
%             hold on 
%             plot3(out.estimated_contact_point1.Data(i,1),out.estimated_contact_point1.Data(i,2),out.estimated_contact_point1.Data(i,3),"r.", "lineWidth", 15);
%             hold on 
%             plot3(out.estimated_contact_point2.Data(i,1),out.estimated_contact_point2.Data(i,2),out.estimated_contact_point2.Data(i,3),"r.", "lineWidth", 15);
            
            %% contant force
            %ground trouth
            cf = out.contact_force.Data(i,:)';
            cf_direction = -cf/norm(cf);
            lambda_final = norm(cf)/100;
            last_point = contact_point_prime + lambda_final*cf_direction;
            cf_plot = line([contact_point_prime(1), last_point(1)], [contact_point_prime(2), last_point(2)], [contact_point_prime(3), last_point(3)],"color", "g", "lineWidth", 3);
            plotArrow(contact_point_prime, cf_direction, "g", 3);
            
            %estimated force
%             ecf = out.estimated_force.Data(i,:);
%             ecf_direction = -ecf/norm(ecf);
%             elambda_final = norm(ecf)/100;
%             elast_point = estimated_contact_point2' + elambda_final*ecf_direction';
%             ecf_plot = line([estimated_contact_point2(1), elast_point(1)], [estimated_contact_point2(2), elast_point(2)], [estimated_contact_point2(3), elast_point(3)],"color", "r", "lineWidth", 3);
%             plotArrow(estimated_contact_point2', ecf_direction', "r", 3);
            
            legend([cf_plot],  "external force gt");
        else 
            legend off
        end
        
        title("6-th Link");
        %view(-15,15);
        view(45,5);
        grid on


        xlim([-0.5 0.05]);
        ylim([-0.478 0.158]);
        zlim([0.667910235373253 1.30405666293198]);
%         

%         xlim([-0.8 0.4]);
%         ylim([-0.1 0.3]);
%         zlim([0.4 0.9]);

%          xlim([-0.5 0]);
%         ylim([-0.2 0.4]);
%         zlim([0.1 0.5]);

        xlabel("x");
        ylabel("y");
        zlabel("z");
        drawnow();

    
        
        f_p = subplot(sp_rows,sp_cols,1);
        cla(f_p);
        plotForces(out, dofs, lineWidth);
%         plotForces2(out.tout,Fc, lineWidth);
%         hold on 
        line([out.tout(i),out.tout(i)],[min_force-5,max_force+5], "color", "k");
        title("Contact Forces");
        xlabel("Time [s]");
        ylabel("Force [N]");
        ylim([min_force-5,max_force+5]);

        
% % % %         s_p = subplot(sp_rows,sp_cols,3);
% % % %         cla(s_p);
% % % %         plotTorque(out, dofs, lineWidth);
% % % %         hold on 
% % % %         line([out.tout(i),out.tout(i)],[min_torque-5,max_torque+5], "color", "k");
% % % %         title("External Torque");
% % % %         xlabel("Time [s]");
% % % %         ylabel("Torque [Nm]");
% % % %         ylim([min_torque-5,max_torque+5]);
        
        s_p = subplot(sp_rows,sp_cols,3);
        cla(s_p);
        plotObjectiveFunction(out.tout, fval, lineWidth);

        dimention = [0.25 0.22 .3 .3];
        if exist('a','var')
          delete(a)
        end
        string = sprintf("%.3f",out.tout(i));
        string =  strcat("Time: ", string , " s");
        a = annotation('textbox',dimention,'String',string,'FitBoxToText','on', 'EdgeColor','r', 'FontSize',24);
       
        
        if(is_recording)
            frame = getframe(gcf);
            writeVideo(v, frame);       
        end

    end
    
    if(is_recording)
     close(v);
    end
end


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
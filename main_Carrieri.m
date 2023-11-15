clc
clear all
close all
addpath 'C:\Users\lapoc\OneDrive\Desktop\MATLAB_Simulator'
addpath 'C:\Users\lapoc\OneDrive\Desktop\MATLAB_Simulator\Dynamics'
addpath 'C:\Users\lapoc\OneDrive\Desktop\MATLAB_Simulator\Functions'
addpath 'C:\Users\lapoc\OneDrive\Desktop\MATLAB_Simulator\File_mat'
prompt = "Write the filename: ";
filename = input(prompt)
load(filename);
n=7;

% If the joint friction is taken into account, open forceReconstruction and
% uncomment line 22







Q = Q(1:samples, 1:n);
QD = QD(1:samples, 1:n);
QDD = QDD(1:samples, 1:n);
%Q = NoncausalButterworthFilter(Q);

TAU = TAU(1:samples, 1:7);
%TAU = NoncausalButterworthFilter(TAU);




%QD = TimeDerivative(Q, DeltaT);
%QDD = TimeDerivative(QD, DeltaT);



% P = zeros(samples,1);
% PD = zeros(samples,1);
% 
% for t=1:samples
%     q = Q(t, :)';
%     qd = QD(t, :)';
%     
%     T = get_transform(q,needle_length);
%     P(t) = T{7}(3,4)';
% 
%     J = Geometric_Jacobian(q, needle_length, n);
%     PD(t) = J(3,:)*qd;
% end
% P = NoncausalButterworthFilter(P);
% PD = NoncausalButterworthFilter(PD);


[R, F_EXT_RECON, TAU_PREDICT,TAU_FRICTION,link,is_collided] = forceReconstruction(gain, samples, n, DeltaT, Q, QD, QDD, TAU,threshold_Collision);
%  TAU_error = TAU_PREDICT(end-20:end,:) -TAU(end-20:end,:)
%  
%  Residual_error = (R(end-10:end,1:2)-TauExtForce(end-10:end,1:2))
% 
% R(end-10:end,1:2)
% TauExtForce(end-10:end,1:2)
% 
% return;

estimated_cp = f(Q(end,1),Q(end,2),Q(end,3),Q(end,4),Q(end,5),Q(end,6));


%     h = figure('units','normalized','outerposition',[0 0 1 1]);
%     text(0.5,0.5,'Explanation'); axis off;
%     subplot(sp_rows,sp_cols,1);
%     plotForces(out, dofs, lineWidth);
% 
%     subplot(sp_rows,sp_cols,3);
%     plotTorque(out, dofs, lineWidth)
% 
% 
%     ll = subplot(sp_rows,sp_cols,[2 4]);
    
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


%     for i=initial_idx:speed:final_idx
    
        
        T = QtoP(Q(end,:,link);
       
        R = T(1:3,1:3);
        tran = T(1:3,4);

        
        estimated_contat_point_prime = R*estimated_cp' + tran;
        [chi, W] =cpf(num_part, chi, Q(end,:), R(end,:), estimated_contat_point_prime,linkSampled)
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
    
   
   


    

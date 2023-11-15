

    %to test it observationModel(eye(4)*1,[0 -pi/3 pi/4 pi/2 0 0 0],[1 3 2
    %0 0 0 0],[1;1;1;1],4)
    link=5;
    q=[0 0 pi/3 pi/5 0 0 0];
    chi=[0.5 0.2 0.11]'
    
   
    
    f_i=[3 4 3]'; % force in the actual frame
    Sf_i=[0 -f_i(3) f_i(2) ; f_i(3) 0 -f_i(1) ; -f_i(2) f_i(1) 0 ];
                m=-Sf_i*chi
       T_actualframe= vpa(simplify(QtoP(q,link)),3);
       R=T_actualframe(1:3,1:3);
        [Jc2,~] = compute_jacobian2(q,chi,link);
        [Jc,~] = compute_jacobian(q,chi,link);
      
       
% Tau=[1 2 2 5 0 0 0]'
% w= pinv(J_withwrenches)'*Tau
% w2=pinv(Jc)'*Tau
% w3=pinv(Jc2)'*Tau
% 
% return;
J_withwrenches = ComputePoint_withWrenches(q,link);
F=R*f_i;
TauExternalForce=((J_withwrenches)'*[f_i;m])'

TauExternalForce2 =(Jc'*f_i)'
TauExternalForce3 =vpa((Jc2'*f_i)',4)
TauExternalForce22 =vpa(((R'*Jc)'*f_i)',4)
TauExternalForce33 =vpa(((R'*Jc2)'*f_i)',4)
TauExternalForce222 =vpa(((Jc)'*F)',4)
TauExternalForce333 =vpa(((Jc2)'*F)',4)
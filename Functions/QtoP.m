function [B] = QtoP(qq,link)
    syms q1 q2 q3 q4 q5 q6 q7
    q = [q1;q2;q3;q4;q5;q6;q7];
        
    C1=cos(q1);
    S1=sin(q1);
    C2=cos(q2);
    S2=sin(q2);
    C3=cos(q3);
    S3=sin(q3);
    C4=cos(q4);
    S4=sin(q4);
    C5=cos(q5);
    S5=sin(q5);
    C6=cos(q6);
    S6=sin(q6);
    C7=cos(q7);
    S7=sin(q7);
    L = 0.4;
    M=0.39;
    h=0.31;
    
    
    %  l1=0.0;     % for real robot where the base frame is on the second joint
    %  l2=0.4;
    %  l3=0.39;
      l4=0.078;   % EE in the tip of KUKA without auxiliary addition
    
    M1 = [C1,0,S1,0; S1,0,-C1,0; 0,1,0,0; 0,0,0,1];
    M2 = [C2,0,-S2,0; S2,0,C2,0; 0,-1,0,0; 0,0,0,1];
    M3 = [C3,0,-S3,0; S3,0,C3,0; 0,-1,0,L; 0,0,0,1];
    M4 = [C4,0,S4,0; S4,0,-C4,0; 0,1,0,0; 0,0,0,1];
    M5 = [C5,0,S5,0; S5,0,-C5,0; 0,1,0,M; 0,0,0,1];
    M6 = [C6,0,-S6,0; S6,0,C6,0; 0,-1,0,0; 0,0,0,1];
    M7 = [C7,-S7,0,0; S7,C7,0,0; 0,0,1,l4; 0,0,0,1];
    wTr = [1 0 0 0; 0 1 0 0; 0 0 1 h; 0 0 0 1];
    A{7} = (((((wTr*M1*M2)*M3)*M4)*M5)*M6)*M7;
   
    A{6} = ((((wTr*M1*M2)*M3)*M4)*M5)*M6;
    A{5} = (((wTr*M1*M2)*M3)*M4)*M5;
    A{4} = ((wTr*M1*M2)*M3)*M4;
    A{3} = (wTr*M1*M2)*M3;
    A{2} = wTr*M1*M2;
    A{1} = wTr*M1;
    for i=1:link
        A{i}=subs(A{i},{q1,q2,q3,q4,q5,q6,q7},{qq(1),qq(2),qq(3),qq(4),qq(5),qq(6),qq(7)});


    end
    B=A{link};
    
end
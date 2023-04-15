function [dJdt] = compute_derivative_jacobian(Jsubs,qq)
    syms t
    syms Q(t) Q1(t) Q2(t) Q3(t) Q4(t) Q5(t) Q6(t) Q7(t)
     dJdt = diff(Jsubs,t);
    dJdt = subs(dJdt,{diff(Q1),diff(Q2),diff(Q3),diff(Q4),diff(Q5),diff(Q6),diff(Q7)}...
    ,{qq(8),qq(9),qq(10),qq(11),qq(12),qq(13),qq(14)})
    dJdt=subs(dJdt,{Q1,Q2,Q3,Q4,Q5,Q6,Q7},{qq(1),qq(2),qq(3),qq(4),qq(5),qq(6),qq(7)});
    dJdt=dJdt(1:3,:);
end
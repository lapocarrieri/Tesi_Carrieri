function dqdt = kuka_lwr(t,q,acc)
d2q = acc;
dq = q(8:14);
dqdt = [dq;d2q'];
end

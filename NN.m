clc
clear all
load('C:\Users\lapoc\OneDrive\Desktop\MATLAB_Simulator\Mio\Residuals3.mat')
Tau_calculated=vpa(Tau_calculated',3);
size(Tau_calculated)


A = Tau_calculated(1:3,1:100)
net = feedforwardnet(10);

targets = link_calculated(1:100)';
net.numinputs = 7;
[net,tr] = train(net,A,targets);
view(net)
output = net([1 5]')
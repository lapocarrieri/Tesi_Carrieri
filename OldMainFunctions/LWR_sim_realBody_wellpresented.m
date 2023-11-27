%% Description
% In LWR_sim.m, I have written code to calculate the residuals 'r' and 'sigma' to
% identify which link is subjected to force. The application point is computed
% using the residual, which is then used as an initialization for the CPF (Contact Particle Filter).
% I have implemented the Contact Particle Filter that works well if the initial value is good.
% Additionally, the point calculated in the previous sample-instant is used in the next,
% assuming the point is almost constant.
% Plots have been created to observe the behavior of the particles and the error trajectory
% in comparison to the true application point.
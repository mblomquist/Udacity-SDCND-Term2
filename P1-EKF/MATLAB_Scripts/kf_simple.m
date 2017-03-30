function [mu_new,sigma_new] = kf_simple(mu_old,sigma_old,u_t,z_t,A,B,C,R,Q)
% Kalman Filter Algorithm
%   Reference Kalman Filter Algorithm pg 42, Probabilistic Robotics
%   by Thrun, Burgard, and Fox.

mu_hat = A*mu_old+B*u_t;
sigma_hat = A*sigma_old*A'+R;

K = sigma_hat*C'/(C*sigma_hat*C'+Q);

mu_new = mu_hat+K*(z_t-C*mu_hat);
sigma_new = (eye(size(sigma_hat))-K*C)*sigma_hat;

end

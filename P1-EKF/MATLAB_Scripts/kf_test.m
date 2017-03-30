% Kalman Filter Testing Script
close all
clear all
clc

% Number of Steps
n = 101;

% Create simple linearized model
A = [1 1; 0 1];
B = [1;0];
C = [1 0];

% Set Covariance Matrices
R = 1;
Q = 1;

% Set Initial Values for Mean and Variance
sigma_old = [0 0; 0 0];
mu_old = [0;0];

% Set Control Inputs // No inputs
u_t = [0 0];

% Create empty array for measurements.
z_t = zeros(n,2);

% Fill measurements array with a function.
for i = 1:1:n
    z_t(i,:) = [-.001*(50-i)^2+.01*i+5 0];
end

% Create empty array for storing position data.
mu = zeros(1,n);

% Set first position value to estimate.
mu(1) = mu_old(1);

% Run Kalman filter algorithm for 2:n steps
for i = 2:1:n
        
    [mu_old,sigma_old] = kf_algo(mu_old,sigma_old,u_t,z_t(i,:),A,B,C,R,Q);
    mu(i) = mu_old(1);
    
end

% Create empty array for rms error.
e = zeros(1,n);

% Calculate rms error
for i = 1:1:n
    e(i) = sqrt(mean(mu(i)-z_t(i,1))^2);
end

% Plot information
subplot(2,1,1);
plot(0:1:n-1,z_t(:,1),'b-o');
hold on
plot(0:1:n-1,mu,'g--*') 

grid minor
title 'Kalman Filter Tracking'
xlabel 'Iteration'
ylabel 'Position Estimate'
legend 'Actual Position' 'Estimated Position' 

subplot(2,1,2);
plot(0:1:n-1,e);

title 'Error_{rms}'
grid minor
text(50,max(e)*.95,'f(i) = -.001*(50-i)^2+.01*i+5')
text(50,max(e)*.75,strcat('Avg Error_{rms}: ',num2str(mean(e))))

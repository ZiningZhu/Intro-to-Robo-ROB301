x0 = 100;
x_tgt = 0;
x = [x0];
z = [];
sim_time = 100;

%% No noise
clear all; clc;
x0 = 100;
x_tgt = 0;
x = [x0];
z = [];
sim_time = 100;


v = 0; w = 0;
x_k = x0;
K = 0.1;
for k=1:sim_time
    z_k = x_k;
    u_k = -K * z_k;
    x_k = x_k + u_k;
    x = [x x_k];
end

figure;
plot(x);
title(strcat('no noise: std(x)=', num2str(std(x))));
xlabel('time step');
ylabel('x');
print('NoNoise', '-dpng');

x_baseline = x;
%% Apply noise, no Kalman
close all; clc;
x0 = 100;
x_tgt = 0;
x = [x0];
z = [];
sim_time = 100;

r = 20;
x_k = x0;

K = 0.1;
for k = 1:sim_time
    w_k = rand * 2 * r - r;
    v_k = rand * 2 * r - r;
    z_k = x_k + w_k;
    u_k = -K * z_k;
    x_k = x_k + u_k + v_k;
    x = [x x_k];
end
figure;
plot(x);
title(strcat('Applied noise: r=20, std(x-x_{base})=', num2str(std(x-x_baseline))));
xlabel('time step');
ylabel('x');
print('WithNoise_r=20', '-dpng');

%% Kalman filter
clc;close all;
x0 = 100;
x_tgt = 0;
x = [x0];     % estimated x values
xtrue = [x0]; % true x values
z = [];       % true measurement values
ze = [];      % estimation of measurement
u = [-10];    % Need this random initialization
sim_time = 100;

r = 1;
x_k = x0;

K = 0.1;

P = [1/(2*r)]; % covariances of x. Randomly initialized
Q = 1/(2*r); % covariance of v
R = 1/(2*r); % covariance of w
S = []; % meqsurement residual
W = []; % Kalman gain. 

for k = 2:sim_time
    w_k = rand * 2 * r - r;
    v_k = rand * 2 * r - r;
    
    xtrue(k) = xtrue(k-1) + u(k-1) + v_k; % State update. Not part of Kalman
    x(k) = x(k-1) + u(k-1); % a priori state estimation
    ze(k) = x(k) + w_k;     % a priori measurement prediction
    z(k) = xtrue(k) + w_k;
    s(k) = z(k) - ze(k);
    
    P(k) = P(k-1) + Q;
    S(k) = P(k) + R;
    W(k) = P(k) / S(k);
    P(k) = P(k) - W(k)*S(k)*W(k);
    
    x(k) = x(k) + W(k) * s(k); % a posteriori estimation
    
    u(k) = -K * x(k);  % controlling command
    
    
end
figure;
plot(xtrue);
xtrue = [x0 xtrue]
title(strcat('Kalman with noise: r=1, std(x-x_{base})=', num2str(std(xtrue-x_baseline))));
xlabel('time step');
ylabel('x');
print('KalmanWithNoise_r=1', '-dpng');



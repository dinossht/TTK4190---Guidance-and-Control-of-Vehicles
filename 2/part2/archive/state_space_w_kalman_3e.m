clc; clear; close all;

%Unit conversions
deg2rad = pi/180;
rad2deg = 180/pi;
km_h2m_s = 1/3.6;
m_s2km_h = 3.6;

%Constants
a_phi_1 = 2.87; a_phi_2 = -0.65;
g = 9.81; d = 1.5; vg = 580*km_h2m_s;

%Roll controller params
aileron_max = 30;
error_max = 15;
kp_phi = (aileron_max/error_max)*sign(a_phi_2); 

omega_n_phi = sqrt(abs(a_phi_2)*aileron_max/error_max);
zeta_phi = 0.707;  % givenb
kd_phi = (2*zeta_phi*omega_n_phi-a_phi_1)/a_phi_2; 
%ki_phi = 0;

%Course controller
omega_chi_phi = omega_n_phi/10;  % ten times lower bandwidth
zeta_chi = 0.5; % design choice
kp_chi = 2*zeta_chi*omega_chi_phi*vg/g;

ki_chi = omega_chi_phi^2*vg/g;

%Measurement noise
roll_rate_noise_power = (0.2*deg2rad)^2;
yaw_rate_noise_power = (0.2*deg2rad)^2;

%Course angle input series (in deg)
step_series = [0 -15 10];


% Simulink & plotting  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
simtime = 600;
task_nr = 0;
sensor_fail_time = -1; %simtime/2;
model = 'state_space_w_kalman_model_3.slx';
sim(model);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%s
%% 
load('course.mat');
time_c      = ans(1,:);
course      = ans(2,:);
course_ref  = ans(3,:);

figure(1);
plot(time, course_ref,'b'); hold on;
plot(time, course,'g');
title('Course vs. reference'); legend('Course-ref', 'Course');
xlabel('Time [s]'); ylabel('Deg'); grid on;

%%
figure(2);

load('measurements.mat');
time        = ans(1,:);
p_mes       = ans(2,:);
r_mes       = ans(3,:);

subplot(2,1,1);
plot(time, p_mes,'r'); hold on;

subplot(2,1,2);
plot(time, r_mes,'r'); hold on;

load('estimates.mat');
time        = ans(1,:);
p_est       = ans(4,:);
r_est       = ans(5,:);

subplot(2,1,1);
plot(time, p_est,'g'); 
title('Roll rate'); legend('Measurement', 'Estimate');
xlabel('Time [s]'); ylabel('Deg/s'); ylim([-10 10]); grid on;

subplot(2,1,2);
plot(time, r_est,'g'); 
title('Yaw rate'); legend('Measurement', 'Estimate');
xlabel('Time [s]'); ylabel('Deg/s'); ylim([-10 10]); grid on;

%%
figure(3);

load('measurements.mat');
time        = ans(1,:);
p_mes       = ans(2,:);
r_mes       = ans(3,:);

subplot(2,1,1);
plot(time, p_mes,'r'); hold on;

subplot(2,1,2);
plot(time, r_mes,'r'); hold on;

load('states.mat');
p           = ans(4,:);
r           = ans(5,:);

subplot(2,1,1);
plot(time, p,'g'); 
title('Roll rate'); legend('Measurement', 'Actual');
xlabel('Time [s]'); ylabel('Deg/s'); ylim([-10 10]); grid on;

subplot(2,1,2);
plot(time, r,'g'); 
title('Yaw rate'); legend('Measurement', 'Actual');
xlabel('Time [s]'); ylabel('Deg/s'); ylim([-8 8]); grid on;

%%
figure(4);

load('estimates.mat');
time        = ans(1,:);
beta_est    = ans(2,:);
phi_est     = ans(3,:);
p_est       = ans(4,:);
r_est       = ans(5,:);

subplot(4,1,1);
plot(time, beta_est,'g'); hold on; 

subplot(4,1,2);
plot(time, phi_est,'g'); hold on;

subplot(4,1,3);
plot(time, p_est,'g'); hold on;

subplot(4,1,4);
plot(time, r_est,'g'); hold on;

load('states.mat');
time        = ans(1,:);
beta        = ans(2,:);
phi         = ans(3,:);
p           = ans(4,:);
r           = ans(5,:);

subplot(4,1,1);
plot(time, beta,'r'); 
title('Sideslip'); legend('Estimate', 'Actual');
xlabel('Time [s]'); ylabel('Deg'); ylim([-1 1]); grid on;

subplot(4,1,2);
plot(time, phi,'r');
title('Roll'); legend('Estimate', 'Actual');
xlabel('Time [s]'); ylabel('Deg'); ylim([-40 60]); grid on;

subplot(4,1,3);
plot(time, p,'r');
title('Roll rate'); legend('Estimate', 'Actual');
xlabel('Time [s]'); ylabel('Deg/s'); ylim([-10 10]); grid on;

subplot(4,1,4);
plot(time, r,'r'); 
title('Yaw rate'); legend('Estimate', 'Actual');
xlabel('Time [s]'); ylabel('Deg/s'); ylim([-5 5]); grid on;
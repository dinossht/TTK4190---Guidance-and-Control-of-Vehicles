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
step_series = [15 0 -15];%[0 2.5 5 7.5 10 12.5 15];
simtime = 300;

%% Simulate ideal model - no noise %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
sim('state_space_w_kalman_3_ideal.slx');
figure(1);
subplot(2,1,1);
load('out/course.mat');
time     = ans(1,:);
course   = ans(2,:);

plot(time, course, 'k'); hold on;

%% Simulink & plotting  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
task = 'e';

if task == 'e'
    task_nr = 0;
    sensor_fail_time = -1;
elseif task == 'f'
    task_nr = 1;
    sensor_fail_time = -1;  
elseif task == 'g'
    task_nr = 1;
    sensor_fail_time = simtime/2; 
end
    
model = 'state_space_w_kalman_model_3.slx';
sim(model);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% 
load('out/course.mat');
time        = ans(1,:);
course      = ans(2,:);
course_ref  = ans(3,:);

figure(1);
subplot(2,1,1);
plot(time, course_ref,'b'); hold on;
plot(time, course,'g');
title('Course response'); legend('Course-ideal', 'Course-desired.', 'Course');
xlabel('Time [s]'); ylabel('Deg'); grid on; ylim([-25 25]);

%%
figure(2);

load('out/measurements.mat');
time        = ans(1,:);
p_mes       = ans(2,:);
r_mes       = ans(3,:);

subplot(2,1,1);
plot(time, p_mes,'r'); hold on;

subplot(2,1,2);
plot(time, r_mes,'r'); hold on;

load('out/estimates.mat');
time        = ans(1,:);
p_est       = ans(4,:);
r_est       = ans(5,:);

subplot(2,1,1);
plot(time, p_est,'g'); hold on;
subplot(2,1,2);
plot(time, r_est,'g'); hold on;
%%
figure(2);
load('out/states.mat');
time        = ans(1,:);
p           = ans(4,:);
r           = ans(5,:);

subplot(2,1,1); 
plot(time, p,'k'); 
title('Roll rate'); legend('Measurement', 'Estimate', 'True');
xlabel('Time [s]'); ylabel('Deg/s'); ylim([-10 10]); grid on;

subplot(2,1,2);
plot(time, r,'k'); 
title('Yaw rate'); legend('Measurement', 'Estimate', 'True');
xlabel('Time [s]'); ylabel('Deg/s'); ylim([-10 10]); grid on;

%%
figure(3);

load('out/measurements.mat');
time        = ans(1,:);
p_mes       = ans(2,:);
r_mes       = ans(3,:);

subplot(2,1,1);
plot(time, p_mes,'r'); hold on;

subplot(2,1,2);
plot(time, r_mes,'r'); hold on;

load('out/states.mat');
time        = ans(1,:);
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

load('out/estimates.mat');
time        = ans(1,:);
beta_est    = ans(2,:);
phi_est     = ans(3,:);
p_est       = ans(4,:);
r_est       = ans(5,:);

subplot(4,1,1);
plot(time, beta_est,'r'); hold on; 

subplot(4,1,2);
plot(time, phi_est,'r'); hold on;

subplot(4,1,3);
plot(time, p_est,'r'); hold on;

subplot(4,1,4);
plot(time, r_est,'r'); hold on;

load('out/states.mat');
time        = ans(1,:);
beta        = ans(2,:);
phi         = ans(3,:);
p           = ans(4,:);
r           = ans(5,:);

subplot(4,1,1);
plot(time, beta,'g'); 
title('Sideslip'); legend('Estimate', 'Actual');
xlabel('Time [s]'); ylabel('Deg'); ylim([-1 1]); grid on;

subplot(4,1,2);
plot(time, phi,'g');
title('Roll'); legend('Estimate', 'Actual');
xlabel('Time [s]'); ylabel('Deg'); ylim([-40 60]); grid on;

subplot(4,1,3);
plot(time, p,'g');
title('Roll rate'); legend('Estimate', 'Actual');
xlabel('Time [s]'); ylabel('Deg/s'); ylim([-10 10]); grid on;

subplot(4,1,4);
plot(time, r,'g'); 
title('Yaw rate'); legend('Estimate', 'Actual');
xlabel('Time [s]'); ylabel('Deg/s'); ylim([-5 5]); grid on;

%% 
load('out/aileron.mat');
time        = ans(1,:);
aileron     = ans(2,:);

figure(1);
subplot(2,1,2);
plot(time, aileron,'b');
title('Aileron input'); ylim([-30 30]);
xlabel('Time [s]'); ylabel('Deg'); grid on;

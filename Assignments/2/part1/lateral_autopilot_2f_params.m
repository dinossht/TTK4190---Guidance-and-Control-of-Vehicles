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
zeta_phi = 0.707;  % given
kd_phi = (2*zeta_phi*omega_n_phi-a_phi_1)/a_phi_2; 
%ki_phi = 0;

%Course controller
omega_chi_phi = omega_n_phi/10;  % ten times lower bandwidth
zeta_chi = 0.5; % design choice
kp_chi = 2*zeta_chi*omega_chi_phi*vg/g;

ki_chi = omega_chi_phi^2*vg/g;

%Bias term in coordinated turn equation
d = 1.5;

%State space model
A = [
    -0.322  0.052   0.028   -1.12   0.002   ;
    0       0       1       -0.001  0       ;
    -10.6   0       -2.87   0.46    -0.65   ;
    6.87    0       -0.04   -0.32   -0.02   ;
    0       0       0       0       -7.5    ;];
B = [0 0 0 0 7.5]';
C = [eye(4) zeros(4,1)];

step_series = [15 0 -15];
%Simulink & plotting 2e)
sim('lateral_autopilot_2e.slx');
load('results.mat');
time =          ans(1,:);
course =        ans(2,:);
course_ref =    ans(3,:);
aileron =       ans(4,:);

%Plot 2d)
subplot(2,1,1);
plot(time, course_ref,'b');hold on;
plot(time, course,'g');hold on;

subplot(2,1,2);
plot(time, aileron,'g');hold on;

%Simulink & plotting 2f)
sim('lateral_autopilot_2f.slx');
load('results.mat');
time =          ans(1,:);
course =        ans(2,:);
course_ref =    ans(3,:);
aileron =       ans(4,:);

%Plot 2f)
subplot(2,1,1);
hold on;
plot(time, course,'r');
xlabel('Time [s]');
ylabel('Deg');
legend('Course-ref','Course-2e','Course-2f');
grid on;
title('Complete state space model with anti-windup ');

subplot(2,1,2);
plot(time, aileron,'r');
xlabel('Time [s]');
ylabel('Deg');
legend('Aileron-2e','Aileron-2f');
grid on;



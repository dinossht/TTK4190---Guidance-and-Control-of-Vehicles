clc;clear;

%Constants
a_phi_1 = 2.831; a_phi_2 = -0.629;
g = 9.81; d = 1.5; vg = 161.111;
deg2rad = pi/180;
rad2deg = 180/pi;

%Roll controller params
kp_phi = -2; kd_phi = 1.979; 
ki_phi = 0;

%Course controller
kp_chi = 3.284; ki_chi = 0.6549;

sim('kevin_simulation1d.slx');

% struct: time-aileron-course
load('results.mat');

time = ans(1,:);
aileron = ans(2,:);
course = ans(3,:);
desired_course = ans(4,:);

subplot(2,1,1);
plot(time, aileron,'r');
xlabel('Time [s]');
ylabel('Deg [�]');
legend('Aileron');
hold on;
grid on;

subplot(2,1,2);
plot(time, course, 'b');
ylim([-30 30]);
hold on;
plot(time, desired_course, 'g');
grid on;
xlabel('Time [s]');
ylabel('Deg [�]');
legend('Course', 'Desired course');

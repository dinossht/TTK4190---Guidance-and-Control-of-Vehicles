%% Information 
% This file is only an example of how you can start the simulation. The
% sampling time decides how often you store states. The execution  time
% will increase if you reduce the sampling time.

% Please note that the file "pathplotter.m" (only used in the second part
% of the assignment) shows the ship path during the path following and
% target tracking part of the assignment. It can be clever to adjust the sampling
% time when you use that file because it draws a sketch of the ship in the
% North-East plane at each time instant. Having a small sampling time will
% lead to multiple ship drawings on top of each other. 

% You should base all of your simulink models on the MSFartoystyring model
% and extend that as you solve the assignment. For your own sake, it is
% wise to create a new model and run file for each task.

% The msfartoystyring.m file includes the ship model. You are not allowed
% to change anything within that file. You need to include that file in
% every folder where you have a simulink model based on
% "MSFartoystyring.slx". 

% WP.mat is a set of six waypoints that you need to use in the second part of
% the assignment. The north position is given in the first row and the east
% position in the second row. 

clear;clc;close all;
%% Task 1.2 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% 10 degree step = 0.1745 rad

tstart=0;           % Sim start time
tstop=100000;       % Sim stop time
tsamp=10;           % Sampling time for how often states are stored. (NOT ODE solver time step)
                
p0=zeros(2,1);      % Initial position (NED)
v0=[6.63 0]';       % Initial velocity (body)
psi0=0;             % Inital yaw angle
r0=0;               % Inital yaw rate
c=0;                % Current on (1)/off (0)

step_time = 5000;
unit_step = 10*(pi/180);
sim basic; % The measurements from the simulink model are automatically written to the workspace.

%% Find K and T
K_ = (r(end) - r(5000));
K = K_/unit_step;

[P,I] = max(r+0.001 >= (r(5000)+0.63*K) & r-0.001 <= (r(5000)+0.63*K));
T = (5002.5-5000)*tsamp;

%% plot
figure(1);
plot(t,r,'color','k'); hold on;
line([0 t(end)],[r(5000)+K_ r(5000)+K_],'color','r'); hold on;
line([0 t(end)],[r(5000)+K_*0.632 r(5000)+K_*.632],'color','g'); hold on;
line([50015 50015],[r(5000) r(5000)+K_*.632],'color','b'); hold on;
legend('step response','K=-0.0155', '0.63*K', 'T=25s');

xlim([t(5000) t(5100)]);
ylim([-0.007 r(5000)]);
xlabel('time [s]');
ylabel('rad/s');
title('Step response of yaw rate');


%% Task 1.3 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% Task 1.4 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% (table 12.2 from textbook)
wb = 0.05;
wn = wb/0.64;  
damping = 1;

Kp = wn^2*T/-K;
Kd = (2*damping*wn-1)/(-K);

c=1;
sim MSFartoystyring;
%% yaw
figure(2);clf;
subplot(2,1,1);
plot(t,-psi);hold on;plot(t,yaw_ref); legend('psi','ref');
xlabel('time [s]'); ylabel('rad');
title('Yaw controller sinusoidal response')
xlim([0 15000])

subplot(2,1,2);
plot(t,-psi-yaw_ref);
xlabel('time [s]'); ylabel('rad');
title('Yaw reference-output error')
xlim([0 15000])

%% yaw rate
figure(3);clf;
subplot(2,1,1);
plot(t,-r);hold on;plot(t,yaw_rate_ref); legend('r','ref');
xlabel('time [s]'); ylabel('rad/s');
title('Yaw rate controller sinusoidal response')
xlim([0 15000])

subplot(2,1,2);
plot(t,-r-yaw_rate_ref);
xlabel('time [s]'); ylabel('rad/s');
title('Yaw rate reference-output error')
xlim([0 15000])

%% rudder input and saturation constraints
figure(4);clf;
plot(t,rudder_input);hold on;
line([0 t(end)],[0.4363 0.4363], 'color','r'); hold on;
line([0 t(end)],[-0.4363 -0.4363],'color','r');
legend('rudder input','saturation max', 'saturation min');
xlabel('time [s]'); ylabel('rad');
title('Rudder input and saturation limits')
xlim([0 15000])

%% Task 1.5 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Surge speed autopilot
%% Task 1.6 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
c=0;
sim basic_speed;

K_sp = 5.955/7.3;
T_sp = (5043-5000)*tsamp;



%% Task 1.8 funker ikke med current - on %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
Kp_sp = -50;%7/5.71;%1*(1/K_sp);
Kd_sp = -0.2;%0;
Ki_sp = 0.001;%0.0001;%0.01;%0.005;

cutoff = 10;

p0=zeros(2,1);      % Initial position (NED)
v0=[3 0]';          % Initial velocity (body)
psi0=0;             % Inital yaw angle
r0=0;               % Inital yaw rate
c=1;                % Current on (1)/off (0)

sim MSFartoystyring_speed_1_7;
clf;
plot(t,v(:,1));
% Ziegler Nichols - 

%% Task 2.1 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
p0=[0,0]';%[1000 700]';     % Initial position (NED)
v0=[6.63 0]';       % Initial velocity (body)
psi0=60*pi/180;     % Inital yaw angle
r0=0;               % Inital yaw rate
c=0;                % Current on (1)/off (0)

Delta  = 900;     %Lookahead distance [m]
Ki_LOS = 5e-6;
I_max  = 0.15;




load('WP.mat');
figure(8);
scatter(WP(1,:),WP(2,:)); hold on;
plot(WP(1,:),WP(2,:));

sim MSFartoystyring_2_1;

pathplotter(p(:,1), p(:,2),  psi, tsamp, 10, tstart, tstop, 0, WP)
ylim([0 50000])





















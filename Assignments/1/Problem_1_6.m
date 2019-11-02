% M-script for numerical integration of the attitude dynamics of a rigid 
% body represented by unit quaternions. The MSS m-files must be on your
% Matlab path in order to run the script.
%
% System:                      .
%                              q = T(q)w
%                              .
%                            I w - S(Iw)w = tau
% Control law:
%                            tau = constant
% 
% Definitions:             
%                            I = inertia matrix (3x3)
%                            S(w) = skew-symmetric matrix (3x3)
%                            T(q) = transformation matrix (4x3)
%                            tau = control input (3x1)
%                            w = angular velocity vector (3x1)
%                            q = unit quaternion vector (4x1)
%
% Author:                   2018-08-15 Thor I. Fossen and H�kon H. Helgesen

%% USER INPUTS
clear; clc; close all;
addpath(genpath("/home/dino/Documents/School/Courses/TTK4190 - Fartøystyring /MSS"));
h = 0.1;                     % sample time (s)
N  = 6000;                    % number of samples. Should be adjusted

% model parameters
m = 180;
r = 2;
I = m*r^2*eye(3);            % inertia matrix
I_inv = inv(I);

% constants
deg2rad = pi/180;   
rad2deg = 180/pi;


phi = -5*deg2rad;            % initial Euler angles
theta = 10*deg2rad;
psi = -20*deg2rad;

q = euler2q(phi,theta,psi);   % transform initial Euler angles to q



%% Setpoints
phi_d = phi;            % initial Euler angles
theta_d = theta;
psi_d = psi;
q_d = euler2q(phi_d,theta_d,psi_d);  
%% Invers quaternion
q_d_bar = [q_d(1,:) -q_d(2:4,:)']';



%% w_d setpoint
T_inv = [...
    1 0             -sin(theta_d);...
    0 cos(phi_d)    cos(theta_d)*sin(phi_d);...
    0 -sin(phi_d)   cos(theta_d)*cos(phi_d);];
euler_dot = [0 0 0]';
w_d = T_inv*euler_dot;




%%
w = [0 0 0]';                 % initial angular rates

table = zeros(N+1,21);        % memory allocation

%% FOR-END LOOP
for i = 1:N+1,   
   t = (i-1)*h;                  % time

   [phi,theta,psi] = q2euler(q); % transform q to Euler angles
   [J,J1,J2] = quatern(q);       % kinematic transformation matrices
   
   q_dot = J2*w;                        % quaternion kinematics

   
   
   Kp = 20;   
   Kd = 400*eye(3);   
   %% Error calculation

   q_tilde = quatprod(q_d_bar,q);

   w_tilde = w-w_d;
   
   %% State error feedback 
   tau = -Kd*w_tilde-Kp*q_tilde(2:4);            % control law  
   
   %%
   w_dot = I_inv*(Smtrx(I*w)*w + tau);  % rigid-body kinetics
   
   table(i,:) = [t q' phi theta psi w' tau' phi_d theta_d psi_d q_tilde'];  % store data in table
   
   q = q + h*q_dot;	             % Euler integration
   w = w + h*w_dot;
   
   q  = q/norm(q);               % unit quaternion normalization

   
   
   %% Time varying setpoints 
   phi_d = 0*deg2rad;            
   theta_d = 15*cos(0.1*t)*deg2rad;
   psi_d = 10*sin(0.05*t)*deg2rad;
   q_d = euler2q(phi_d,theta_d,psi_d);  
   %% Invers quaternion
   q_d_bar = [q_d(1,:) -q_d(2:4,:)']';   


    T_inv = [...
            1 0             -sin(theta_d);...
            0 cos(phi_d)    cos(theta_d)*sin(phi_d);...
            0 -sin(phi_d)   cos(theta_d)*cos(phi_d);];
        
    %% Shitty manual derivative
    euler_dot = [0 -1.5*sin(0.1*t)*deg2rad 0.5*cos(0.05*t)*deg2rad]';
    w_d = T_inv*euler_dot;      


end 

%% PLOT FIGURES
t       = table(:,1);  
q       = table(:,2:5); 
phi     = rad2deg*table(:,6);
theta   = rad2deg*table(:,7);
psi     = rad2deg*table(:,8);
w       = rad2deg*table(:,9:11); 
tau     = table(:,12:14);
phi_d     = rad2deg*table(:,15);
theta_d   = rad2deg*table(:,16);
psi_d     = rad2deg*table(:,17); 
q_tilde = rad2deg*table(:,18:21);


figure (1); clf;
subplot(3,1,1);
plot(t, phi, 'b');hold on;
plot(t, phi_d,'b.');
grid on;legend('\phi', '\phi_d')
title('Euler angles');
xlabel('time [s]'); 
ylabel('angle [deg]');

subplot(3,1,2);
plot(t, theta, 'r');hold on;
plot(t, theta_d,'r.');
grid on;legend('\theta', '\theta_d');
title('Euler angles');
xlabel('time [s]'); 
ylabel('angle [deg]');

subplot(3,1,3);
plot(t, psi, 'g');hold on;
plot(t, psi_d,'g.');
grid on;legend('\psi', '\psi_d');
title('Euler angles');
xlabel('time [s]'); 
ylabel('angle [deg]');

figure (2); clf;
hold on;
plot(t, w(:,1), 'b');
plot(t, w(:,2), 'r');
plot(t, w(:,3), 'g');
hold off;
grid on;
legend('x', 'y', 'z');
title('Angular velocities');
xlabel('time [s]'); 
ylabel('angular rate [deg/s]');

figure (3); clf;
hold on;
plot(t, tau(:,1), 'b');
plot(t, tau(:,2), 'r');
plot(t, tau(:,3), 'g');
hold off;
grid on;
legend('x', 'y', 'z');
title('Control input');
xlabel('time [s]'); 
ylabel('input [Nm]');

figure (4); clf;
hold on;
plot(t, q_tilde(:,2), 'b');
plot(t, q_tilde(:,3), 'r');
plot(t, q_tilde(:,4), 'g');
hold off;
grid on;
legend('e_\phi', 'e_\theta', 'e_\psi');
title('Tracking error');
xlabel('time [s]'); 
ylabel('error angle [deg]');
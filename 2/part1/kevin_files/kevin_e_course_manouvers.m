clc;clear;
%Roll controller params
kp_phi = -2; kd_phi = 1.979; 
ki_phi = 0;

%Course controller
kp_chi = 3.284; ki_chi = 0.6549;

%Integration params
h=0.01;
N = 180/h;

deg2rad = pi/180;
rad2deg = 180/pi;
delta_e_sat = 30*deg2rad;
g = 9.81; vg = 161.111;

%Bias on course
d = 1.5*deg2rad;

%References on course
course_d_1 = deg2rad*3;
course_d_2 = deg2rad*0;
course_d_3 = deg2rad*-2;
course_c = course_d_1;

%THe 5 states + time + coursedynamics + aileron + references
table = zeros(N+1,9);

%State space matrices
A = [[-0.322 0.052 0.028 -1.12 0.002];
    [0 0 1 -0.0010 0];
    [-10.6 0 -2.87 0.46 -0.65];
    [6.87 0 -0.04 -0.32 -0.02];
    [0 0 0 0 -7.5]];

B = [0 0 0 0 7.5]';

%Init vals of states
x = zeros(5,1);
chi = 0;

e_chi_int = 0;
e_phi_prev = 0;
for i=1:N+1
    t = (i-1)*h;
    
    %Changing references each 60 sek
    if (t == 60)
        course_c = course_d_2;
    end
    if (t == 120)
        course_c = course_d_3;
    end
    
    %PI controller on course error dynamics
    e_chi = course_c - chi;
    e_chi_int = e_chi*h + e_chi_int;
    phi_c = kp_chi*e_chi + ki_chi*e_chi_int;
    
    %PD Controller on roll error dynamics
    e_phi = phi_c - x(2);
    delta_a = kp_phi*e_phi + kd_phi*(e_phi-e_phi_prev)/h;
    e_phi_prev = e_phi;
    %Saturation
    u = min(delta_e_sat, max(-delta_e_sat,delta_a));
    
    table(i,:) = [t x' chi u course_c];
    
    x_dot = A*x+B*u;
    chi_dot = g/vg*tan(x(2))*cos(x(1)) + d;
    
    x = x + h*x_dot;
    chi = chi + h*chi_dot;
end

time = table(:,1);
aileron = table(:,8)*rad2deg;
course = rad2deg*table(:,7);
course_d = rad2deg*table(:,9);

subplot(2,1,1);
plot(time, aileron,'r');
xlabel('Time [s]');
ylabel('Deg [°]');
legend('Aileron');
grid on;

subplot(2,1,2);
plot(time, course, 'b');
ylim([-20 20]);
hold on;
plot(time, course_d, 'g');
xlabel('Time [s]');
ylabel('Deg [°]');
grid on;
legend('Course', 'Desired course');

tstart=0;           % Sim start time
tstop=100000;       % Sim stop time
tsamp=10;           % Sampling time for how often states are stored. (NOT ODE solver time step)
                
p0=zeros(2,1);      % Initial position (NED)
v0=[6.63 0]';       % Initial velocity (body)
psi0=0;             % Inital yaw angle
r0=0;               % Inital yaw rate
c=0;                % Current on (1)/off (0)


sim MSFartoystyring % T


K = (r(end) - r(5000))/unit_step;

[P,I] = max(r+0.001 >= (r(5000)+0.63*K) & r-0.001 <= (r(5000)+0.63*K));

T = (5002.5-5000)*tsamp;

%% plot
plot(t,r,'color','k'); hold on;
line([0 t(end)],[r(5000)+K r(5000)+K],'color','r'); hold on;
line([0 t(end)],[r(5000)+K*0.632 r(5000)+K*.632],'color','g'); hold on;
line([50015 50015],[r(5000) r(5000)+K*.632],'color','b'); hold on;
legend('step response','K=-0.0027', '0.63*K', 'T=25s');
%line([t(5000) 50015],[r(5000) r(5003)],'color','b')

xlim([t(5000) t(5100)]);
ylim([-0.0091 r(5000)]);
xlabel('time');
ylabel('rad/s');
title('Step response of yaw rate');


 
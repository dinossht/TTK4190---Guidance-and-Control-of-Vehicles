function pathplotter(x, y,  psi, tsamp, dec, tstart, tstop, track, WP)
% PATHPLOTTER draws the path of the ship MS Fartoystyring used in TTK4190 
% Guidance and Control Assignment 3, part 2.
%
% PATHPLOTTER outputs a single xy-plot of the MS Fartoystyring's trajectory 
% and, depending upon the input, either waypoints and path or the trajectory
% of the target. The MS Fartoystyring is yellow and the target green. MS
% Fartoystyring's trajectory is blue, and the waypoints/target trajectory
% red.
%
% x      is the ship's north position (in NED). x is a vector in R^T, T is 
%          number of samples
% y      is the ship's east position (in NED). y is a vector in R^T 
% psi    is the ship's yaw angle (in NED). psi is a vector in R^T 
% tsamp  is the sampling time
% dec>=1 is how much of the data should be used in the plot. dec should 
%          be a natural number. E.g. dec=2 reduces number of data points by
%          a factor of 2. Too low value of dec makes the plot illegible
% tstart is simulation start time
% tstop  is simulation stop time
% track  is a boolean value indicating whether or not the ship is following 
%          waypoints (Tasks 2.2-2.6) (track=0) or tracking a target (Task
%          2.7) (track=1).
%
% Input must be sampled with a fixed time step.
%
% You are free to modify the code as necessary.

close all
figure
hold on
psiTemp=atan2(WP(2,2)-WP(2,1),WP(1,2)-WP(1,1));
if track
    plot([WP(2,1), WP(2,2)+3*sin(psiTemp)*(tstop-tstart)], [WP(1,1), WP(1,2)+3*cos(psiTemp)*(tstop-tstart)], 'r')
else
    siz=size(WP);
    for ii=1:(siz(2)-1)
        plot([WP(2,ii), WP(2,ii+1)], [WP(1,ii), WP(1,ii+1)], 'r-x')
    end
end
plot(y, x)
L=300;
    

R = [cos(psiTemp) -sin(psiTemp); sin(psiTemp) cos(psiTemp)]; 
target = R*[L/2 .9*L/2 .5*L/2 -L/2 -L/2 .5*L/2 .9*L/2 L/2; 
              0 10 20 20 -20 -20 -10 0];
tnow=tstart;
for now=1:dec:length(x)
    if track
        %Target
        plot(WP(2,2)+3*sin(psiTemp)*(tnow-tstart)+target(2,:),WP(1,2)+3*cos(psiTemp)*(tnow-tstart)+target(1,:),'g');
        patch(WP(2,2)+3*sin(psiTemp)*(tnow-tstart)+target(2,:),WP(1,2)+3*cos(psiTemp)*(tnow-tstart)+target(1,:),'g');
    end
    
    %MS Fartoystyring
    tmpR=[cos(psi(now)) -sin(psi(now)); sin(psi(now)) cos(psi(now))];
    boat = tmpR*[L/2 .9*L/2 .5*L/2 -L/2 -L/2 .5*L/2 .9*L/2 L/2; 
              0 10 20 20 -20 -20 -10 0];
    plot(y(now)+boat(2,:),x(now)+boat(1,:),'y');
    patch(y(now)+boat(2,:),x(now)+boat(1,:),'y');
    
    tnow=tnow+tsamp*dec;
end
hold off
xlabel('East [m]')
ylabel('North [m]')
axis equal
grid on;

if track
    tim=tstart:tsamp:tstop;
    st = size(tim);
    sy = size(y);
    if(st(1) ~= sy(1))
        tim = tim';
    end
    
    dx = WP(1,2)+3*cos(psiTemp)*tim-x;
    dy = WP(2,2)+3*sin(psiTemp)*tim-y;
    
    figure
    plot(tim,sqrt(dx.^2+dy.^2));
    xlabel('time [s]')
    ylabel('distance [m]')
    grid on;
    title('Distance to target')
    
    figure
    hold on
    plot(tim, dx, 'r')
    plot(tim, dy)
    xlabel('time [s]')
    ylabel('distance [m]')
    title('Distance to target')
    legend('x', 'y')
    grid on;
    hold off
else
    sw = size(WP);
    alph = zeros(max(sw)-1, 1);
    for ii=1:length(alph)
        alph(ii) = atan2(WP(2,ii+1)-WP(2,ii), WP(1,ii+1)-WP(1,ii));
    end
    tim=tstart:tsamp:tstop;
    e = zeros(length(tim), 1);
    s = zeros(length(tim), 1);

    tmpE = zeros(size(alph));
    tmpS = zeros(size(alph));

    eps = 20;
    mind = 1;
    minds = zeros(size(tim));
    for ii=1:length(tim)
        for jj=1:length(tmpE)
            tmpS(jj) = (x(ii)-WP(1,jj))*cos(alph(jj))+(y(ii)-WP(2,jj))*sin(alph(jj));
            tmpS(jj) = sqrt((WP(1,jj)-WP(1,jj+1))^2+(WP(2,jj)-WP(2,jj+1))^2)-tmpS(jj);
            tmpE(jj) = -(x(ii)-WP(1,jj))*sin(alph(jj))+(y(ii)-WP(2,jj))*cos(alph(jj));
        end
        
        if tmpS(mind)<eps && mind<length(tmpE)
            mind = mind+1;
        end
        minds(ii) = mind;
        e(ii) = tmpE(mind);
        s(ii) = tmpS(mind);
    end    
    figure
    plot(tim, e)
    xlabel('time [s]')
    ylabel('distance [m]')
    title('Cross-track error')
    grid on;
end
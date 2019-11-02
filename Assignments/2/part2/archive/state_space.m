function [y, w] = state_space(u)
% Generate the output of the state space

    % Persistent variables to be kept between function calls
    persistent counter; % Variable for checking if it is the first iteration.
    persistent x;       % The state x
    
   % The next line is necessary for generating unique process noise for every simulation.
   % Otherwise, the noise will be equal every time you run the simulation,
   % even though it is white. This has to do with how Matlab generates
   % random numbers. 
   coder.extrinsic('randn','rng');
    
    h = 0.01;       % Time step of Euler integration
    delta_a_c = u;  % Aileron input
    
    if isempty(counter)
      % First iteration
        counter = 1;
           
        % Initialize state space
        x = zeros(5,1);
        w = zeros(5,1); 
        rng('shuffle'); % Necessary for generating unique process noise for every simulation.
    
    else
        % State-space matrices
        A = [-0.322, 0.052, 0.028, -1.12, 0.002;
             0, 0, 1, -0.001, 0;
             -10.6, 0, -2.87, 0.46, -0.65;
             6.87, 0, -0.04, -0.32, -0.02;
             0, 0, 0, 0, -7.5];
 
        B = [0; 0; 0; 0; 7.5;];
        
        E = eye(5);
        
        % Discrete version
        Ad = eye(5) + h*A;
        Bd = h*B;
        Ed = E;
        
        % The Qd matrix in the assignment is given directly in discrete-time here and
        % therefore, it is not necessary to discretize E.
        Qd = h*10^-6*[0.001, 0, 0, 0, 0;
                  0, 1, 0, 0, 0;
                  0, 0, 100, 0, 0;
                  0, 0, 0, 10, 0;
                  0, 0, 0, 0, 0];
          
        w = sqrt(Qd)*randn(5,1,1); % The standard deviation must be multiplied with random numbers with a zero-mean unit variance random number to generate process noise. 
        
        % Discrete-time state space update
        x = Ad*x + Bd * delta_a_c + Ed*w;
    end
    y = x(1:4); % Return every state except the actuator state.
end
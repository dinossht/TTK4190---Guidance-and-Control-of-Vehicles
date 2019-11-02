function x_est = kalman_filter(y, u)
    
    % Persistent variables to be kept between function calls
    persistent counter; % Variable for checking if it is the first iteration.
    persistent x_upd;       % The updated state x
    persistent P_upd;       % The updated error covariance
    persistent x_pred;      % The Predicted state x
    persistent P_pred;      % The Predicted error covariance
    % Add as well A,B etc. here
    
    h = 0.01;       % Time step (of Euler integration)
    
    if isempty(counter)
        % First iteration
        counter = 1;
           
        % Initialize conditions
        x_upd = zeros(4,1); %###############################################
        P_upd = eye(4); %###################################################
        
    end
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % State-space matrices %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    A = [-0.322,    0.052,  0.028,  -1.12   ;
         0,         0,      1,      -0.001  ;
         -10.6,     0,      -2.87,  0.46    ;
         6.87,      0,      -0.04,  -0.32   ;];

    B = [0.002; 0; -0.65; -0.02];
    C = [zeros(2), eye(2)];  
    E = eye(4);

    % Discrete version %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    [Ad, Bd] = c2d(A, B, h); % Ad = eye(4) + h*A;Bd = h*B;  
    Cd = C;
    Ed = E;

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Discrete-time Kalman filter algorithm %%%%%%%%%%%%%%%%%%%%%%%%%%%
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    % KF coviariance matrices %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    Q = [1      0       0       0   ;
         0      1       0       0   ;
         0      0       1       0   ;
         0      0       0       1   ;]; %###############################

    R = 1e-3 * [1      0   ;
                0      1   ;]; %###############################################

    % Predict %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    % Predicted (a priori) state estimate
    x_pred = Ad * x_upd + Bd * u;        
    % Predicted (a priori) error covariance
    P_pred = Ad * P_upd * Ad' + E * Q * E'; %######Should E be included?

    % Update %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    % Innovation or measurement pre-fit residual
    y_tilde = y - C * x_pred;
    % Innovation (or pre-fit residual) covariance
    S = C * P_pred * C' + R;
    % Optimal Kalman gain
    K = P_pred * C' * inv(S);

    % Updated (a posteriori) state estimate
    x_upd = x_pred + K * y_tilde;
    % Updated (a posteriori) estimate covariance
    P_upd = (eye(4) - K * C) * P_pred; 

    % Return output
    x_est = x_upd;  
end


function [ predictx, predicty, state, param ] = kalmanFilter( t, x, y, state, param, previous_t )
%UNTITLED Summary of this function goes here
%   Four dimensional state: position_x, position_y, velocity_x, velocity_y

    %% Place parameters like covarainces, etc. here:
    % P = eye(4)
    % R = eye(2)

    % Check if the first time running this function
    if previous_t<0
        state = [x, y, 0, 0];
        param.P = 0.1 * eye(4);
        predictx = x;
        predicty = y;
        return;
    end

    %% TODO: Add Kalman filter updates
    % As an example, here is a Naive estimate without a Kalman filter
    % You should replace this code
    dt = t - previous_t;
    
    % State Transistion Matrix:
    A = [1 0 dt 0;
         0 1 0 dt;
         0 0 1 0;
         0 0 0 1];
     
    % Observation Matrix:
    C = [1 0 0 0;
         0 1 0 0];
    
    % Calculate Measurements:
    % They are just the x and y values returned to us
    z = C * [x;y;0;0];
    % Measurement Model Noise Covariance:
    CovO = [0.01 0;
            0 0.01];
    
    R = CovO;
    % Tuning the System model:
    % As suggested in the assignment, if the value of a is kept very high,
    % the covariance become high, and the kalman filter follows the
    % observed path. This is because high uncertaininty causes less belief
    % in system model. This is good to test if the model is working fine or
    % not.
    
    % Coarse Test for System Covariance Matrix:
%     a = 10^6;
    
    % Once it passes the coarse test, we can consider a smaller value till
    % our prediction is close the what the kalman predicted.
    % Fine Test for System Covariance Matrix:
    a = dt/2;
    
    % System Model Noise Covariance: 
    CovM = [a*a 0 a 0;
            0 a*a  0 a;
            a 0 1 0;
            0 a 0 1];
        
    % Calculate State Covariance:
    P = A * param.P * A' + CovM;
    
    % Calculate Optimum Gain: K = P*C'*inv(R + C*P*c')
    K = (P * C')/(R + C * P * C');
    
    % Predict the values from the current states and the gain:
    predict = A * state' + K * (z - C * A * state');
    
    % Updated the State Covariance Matrix:
    param.P = P - K * C * P;
    
    % Update the State values:
    state = predict';
    
    % The values we have are for next time step, We want to extrapolate
    % data for 10 frames forward:
    predictx = predict(1) + predict(3)*(0.33);
    predicty = predict(2) + predict(4)*(0.33);
end

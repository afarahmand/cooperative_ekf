% Ashil Farahmand
% Recursive Kalman Filter

% Dimensions
% ------------------------------------------------------------------------
% F [n x n]
% x [n x 1]
% G [n x n]
% w [n x 1]
% z [m x n]
% H [m x n]
% e [m x 1]
% sigma [
% ------------------------------------------------------------------------
% Observation Eqn: z(k) = H(k)*x(k) + e(k)
% State Eqn: x(k) = F(k-1)x(k-1) + G(k-1)w(k-1)
% sigma = Cov of prediction state error
% R = Cov observation noise/delta
% ------------------------------------------------------------------------

clc
clear

iterations = 20;

% Sampling Period
T = 1;
rough = 1;                 % 0 = random noise, x = noise multiplied by value of rough alternates :/- each iteration
outliers = 0;               % 0 = random noise, 1 = outliers introduced into system

% DeclareVariables and set ICs
F = [1 T 0 0; 0 1 0 0; 0 0 1 T; 0 0 0 1];  % Transition matrix
x = [0; 0; 0; 0];                                       % State vector
x_hat = x;                                              % Estimate of x
G = 0;                                                     %
w = T*[randn; randn; randn; randn];  % System Error
W = 0;                                                    % Cov(w)

z = [0; 0];                                                % Observations
H = [1 0 0 0; 0 0 1 0];                             %
e = T*[randn; randn];                            % Observation Error

true_states = zeros(iterations, 4);
predicted_states = zeros(iterations, 4);
corrected_states = zeros(iterations, 4);

% Assign Initial Conditions
x(2) = 20;            % Sets initial particle velocities {x(2) is in x1 dimension, x(4) is in x2 dimension}
x(4) = 0;
x_hat = x;

true_states(1, 2) = x(2);                           % Assigns initial particle velocities
true_states(1, 4) = x(4);

predicted_states(1, 2) = true_states(2, 1);
predicted_states(1, 4) = true_states(4, 1);

sigma = I(4);
R = I(2);
W = I(4);
G = I(4);

% Begin to iterate (index 1 holds the initial conditions)
for count = 2:1:iterations    
    % Prediction
    corrected_states(count, 1:4) = x_hat';
    x_hat = F*x_hat;
    predicted_states(count, 1:4) = x_hat';
        
    % Get new observations and iterate
    x = F*x + G*w;
    z = H*x + e;
    w = T*[randn; randn; randn; randn];
    e = T*[randn; randn];
    
    % Outlier Insertion
    if (count == 3)
        w(4) = T*5;
    end
    
    % Correction
    sigma = F*sigma*F' + G*W*G';             % Cov of prediction state error
    K = sigma*H'*inv(H*sigma*H' + R);    % Kalman Gain
    x_hat = x_hat + K*(z - H*x_hat);          % Innovation
    
    % Record true_states of particle positions to plot later
    true_states(count, 1:4) = x';
    
    % Outlier Insertion
    if (count == 3) & (outliers == 1)
        %e(4) = T*20;
    end
    %w(4) = count*1;                % Acceleration in y direction
    
    if (rough ~= 1)
        if (count == 2*floor(count/2))         %Test for even number
            w(3) = -rough*T*abs(randn);
        else
            w(3) = rough*T*abs(randn);
        end
    end
    
end

% Plot True Values
hold off
plot(0, 0, '* blue');
hold on
tp = plot(true_states(1:iterations, 1), true_states(1:iterations, 3), '* blue');
plot(true_states(1:iterations, 1), true_states(1:iterations, 3), 'blue');
title('Particle Trajectory');
xlabel('x1');
ylabel('x2');

% Plot Predicted Values
pp = plot(predicted_states(1:iterations, 1), predicted_states(1:iterations, 3), 'O red');
plot(predicted_states(1:iterations, 1), predicted_states(1:iterations, 3), 'red');

% Plot Corrected Values
cp = plot(corrected_states(1:iterations, 1), corrected_states(1:iterations, 3), 'sq green');
plot(corrected_states(1:iterations, 1), corrected_states(1:iterations, 3), 'green');

legend([tp pp cp], 'True', 'Predicted', 'Corrected');
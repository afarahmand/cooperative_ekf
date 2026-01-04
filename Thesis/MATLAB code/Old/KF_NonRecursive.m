% Ashil Farahmand
% Non-Recursive Kalman Filter

% F [n x n]
% x [n x 1]
% G [n x n]
% w [n x 1]
% z [m x n]
% H [m x n]
% e [m x 1]
% -----------------------------------------------------------
% Observation Eqn: z(k) = H(k)*x(k) + e(k)
% State Eqn: x(k) = F(k-1)x(k-1) + G(k-1)w(k-1)
% sigma = Cov of prediction state error
% R = Cov observation noise
% -----------------------------------------------------------

clc
clear

iterations = 30;

T = 1;                         % Sampling Period
rough = 0;                 % 0 = random noise, x = noise multiplied by value of rough alternates :/- each iteration
outliers = 0;               % 0 = random noise, 1 = outliers introduced into system

% DeclareVariables and set ICs
F = [1 T 0 0; 
         0 1 0 0; 
         0 0 1 T; 
         0 0 0 1];                                             % Transition matrix
x = zeros(4, iterations);                            % State vector
x_hat_pred = zeros(4, iterations);          % Predicted estimate of x
x_hat_corr = zeros(4, iterations);            % Corrected estimate of x
pred_state_error = zeros(4, iterations);  % Prediction State Error (x - x_hat_pred)
G = 0;                                                        %
w = T*randn(4, iterations);                      % System Error
W = 0;                                                        % Cov(w)
sigma = zeros(4, 4, iterations);                 % Cov(pred_state_error)

z = zeros(4, iterations);                             % Observations
H = I(4);                                                     %
e = T*randn(4, iterations);                        % Observation Error

R = zeros(4, 4, iterations);                        % Cov(e)

% Assign Initial Conditions
x(2, 1) = 20;            % Sets initial particle velocities {x(2) is in x1 dimension, x(4) is in x2 dimension}
x(4, 1) = 0;
x_hat_pred(:, 1) = x(:, 1);
x_hat_corr(:, 1) = x_hat_pred(:, 1);

sigma(:, :, 1) = I(4);
R(:, :, 1) = I(4);
W = I(4);
G = I(4);

% Begin to iterate (index 1 holds the initial conditions)
for count = 2:iterations
    R(:, :, count) = I(4);
    
    % Prediction
    x_hat_pred(:, count) = F*x_hat_corr(:, count - 1);
    
    % Get new observations and iterate
    x(:, count) = F*x(:, count - 1) + G*w(:, count - 1);
    z(:, count) = H*x(:, count) + e(:, count);
    
    % Correction
    sigma(:, :, count) = F*sigma(:, :, count - 1)*F' + G*W*G';                                                         % Cov of prediction state error
    K = sigma(:, :, count)*H'*inv(H*sigma(:, :, count)*H' + R(:, :, 1));                                           % Kalman Gain
    x_hat_corr(:, count) = x_hat_pred(:, count) + K*(z(:, count) - H*x_hat_pred(:, count));      % Innovation
    
    % Outlier Insertion
    if (count == 3) & (outliers == 1)
        %e(4) = T*20;
    end
    
    % Creates fluctuating process noise
    if (rough ~= 0)
        if (count == 2*floor(count/2))         %Test for even number
            w(3, count) = -rough*T*abs(randn);
        else
            w(3, count) = rough*T*abs(randn);
        end
    end
end

% Begin batch processing using Generalized Linear Regression model
pred_state_error = x - x_hat_pred;

z_tild = [z; 
                x_hat_pred];
H_tild = [H; 
                  I(4)];
e_tild = [e; 
                pred_state_error];
R_tild = [R zeros(4, 4, iterations);
                 zeros(4, 4, iterations) sigma];

% Perform Cholesky Decomposition(R must be Hermitian)
for count = 1:iterations
    L = tril(R_tild(:, :, count));

    y = inv(L)*z_tild;
    A = inv(L)*H_tild;
    n = inv(L)*e_tild;
end

size(A)

% True Velocity in y direction
%figure(3);
%tv_y = plot(1:iterations, x(1:iterations, 4), '* black');
%plot(1:iterations, x(1:iterations, 4), 'black');

% Predicted Value in y direction
%pv_x = plot(1:iterations, x_hat_pred(1:iterations, 4), 'O green');
%plot(1:iterations, x_hat_pred(1:iterations, 4), 'green');
%title('Velocity in Y-Direction');
%xlabel('Iteration');
%ylabel('Velocity');
%legend([tv_y pv_y], 'True', 'Predicted');

% True Velocity in x direction
%figure(2);
%tv_x = plot(1:iterations, x(1:iterations, 2), '* black');
%plot(1:iterations, x(1:iterations, 2), 'black');

% Predicted Value in x direction
%pv_x = plot(1:iterations, x_hat_pred(1:iterations, 2), 'O green');
%plot(1:iterations, x_hat_pred(1:iterations, 2), 'green');
%title('Velocity in X-Direction');
%xlabel('Iteration');
%ylabel('Velocity');
%legend([tv_x pv_x], 'True', 'Predicted');

% --------------------------------------------------------------------
% Plots
%figure(1)
hold off
plot(0, 0, '* blue');
hold on

% True Position Values
tp = plot(T*x(1, :), x(3, :), 'sq blue');
plot(T*x(1, :), x(3, :), 'blue');

% Predicted Position Values
pp = plot(T*x_hat_pred(1, :), x_hat_pred(3, :), 'O red');
plot(T*x_hat_pred(1, :), x_hat_pred(3, :), 'red');

% Corrected Position Values
cp = plot(T*x_hat_corr(1, :), x_hat_corr(3, :), 'O green');
plot(T*x_hat_corr(1, :), x_hat_corr(3, :), 'green');

title('Particle Position');
xlabel('x');
ylabel('y');
legend([tp pp, cp], 'True', 'Predicted', 'Corrected');
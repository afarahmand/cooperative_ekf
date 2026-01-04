clear
clc

n = 9;

% WLS-estimator that fits 20 data points

%z = [1.0901;    2.1410;    3.1938;    4.2460;    5.2976;    6.3487;    7.3996;    8.4484];
%H = [10 1; 20 1; 30 1; 40 1; 50 1; 60 1; 70 1; 80 1];
z = (10:10:90)';
z(9) = 85;
H = [1.0901 1; 2.1410 1; 3.1938 1; 4.2460 1; 5.2976 1; 6.3487 1; 7.3996 1; 8.4484 1; 8.9741 1];

% Calculate x
x_hat = inv(H'*H)*H'*z
 
% Plot outputs
t = 1:.1:H(n);

figure(1)
subplot(2, 1, 1);
hold on; grid;
actual = plot(H(:, 1), z, '* blue');
estimate = plot(t, t*x_hat(1) + x_hat(2), 'red');
title('Brake Angle wrt Deceleration');
xlabel('Deceleration [m/s^2]');
ylabel('\beta [degrees]');
legend([actual estimate], 'Actual', 'Estimated', 'Location', 'Best');

subplot(2, 1, 2);
hold on; grid;
stem(H(:, 1), 100*(z - H*x_hat)./z)
title('Percent Estimation Error');
xlabel('Deceleration [m/s^2]');
ylabel('% Error [degrees]');
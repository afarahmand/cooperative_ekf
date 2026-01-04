clc
clear
format long

output = [1; 1; 1];

points_5 = [50, .82;
                      45, .95];
points_4 = [50, 1.85;
                      45, 2.00;
                      40, 2.1;
                      35, 2.21;
                      30, 2.3;
                      25, 2.38];
points_3 = [30, 3.79;
                      25, 3.87;
                      20, 4];
                  
x_5 = (45:5:50)';
C = -.026; b = 2.12; y_5 = C*x_5 + b;
x_3 = (20:5:30)';
C = -.021; b = 4.395; y_3 = C*x_3 + b;
x_4 = (25:5:50)';
C = -.0212; b = 2.945; y_4 = C*x_4 + b;

if(output(1) == 1)
    figure(1)
    hold on
    act = plot(points_3(:, 1), points_3(:, 2), 'blue O');     % Actual Points
    est = plot(x_3, y_3(:, 1), 'red');                                             % Estimate
    error = plot(x_3, y_3(:, 1) - points_3(:, 2), 'green O');                 % Error
    plot([20 30], [0 0], 'black')
    %error = act;
    legend([act est error], 'Actual', 'Estimated', 'Error', 'Location', 'Best')
end

if(output(2) == 1)
    figure(2)
    hold on
    act = plot(points_4(:, 1), points_4(:, 2), 'blue O');     % Actual Points
    est = plot(x_4, y_4(:, 1), 'red');                                             % Estimate
    %error = plot(x, y(:, 1) - points_4(:, 2), 'green O');                 % Error
    %plot([25 50], [0 0], 'black')
    error = act;
    legend([act est error], 'Actual', 'Estimated', 'Error', 'Location', 'Best')
end

if(output(3) == 1)
    figure(3)
    hold on
    act = plot(points_5(:, 1), points_5(:, 2), 'blue O');     % Actual Points
    est = plot(x_5, y_5(:, 1), 'red');                                             % Estimate
    error = plot(x_5, y_5(:, 1) - points_5(:, 2), 'green O');                 % Error
    plot([45 50], [0 0], 'black')
    %error = act;
    legend([act est error], 'Actual', 'Estimated', 'Error', 'Location', 'Best')
end
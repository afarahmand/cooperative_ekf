% Comparison b/w Runge Kutta, Euler and actual solutions

clc
clear

step_size = 1;
iterations = 10;

% Actual
x = 0:.001:iterations;
Actual = x.^2;

% Approximations
RK4 = zeros(iterations/step_size, 1);
Euler = zeros(iterations/step_size, 1);
k = zeros(4, 1);

% Set ICs
Euler(1) = 0;
RK4(1) = 0;

for i = step_size:step_size:iterations
    Euler(i/step_size + 1) =  Euler(i/step_size) + 2*(i - step_size)*step_size;
    
    j = i - step_size;
    k(1) = 2*(j);
    k(2) = 2*(j + step_size/2);
    k(3) = 2*(j + step_size/2);
    k(4) = 2*(j + step_size);
    
    RK4(i/step_size + 1) = RK4(i/step_size) + (step_size/6)*(k(1) + 2*k(2) + 2*k(3) + k(4));
end    

hold off
i = 0:step_size:iterations;
a = plot(x, Actual, 'blue');
hold on
e = plot(i, Euler, 'red');
r = plot(i, RK4, 'green');
legend([a e r], 'Actual', 'Euler', '4th Order Runge-Kutta');
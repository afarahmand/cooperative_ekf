% Ashil Farahmand
% Recursive Iterated EKF using Modified UCB Engine Model

% Goals
% - calculate time to cross
% ------------------------------------------------------------------------
% Issues with current implementation
% - Assume instantaneous pressure on controls
% - Doesn't down weight following cars
% - Only works with one type of car(internal characteristics of cars are identical) this constraint can easily be relaxed
% - Must try different optimization orders manually until optimal found

tic; clc; clear; format long;

KF_iterations = 5*10^4;                % Number of iterations for KF
integration_iterations = 1;              % Number of integrations per each KF_iteration
T = 1*10^-3;  num_of_cars = 1;

auto_transmission = logical(1);                      % 0 = (diagnostic)Manual, 1 = Automatic
noise = 0;                                                          % 0 = (diagnostic)None, 1 = White, 2 = Colored
rough = logical(0);                                            % 0 = random noise, x = noise multiplied by value of rough alternates :/- each iteration
run_optimization = logical([0; 0; 0; 0]);         % 0 = (diagnostic) Do not change inputs, 1 = change inputs according to cost function
show_optimization = 0;                                 % 0 = (diagnostic) Do not plot optimization, 1 = Plot Optimization
max_corrections = 1*10^4;                             % Prevents infinite loop in correction
e_correction = 10^-6;                                       % Allowable Correction Error

mode_output = [0, 0, 0, 0, 0,    0, 0, 0, 0, 0,    0, 1, 0, 0];
%mode_output = ones(14);

% Output Modes
% -----------------------------------------------------
% Index   |   Property
% 1              throttle angle
% 2                brake angle
% 3           steering direction
% 4            position (x2 vs. x1)
% 5            position (x vs. t)
% 6                  v_mph
% 7                  v_m/s
% 8          distance b/w cars
% 9                     ma
% 10                   we
% 11                   Tb
% 12                  gear
% 13     phase plot (ma vs. we)
% 14        corrector iterations

w = zeros(5*num_of_cars, 1); e = zeros(5*num_of_cars, 1); z = zeros(5*num_of_cars, 1); Fx = zeros(5*num_of_cars, 5*num_of_cars);

nominal_controls = zeros(3*num_of_cars);                                     % Control Vector during one KF iteration
predictions = zeros(5*num_of_cars, integration_iterations + 1);   % Holds the predictions during integration
corrections = zeros(5*num_of_cars, 2);                                             % Holds the corrected states when iterating the correction step (holds previous and next corrections)
correction_iterations = ones(KF_iterations, 1);                                 % Holds the number of iterations the corrector performs/KF iteration

predicted_states = zeros(5*num_of_cars, KF_iterations); corrected_states = zeros(5*num_of_cars, KF_iterations); gear_ratios = zeros(num_of_cars, KF_iterations);
predicted_opt_states = zeros(4*num_of_cars, KF_iterations);

% Topology Parameters
speed_limits = [22; 50; 10];            % Speed limits (left turn, straight, right turn)
lane_width = 2; line_separation = .1;

% Used by cost function
iteration_to_start_optimization = 10;                                % KF_iteration to start optimization
safe_distance = [2; 2; 2; 2];                                                    % Safe distance to maintain around each respective vehicle
safety_margin = .5;                                                                 % Additional distance to maintain between vehicles
direction = [0; 0; 0; 0];                                                           % 1 - Left turn, 2 - Straight, 3 - Right turn (Set in vehicle ICs)
order = zeros(num_of_cars, 1);                                           % Order of cars going through the intersection
steady_speed_t_angles = [6.91049; 11.77215; 3.52779];    % Throttle angles to maintain a steady speed at the speed limit
acc_profile = [0.4624; 0.2728; 0.1724; 0.09438; .08311];     % Acceleration values for different velocity ranges
braking_distances = [0; 0];                                                    % Distance from intersection to begin braking in preparation for turn (calculated below)
time_optimal = zeros(num_of_cars, 1);                             % The amount of time needed to traverse the intersection when no other vehicles are present
time_to_cross = zeros(num_of_cars, 1);                            % The amount of time needed to traverse the intersection for a given order
target_speed = speed_limits(2);                                           % Used in optimization to signal a necessary acceleration or brake
brake_decel_rates = [-2.037; -4];                                           % Steady state deceleration at corresponding brake angle
brake_decel_angles = [19; 37.6966];                                     % Brake angles to achieve corresponding brake decel rates
steering_angle_increment = [ (90*T*4*speed_limits(1)*1609.344)/(3*pi*lane_width*3600); (90*T*4*speed_limits(3)*1609.344)/(pi*lane_width*3600) ];

% Assign Initial Conditions
K = I(5*num_of_cars); G = I(5*num_of_cars); Q = I(5*num_of_cars);
q = [.01; .01; 10^-10; .01; 100]; % Covariances of process and observation noise
for i = 1:num_of_cars    Q(5*i - 4, 5*i - 4) =  q(1);    Q(5*i - 3, 5*i - 3) = q(2);    Q(5*i - 2, 5*i - 2) = q(3);    Q(5*i - 1, 5*i - 1) = q(4);    Q(5*i - 0, 5*i - 0) = q(5); end
sigma = Q; Rk = Q;

delta_t = T/integration_iterations;                           % One KF_interval time is integration_iterations*delta_t = T
k = zeros(4, 5*num_of_cars);                                    % RK4 variables; one col/SV
RK_states = zeros(5*num_of_cars, 1);                     % state vector
u = zeros(3*num_of_cars, KF_iterations);               % control vector
Jtg_current = zeros(num_of_cars, 1);
MAX = 0.684; R = 8.3145; Tm = 333; Mair = .02897; Vm = .00447; Patm = 101325; c_PR = (R*Tm)/(Patm*Mair*Vm);
Ve = .0049; c_1 = Ve/(4*pi*Vm);
normal_shift_speeds = [0; 7; 15; 25; 45];      % Speed to shift
fast_shift_speeds = [11; 20; 35; 55];              % Speeds to shift under max acceleration
Rg = [0.4167; 0.6817; 1; 1.4993; 2.3058];
c_2 = 1018686; Ca = 0.53384; h = 0.33; Fr = 167.27; Je = 0.2630; Jtg = [0.08202; 0.07592; 0.11388; 0.13150]; Jw = 5.13; M = 2148; Kb = 300; Tau_bv = 0.1;

% Pre-Processing

% Assign braking distances
x = ( (speed_limits(2) - speed_limits(1:2:3))*(1609.344/3600))/(-brake_decel_rates(2));
braking_distances = -.5*brake_decel_rates(2)*x.^2 + speed_limits(1:2:3)*(1609.344/3600).*x + lane_width

% Car 1
car = 1;
direction(car) = 2;
gear_ratios(car, 1) = Rg(2);
predictions(5*car - 4, 1) = -200;
predictions(5*car - 3, 1) = -lane_width/2;
predictions(5*car - 2, 1) = .00111331477934;
predictions(5*car - 1, 1) = 29.61976701882556;
%predictions(5*car - 1, 1) = 45 * (1609.344/3600) *(1/(h*gear_ratios(car, 1)));
u(3*car - 2, 1:KF_iterations) = 4.5623;
u(3*car - 0, 1:KF_iterations) = 0;

% Car 2
car = 2;
if(num_of_cars > 1)
direction(car) = 2;
gear_ratios(car, 1) = Rg(5);
predictions(5*car - 4, 1) = -lane_width/2;
predictions(5*car - 3, 1) = 200;
predictions(5*car - 2, 1) = .00033558;
predictions(5*car - 1, 1) = 45 * (1609.344/3600) * (1/(h*gear_ratios(car, 1)) );
u(3*car - 2, 1:iteration_to_start_optimization) = 85;
u(3*car - 0, 1:KF_iterations) = 270;
end

% Car 3
car = 3;
if(num_of_cars > 2)
direction(car) = 2;
gear_ratios(car, 1) = Rg(1);
predictions(5*car - 4, 1) = -lane_width/2;
predictions(5*car - 3, 1) = 16 + lane_width;
predictions(5*car - 2, 1) = .00033558;
predictions(5*car - 1, 1) = 2 * (1/(h*gear_ratios(car, 1)) );
u(3*car - 2, 1:iteration_to_start_optimization) = 85;
u(3*car - 0, 1:KF_iterations) = 270;
end

% Car 4
car = 4;
if(num_of_cars > 3)
direction(car) = 2;
gear_ratios(car, 1) = Rg(1);
predictions(5*car - 4, 1) = -lane_width/2;
predictions(5*car - 3, 1) = 30 + lane_width;
predictions(5*car - 2, 1) = .00033558;
predictions(5*car - 1, 1) = 2 * (1/(h*gear_ratios(car, 1)) );
u(3*car - 2, 1:iteration_to_start_optimization) = 85;
u(3*car - 0, 1:KF_iterations) = 270;
end

predicted_states(:, 1) = predictions(:, 1); corrected_states(:, 1) = predictions(:, 1);

% Begin to iterate (index 1 holds the ICs)
for KF_interval = 1:KF_iterations    
    
    % Run Optimization
    if((sum(run_optimization) > 0) & (KF_interval > iteration_to_start_optimization))       % At least one car needs to be optimized then true
        ETA = zeros(num_of_cars, 1);
        
        % Find ETA of each car based on current states (Cost Function)
        for i = 1:num_of_cars
            d = max( abs(corrected_states(5*i - 4, KF_interval)), abs(corrected_states(5*i - 3, KF_interval)) ) - lane_width - safe_distance(i);
            v = corrected_states(5*i - 1, KF_interval)*h*gear_ratios(i, KF_interval);
            
            % Too close for changing plans (ETA < 0 then no change of control vars)
            if( (direction ~= 2) & (d < braking_distances(ceil(direction(i)*.6))) )                ETA(i) = -i;                run_optimization(i) = 0;
            elseif(direction(i) == 1)
                
                if( (v + 0.1) > (speed_limits(2)*1609.344/3600) )         % At speed limit
                    d = max( abs(corrected_states(5*i - 4, KF_interval)), abs(corrected_states(5*i - 3, KF_interval)) ) - safe_distance(i) - braking_distances(1);
                    k(1) = d/(corrected_states(5*i - 1, KF_interval)*gear_ratios(i, KF_interval)*h);
                    k(2) = 0;
                elseif( v > (45*1609.344/3600) )                                      % High 4th Gear
                    % Find position of car after accelerating to speed limit
                    k(1) = (speed_limits(2)*1609.344/3600 - v)/acc_profile(5);
                    x = .5*acc_profile(5)*k(1)^2 + v*k(1) - max( abs(corrected_states(5*i - 4, KF_interval)), abs(corrected_states(5*i - 3, KF_interval)) );
                    
                    % Reach Speed Limit
                    if(x < -braking_distances(1))                        k(2) = (abs(x) - braking_distances(1))/(speed_limits(2)*1609.344/3600);
                    elseif(x > -braking_distances(1))               % Do not reach speed limit before line
                        k(1) = ( sqrt(v^2 + 2*acc_profile(5)*(max( abs(corrected_states(5*i - 4, KF_interval)), abs(corrected_states(5*i - 3, KF_interval)) )- braking_distances(1)) ) - v )/acc_profile(5);
                        % Find speed of vehicle @ lane_width, then average
                        % over whole time to find v
                    end
                
                % -------------------------------------------------------------------------------------    
                elseif( v > (35*1609.344/3600) )                                      % Lo 4th Gear
                    k(2) = (45 - 35)*(1609.344/3600)/acc_profile(4);
                    x = .5*acc_profile(4)*k(2)^2 + v*k(2) - max( abs(corrected_states(5*i - 4, KF_interval)), abs(corrected_states(5*i - 3, KF_interval)) );
                    
                    % Reach High 4th Gear
                    if(x < -braking_distances(1))
                        k(2) = (speed_limits(2) - 45)*(1609.344/3600)/acc_profile(5);
                        x  = .5*acc_profile(5)*k(2)^2 + 45*(1609.344/3600)*k(2) + x;
                        
                        % Reach Speed Limit
                        if(x < -braking_distances(1))                            k(1) = (abs(x) - braking_distances(1))/(speed_limits(2)*1609.344/3600);
                        elseif(x > -braking_distances(1))               % Do not reach speed limit before line
                            k(1) = (sqrt((45*1609.344/3600)^2 + 2*acc_profile(5)*(lane_width + abs(x))) - 45*1609.344/3600)/acc_profile(5);
                        % Find speed of vehicle @ lane_width, then average
                        % over whole time to find v
                        end
                        
                    elseif(x > -braking_distances(1))               % Do not reach High 4th Gear
                        k(1) = ( sqrt(v^2 + 2*acc_profile(4)*(lane_width + max( abs(corrected_states(5*i - 4, KF_interval)), abs(corrected_states(5*i - 3, KF_interval)) ))) - v )/acc_profile(4);
                        % Find speed of vehicle @ lane_width, then average
                        % over whole time to find v
                    end
                else
                    if(d > braking_distances(1))
                        k(1) = (d - braking_distances(1))/v;
                    end
                end
                
                k(3) = (braking_distances(1) - lane_width)/((speed_limits(2)/2 - speed_limits(1)/2)*1609.344/3600);
                k(4) = (3*lane_width*pi/4)/(speed_limits(1)*1609.344/3600);
                ETA(i) = k(1) + k(2) + k(3) + k(4);
                
            elseif(direction(i) == 3)
                
                if( (v + 0.1) > (speed_limits(2)*1609.344/3600) )         % At speed limit
                    d = max( abs(corrected_states(5*i - 4, KF_interval)), abs(corrected_states(5*i - 3, KF_interval)) ) - safe_distance(i) - braking_distances(2);
                    k(1) = d/(corrected_states(5*i - 1, KF_interval)*gear_ratios(i, KF_interval)*h);
                    k(2) = 0;
                elseif( v > (45*1609.344/3600) )                                      % High 4th Gear
                    % Find position of car after accelerating to speed limit
                    k(1) = (speed_limits(2)*1609.344/3600 - v)/acc_profile(5);
                    x = .5*acc_profile(5)*k(1)^2 + v*k(1) - max( abs(corrected_states(5*i - 4, KF_interval)), abs(corrected_states(5*i - 3, KF_interval)) );
                    
                    % Reach Speed Limit
                    if(x < -braking_distances(2))                        k(2) = (abs(x) - braking_distances(2))/(speed_limits(2)*1609.344/3600);
                    elseif(x > -braking_distances(2))               % Do not reach speed limit before line
                        k(1) = ( sqrt(v^2 + 2*acc_profile(5)*(max( abs(corrected_states(5*i - 4, KF_interval)), abs(corrected_states(5*i - 3, KF_interval)) )- braking_distances(2)) ) - v )/acc_profile(5);
                        % Find speed of vehicle @ lane_width, then average
                        % over whole time to find v
                    end
                
                
                % -------------------------------------------------------------------------------------    
                elseif( v > (35*1609.344/3600) )                                      % Lo 4th Gear
                    k(2) = (45 - 35)*(1609.344/3600)/acc_profile(4);
                    x = .5*acc_profile(4)*k(2)^2 + v*k(2) - max( abs(corrected_states(5*i - 4, KF_interval)), abs(corrected_states(5*i - 3, KF_interval)) );
                    
                    % Reach High 4th Gear
                    if(x < -braking_distances(2))
                        k(2) = (speed_limits(2) - 45)*(1609.344/3600)/acc_profile(5);
                        x  = .5*acc_profile(5)*k(2)^2 + 45*(1609.344/3600)*k(2) + x;
                        
                        % Reach Speed Limit
                        if(x < -braking_distances(2))                            k(1) = (abs(x) - braking_distances(2))/(speed_limits(2)*1609.344/3600);
                        elseif(x > -braking_distances(2))               % Do not reach speed limit before line
                            k(1) = (sqrt((45*1609.344/3600)^2 + 2*acc_profile(5)*(lane_width + abs(x))) - 45*1609.344/3600)/acc_profile(5);
                        % Find speed of vehicle @ lane_width, then average
                        % over whole time to find v
                        end
                        
                    elseif(x > -braking_distances(2))               % Do not reach High 4th Gear
                        k(1) = ( sqrt(v^2 + 2*acc_profile(4)*(lane_width + max( abs(corrected_states(5*i - 4, KF_interval)), abs(corrected_states(5*i - 3, KF_interval)) ))) - v )/acc_profile(4);
                        % Find speed of vehicle @ lane_width, then average
                        % over whole time to find v
                    end
                else
                    if(d > braking_distances(2))
                        k(1) = (d - braking_distances(2))/v;
                    end
                end
                
                k(3) = (braking_distances(2) - lane_width)/((speed_limits(2)/2 - speed_limits(3)/2)*1609.344/3600);
                k(4) = (lane_width*pi/4)/(speed_limits(3)*1609.344/3600);
                ETA(i) = k(1) + k(2) + k(3) + k(4);
            else
                
                if( (v + 0.1) > (speed_limits(2)*1609.344/3600) )         % At speed limit
                    d = max( abs(corrected_states(5*i - 4, KF_interval)), abs(corrected_states(5*i - 3, KF_interval)) ) - lane_width - safe_distance(i) + 2*lane_width;
                    ETA(i) = d/(corrected_states(5*i - 1, KF_interval)*gear_ratios(i, KF_interval)*h);
                
                elseif( v > (45*1609.344/3600) )                                      % High 4th Gear
                    % Find position of car after accelerating to speed limit
                    k(1) = (speed_limits(2)*1609.344/3600 - v)/acc_profile(5);
                    x = .5*acc_profile(5)*k(1)^2 + v*k(1) - max( abs(corrected_states(5*i - 4, KF_interval)), abs(corrected_states(5*i - 3, KF_interval)) );
                    
                    % Reach Speed Limit
                    if(x < lane_width)                        ETA(i) = k(1) + (abs(x) + lane_width)/(speed_limits(2)*1609.344/3600);
                    elseif(x > lane_width)               % Do not reach speed limit before line
                        ETA(i) = ( sqrt(v^2 + 2*acc_profile(5)*(lane_width + max( abs(corrected_states(5*i - 4, KF_interval)), abs(corrected_states(5*i - 3, KF_interval)) ))) - v )/acc_profile(5);
                        % Find speed of vehicle @ lane_width, then average
                        % over whole time to find v
                    end
                
                % -------------------------------------------------------------------------------------    
                elseif( v > (35*1609.344/3600) )                                      % Lo 4th Gear
                    k(2) = (45 - 35)*(1609.344/3600)/acc_profile(4);
                    x = .5*acc_profile(4)*k(2)^2 + v*k(2) - max( abs(corrected_states(5*i - 4, KF_interval)), abs(corrected_states(5*i - 3, KF_interval)) );
                    
                    % Reach High 4th Gear
                    if(x < lane_width)
                        k(1) = (speed_limits(2) - 45)*(1609.344/3600)/acc_profile(5);
                        x  = .5*acc_profile(5)*k(1)^2 + (45*1609.344/3600)*k(1) + x;
                        
                        % Reach Speed Limit
                        if(x < lane_width)                            ETA(i) = k(1) + k(2) + (abs(x) + lane_width)/(speed_limits(2)*1609.344/3600);
                        elseif(x > lane_width)               % Do not reach speed limit before line
                            ETA(i) = k(2) + (sqrt((45*1609.344/3600)^2 + 2*acc_profile(5)*(lane_width + abs(x))) - 45*1609.344/3600)/acc_profile(5);
                        % Find speed of vehicle @ lane_width, then average
                        % over whole time to find v
                        end
                        
                    elseif(x > lane_width)               % Do not reach High 4th Gear
                        ETA(i) = ( sqrt(v^2 + 2*acc_profile(4)*(lane_width + max( abs(corrected_states(5*i - 4, KF_interval)), abs(corrected_states(5*i - 3, KF_interval)) ))) - v )/acc_profile(4);
                        % Find speed of vehicle @ lane_width, then average
                        % over whole time to find v
                    end
                else
                    ETA(i) = (d + lane_width)/v;
                end
                    
            end
        end
        
        toc
        
        % Sort the ETAs in ascending optimization order (duplicateETAs yield error)
        ETA
        order = sort(ETA)
        for i = 1:num_of_cars            order(i) = find(order(i) == ETA);        end
                
        % Perform Optimization (assume only 1 car first)
        x = [0; 0];        v = 0;        a = 0;        k = zeros(1, 2);
        
        for i = 1:num_of_cars
            delay = 0;                                  % Time delay to avoid collision
            collision_points = 0;               % Indices where a collision will occur with current trajectory plan
            while(run_optimization(order(i)) == 1)
                x = [corrected_states(5*order(i) - 4, KF_interval); corrected_states(5*order(i) - 3, KF_interval)];
                v = corrected_states(5*order(i) - 1, KF_interval)*h*gear_ratios(order(i), KF_interval);
                v_mph = v*3600/1609.344;                target_speed = speed_limits(2)*1609.344/3600;                a = -brake_decel_rates(1)/100;                k(2) = 1;
                source = find(max(abs(x)) == abs(x));                sink = find(min(abs(x)) == abs(x));
                u(3*order(i) - 2, KF_interval:KF_iterations) = 0;                u(3*order(i) - 1, KF_interval:KF_iterations) = 0;                u(3*order(i) - 0, KF_interval:KF_iterations) = u(3*order(i) - 0, 1);
                
                if( (direction(order(i)) == 1) || (direction(order(i)) == 3) )
                    for j = KF_interval:KF_iterations
                        if((sum(size(collision_points)) - 2) > 0) % If original trajectory is invalid, delay some then proceed normally
                            a = a*exp(-T/Tau_bv) + brake_decel_rates(1)/100;                            u(3*order(i) - 1, j) = brake_decel_angles(1);                            k(1) = a;
                            
                            if(v_mph < (speed_limits(2) - delay) )                                collision_points = 1;                            end
                        elseif( abs(x(source)) > braking_distances(ceil(direction(order(i))*.6)) )   % Have not met decel area yet
                            u(3*order(i) - 2, j) = 85;
                            
                            if(collision_points == 1)                                k(1) = k(1)*exp(-T/Tau_bv);
                            else                                k(1) = 0;                            end
                            
                            if(v > target_speed)                                                         a = 0;                                u(3*order(i) - 2, j) = steady_speed_t_angles(2);
                            elseif(v_mph > 47)                                                          a = k(1) + acc_profile(5);
                            elseif(v_mph > fast_shift_speeds(3))                            a = k(1) + acc_profile(4);
                            elseif(v_mph > fast_shift_speeds(2))                            a = k(1) + acc_profile(3);
                            elseif(v_mph > fast_shift_speeds(1))                            a = k(1) + acc_profile(2);
                            else                                                                                     a = k(1) + acc_profile(1);                            end
                        elseif( (abs(x(source)) > lane_width) && (v < target_speed - 0.391) )      % In decel area and below target speed
                            u(3*order(i) - 2, j) = 85;                            target_speed = brake_decel_rates(2)*T + target_speed;                            k(1) = k(1)*exp(-T/Tau_bv);
                            
                            if(v_mph > 47)                                                                a = k(1) + acc_profile(5);
                            elseif(v_mph > fast_shift_speeds(3))                            a = k(1) + acc_profile(4);
                            elseif(v_mph > fast_shift_speeds(2))                            a = k(1) + acc_profile(3);
                            elseif(v_mph > fast_shift_speeds(1))                            a = k(1) + acc_profile(2);
                            else                                                                                     a = k(1) + acc_profile(1);                            end                            
                        elseif( (abs(x(source)) > lane_width) && (v >= target_speed - 0.391) )      % In decel area and at target speed
                            if(k(1) > 0)                                a = -brake_decel_rates(2)/100 + k(1)/100;                                k(1) = -1;                                '*',                            end
                            
                            a = a*exp(-T/Tau_bv) + brake_decel_rates(2)/100;                            u(3*order(i) - 1, j) = brake_decel_angles(2);
                            target_speed = a*T + target_speed;                            k(3) = 1;
                        elseif( (abs(x(source)) <= lane_width) && (abs(x(sink)) <= lane_width) )   % Begin turning manuever(inside Isection)
                            if(k(3) == 1)
                                steering_angle_increment = [ (90*T*4*(v_mph - 0.5)*1609.344)/(3*pi*lane_width*3600); (90*T*4*(v_mph - 0.5)*1609.344)/(pi*lane_width*3600) ];
                                k(3) = -1;
                            end
                            
                            k(1) = 0;
                            a = a*exp(-T/Tau_bv);                            target_speed = speed_limits(2)*1609.344/3600;                            
                            u(3*order(i) - 2, j) = steady_speed_t_angles(direction(order(i)));
                            u(3*order(i), j) = u(3*order(i), j - 1) - (direction(order(i)) - 2)*steering_angle_increment( ceil(direction(order(i))*.6) );
                        elseif(abs(x(sink)) > lane_width)                                                                        % Out of intersection moving away
                            u(3*order(i) - 2, j) = 85;                            u(3*order(i) - 0, j) = u(3*order(i), 1) - (direction(order(i)) - 2)*90;
                            
                            % Assigns time to cross
                            if(k(1) == 0)
                                k(1) = -2;
                                time_to_cross(order(i)) = (j - KF_interval)*T;                                
                                
                                % Assign costs of this iteration's trajectory
                                if(collision_points == 0)                    time_optimal(order(i)) = time_to_cross(order(i));                end
                            end
                            
                            if(v > target_speed)                                                         a = 0;                                u(3*order(i) - 2, j) = steady_speed_t_angles(2);
                            elseif(v_mph > 47)                                                          a = acc_profile(5);
                            elseif(v_mph > fast_shift_speeds(3))                            a = acc_profile(4);
                            elseif(v_mph > fast_shift_speeds(2))                            a = acc_profile(3);
                            elseif(v_mph > fast_shift_speeds(1))                            a = acc_profile(2);
                            else                                                                                     a = acc_profile(1);                            end
                        end
                        
                        x(1) = (.5*a*T^2 + v*T)*cos(u(3*order(i), j)*pi/180) + x(1);
                        x(2) = (.5*a*T^2 + v*T)*sin(u(3*order(i), j)*pi/180) + x(2);
                        v = a*T + v;                        v_mph = v*3600/1609.344;
                        if( (a < 0) & (v < 0) )                            v = 0;                            v_mph = 0;                        end
                        predicted_opt_states((4*order(i) - 3:4*order(i)), j) = [x(1); x(2); v_mph; a];
                    end
                else      % If not turning, then go straight
                    k(3) = 0;
                    for j = KF_interval:KF_iterations                        
                        if((sum(size(collision_points)) - 2) > 0)     % If original trajectory is invalid, delay some then proceed normally
                            a = a*exp(-T/Tau_bv) + brake_decel_rates(1)/100;                            u(3*order(i) - 1, j) = brake_decel_angles(1);                            k(1) = a;
                            
                            if(v_mph < (speed_limits(2) - delay))                                collision_points = 1;                            end
                        elseif( (k(3) == 0) & (abs(x(source)) > lane_width) & (abs(corrected_states(5*order(i) - 2 - sink, KF_interval) - x(source)) > abs(corrected_states(5*order(i) - 2 - sink, KF_interval))) )
                            k(3) = -1;
                            time_to_cross(order(i)) = (j - KF_interval)*T;
                            
                            % Assign costs of this iteration's trajectory
                            if(collision_points == 0)                    time_optimal(order(i)) = time_to_cross(order(i));                end
                        else
                            u(3*order(i) - 2, j) = 85;
                            
                            if(collision_points == 1)                                k(1) = k(1)*exp(-T/Tau_bv);
                            else                                k(1) = 0;                            end
                            
                            if(v > target_speed)                                                         a = 0;                                u(3*order(i) - 2, j) = steady_speed_t_angles(2);
                            elseif(v_mph > 47)                                                          a = k(1) + acc_profile(5);
                            elseif(v_mph > fast_shift_speeds(3))                            a = k(1) + acc_profile(4);
                            elseif(v_mph > fast_shift_speeds(2))                            a = k(1) + acc_profile(3);
                            elseif(v_mph > fast_shift_speeds(1))                            a = k(1) + acc_profile(2);
                            else                                                                                     a = k(1) + acc_profile(1);                            end
                        end
                        
                        x(1) = (.5*a*T^2 + v*T)*cos(u(3*order(i), j)*pi/180) + x(1);
                        x(2) = (.5*a*T^2 + v*T)*sin(u(3*order(i), j)*pi/180) + x(2);
                        v = a*T + v;                        v_mph = v*3600/1609.344;                        
                        if( (a < 0) & (v < 0) )                            v = 0;                            v_mph = 0;                        end                        
                        predicted_opt_states((4*order(i) - 3:4*order(i)), j) = [x(1); x(2); v_mph; a];
                    end
                end
                               
                
                % Compare trajectory plan to plans of all preceding cars
                if(i > 1)
                    for car = 1:(i - 1)
                        x = sqrt( (predicted_opt_states(4*order(i) - 3, 1:KF_iterations) - predicted_opt_states(4*order(car) - 3, 1:KF_iterations)).^2 + (predicted_opt_states(4*order(i) - 2, 1:KF_iterations) - predicted_opt_states(4*order(car) - 2, 1:KF_iterations)).^2 );
                        collision_points = find(x < (safe_distance(order(i)) + safe_distance(order(car)) + safety_margin));                        
                        
                        if((sum(size(collision_points)) - 2) > 0)                            delay = delay + 0.5;
                        else                        run_optimization(order(i)) = 0;                        end
                    end                    
                else                    run_optimization(order(i)) = 0;                end
                
            end
        end
        time_optimal
        time_to_cross - time_optimal
        'Cost = '
        mean(time_to_cross - time_optimal)
        toc
    elseif((show_optimization == 0) | (KF_interval <= iteration_to_start_optimization))
        for i = 1:num_of_cars
            predicted_opt_states(4*i - 3, KF_interval) = corrected_states(5*i - 4, KF_interval);
            predicted_opt_states(4*i - 2, KF_interval) = corrected_states(5*i - 3, KF_interval);
            predicted_opt_states(4*i - 1, KF_interval) = corrected_states(5*i - 1, KF_interval)*h*gear_ratios(i, KF_interval)*3600/1609.344;
            predicted_opt_states(4*i - 0, KF_interval) = 0;
        end
    end
    
    % Assign nominal controls
    predictions(:, 1) = corrected_states(:, KF_interval);
    nominal_controls = u(:, KF_interval);
    
    if(auto_transmission == 1)
        % Transmission
        for car = 1:num_of_cars
            v =  gear_ratios(car, KF_interval)*h*predictions(5*car - 1, 1);
            v_mph = v*(3600/1609.344);                        % Convert m/s -> mph
    
            if( (nominal_controls(3*car - 2) == 85) && (v_mph > normal_shift_speeds(2)) )
                % Down shift for better acceleration                
                if(v_mph > fast_shift_speeds(4))                              gear_ratios(car, KF_interval + 1) = Rg(5);                        Jtg_current(car) = Jtg(4);
                elseif(v_mph > fast_shift_speeds(3))                        gear_ratios(car, KF_interval + 1) = Rg(4);                        Jtg_current(car) = Jtg(4);
                elseif(v_mph > fast_shift_speeds(2))                        gear_ratios(car, KF_interval + 1) = Rg(3);                        Jtg_current(car) = Jtg(3);
                elseif(v_mph > fast_shift_speeds(1))                        gear_ratios(car, KF_interval + 1) = Rg(2);                        Jtg_current(car) = Jtg(2);
                else                                                                                 gear_ratios(car, KF_interval + 1) = Rg(1);                        Jtg_current(car) = Jtg(1);
                end
            elseif(v_mph > normal_shift_speeds(5))                    gear_ratios(car, KF_interval + 1) = Rg(5);                    Jtg_current(car) = Jtg(4);
            elseif(v_mph > normal_shift_speeds(4))                    gear_ratios(car, KF_interval + 1) = Rg(4);                    Jtg_current(car) = Jtg(4);
            elseif(v_mph > normal_shift_speeds(3))                    gear_ratios(car, KF_interval + 1) = Rg(3);                    Jtg_current(car) = Jtg(3);
            elseif(v_mph > normal_shift_speeds(2))                    gear_ratios(car, KF_interval + 1) = Rg(2);                    Jtg_current(car) = Jtg(2);
            else                                                                                   gear_ratios(car, KF_interval + 1) = Rg(1);                    Jtg_current(car) = Jtg(1);
            end
               
            predictions(5*car - 1, 1) = v/(gear_ratios(car, KF_interval + 1)*h);                % Adjusts the engine velocity if gear shift occurs
        end
    end

    % Prediction
    for integration_interval = 1:integration_iterations        
        RK_states = predictions(:, integration_interval);
        
        for j = 1:4
            k(1:5:5*num_of_cars, j) = gear_ratios(:, KF_interval)*h.*RK_states(4:5:5*num_of_cars).*cos(nominal_controls(3:3:3*num_of_cars)*pi/180);
            k(2:5:5*num_of_cars, j) = gear_ratios(:, KF_interval)*h.*RK_states(4:5:5*num_of_cars).*sin(nominal_controls(3:3:3*num_of_cars)*pi/180);
            
            d_ma_i = MAX*( (nominal_controls(1:3:3*num_of_cars)/85).^2 ).*( (-4.9958)*(c_PR*RK_states(3:5:5*num_of_cars)).^5 + (5.8832)*(c_PR*RK_states(3:5:5*num_of_cars)).^4 + (-1.1218)*(c_PR*RK_states(3:5:5*num_of_cars)).^3 + (-.6579)*(c_PR*RK_states(3:5:5*num_of_cars)).^2 + (-.1278)*(c_PR*RK_states(3:5:5*num_of_cars)) + 1.0104 );
            d_ma_o = c_1*(24.5*RK_states(4:5:5*num_of_cars).*RK_states(3:5:5*num_of_cars).^2 - 0.167*RK_states(4:5:5*num_of_cars).*RK_states(3:5:5*num_of_cars) + (8.10*10^(-4))*RK_states(4:5:5*num_of_cars) - (3.10*10^(-4))*RK_states(3:5:5*num_of_cars).^2 + 222*RK_states(3:5:5*num_of_cars) + 0.352).*RK_states(3:5:5*num_of_cars).*RK_states(4:5:5*num_of_cars);
    
            k(3:5:5*num_of_cars, j) = d_ma_i - d_ma_o;            
            k(4:5:5*num_of_cars, j) = ( ((c_2*d_ma_o)./RK_states(4:5:5*num_of_cars)) - (0.1056*RK_states(4:5:5*num_of_cars) + 15.10) - ((1*10^-4)*RK_states(4:5:5*num_of_cars).^2) - (gear_ratios(:, KF_interval).*(RK_states(5:5:(5*num_of_cars)) + Ca*(gear_ratios(:, KF_interval).^2).*(h^3).*(RK_states(4:5:(5*num_of_cars)).^2) + h*Fr)))./(Je + Jtg_current + (gear_ratios(:, KF_interval).^2)*(Jw + 4*M*h^2));
            k(5:5:5*num_of_cars, j) = (Kb*nominal_controls(2:3:3*num_of_cars) - RK_states(5:5:5*num_of_cars))/Tau_bv;
            
            if(j < 3)                RK_states = RK_states + (delta_t/2)*k(:, j);
            else                       RK_states = RK_states + delta_t*k(:, j);            end
        end
        
        predictions(:, integration_interval + 1) = predictions(:, integration_interval) + (delta_t/6)*( k(:, 1) + 2*k(:, 2) + 2*k(:, 3) + k(:, 4) );  % State Transition
    end
    
    for i = 1:num_of_cars
        % we constraints (if braking limit we = 0 else w = idle speed)
        if( (predictions(5*i - 1, integration_iterations + 1) < 6.5) && (u(3*i - 1, KF_interval) > 0) )                predictions(5*i - 1, integration_iterations + 1) = 10^-5;
        elseif(predictions(5*i - 1, integration_iterations + 1) < 6.5)                predictions(5*i - 1, integration_iterations + 1) = 6.5;        end
    end
    
    predicted_states(:, KF_interval + 1) = predictions(:, integration_iterations + 1);
    
    if(noise ~= 0)        % If noise == 0, skip correction step
        % Assign new values to noise dependent variables
        for i = 1:num_of_cars
            if(noise == 2)
                G(5*car - 4, 5*car  - 1) = gear_ratios(car, KF_interval)*h*cos(nominal_controls(3*car - 0)*pi/180);
                G(5*car - 3, 5*car  - 1) = gear_ratios(car, KF_interval)*h*sin(nominal_controls(3*car - 0)*pi/180);
                G(5*car - 1, 5*car  - 2) = ( (.352)*c_2*c_1 )/(  Je + Jtg_current(car) + (gear_ratios(car, KF_interval).^2)*(Jw + 4*M*h^2) );
                G(5*car - 1, 5*car  - 0) = -gear_ratios(car, KF_interval)/(  Je + Jtg_current(car) + (gear_ratios(car, KF_interval).^2)*(Jw + 4*M*h^2) );
            end
            
            w(5*i - 4) = randn*sqrt(q(1)); w(5*i - 3) = randn*sqrt(q(2)); w(5*i - 2) = randn*sqrt(q(3)); w(5*i - 1) = randn*sqrt(q(4)); w(5*i - 0) = randn*sqrt(q(5));        
            e(5*i - 4) = randn*sqrt(q(1)); e(5*i - 3) = randn*sqrt(q(2)); e(5*i - 2) = randn*sqrt(q(3)); e(5*i - 1) = randn*sqrt(q(4)); e(5*i - 0) = randn*sqrt(q(5));
        end
    
        % Creates alternating noise +\- if enabled
        if(rough ~= 0)
            if (KF_interval == 2*floor(KF_interval/2))                w = -rough*abs(w);
            else                w = rough*abs(w);            end
        end
    
        % Get new observations
        x = predicted_states(:, KF_interval + 1) + G*w;        z = x + e;
    
        % Iterated Corrections
        corrections(:, 1) = 0;        corrections(:, 2) = predicted_states(:, KF_interval + 1);

        i = 0;
        while( (max( abs(corrections(:, 2) - corrections(:, 1)) ) > e_correction) && (i < max_corrections) )
            i = i + 1;            corrections(:, 1) = corrections(:, 2);
        
            % Compute Jacobian (non-zero along block diagonal)
            for car = 1:num_of_cars
                Fx(5*car - 4, 5*car - 4) = 0;                Fx(5*car - 4, 5*car - 3) = 0;                Fx(5*car - 4, 5*car - 2) = 0;
                Fx(5*car - 4, 5*car - 1) = gear_ratios(car, KF_interval)*h*cos(nominal_controls(3*car - 0)*pi/180);
                Fx(5*car - 4, 5*car - 0) = 0;
        
                Fx(5*car - 3, 5*car - 4) = 0;                Fx(5*car - 3, 5*car - 3) = 0;                Fx(5*car - 3, 5*car - 2) = 0;
                Fx(5*car - 3, 5*car - 1) = gear_ratios(car, KF_interval)*h*sin(nominal_controls(3*car - 0)*pi/180);
                Fx(5*car - 3, 5*car - 0) = 0;
        
                Fx(5*car - 2, 5*car - 4) = 0;                Fx(5*car - 2, 5*car - 3) = 0;
                Fx(5*car - 2, 5*car - 2) = MAX*((nominal_controls(3*car - 2)/85)^2)*(5*(-4.9958)*c_PR^5*corrections(5*car - 2, 1)^4 + 4*(5.8832)*c_PR^4*corrections(5*car - 2, 1)^3 + 3*(-1.1218)*c_PR^3*corrections(5*car - 2, 1)^2 + 2*(-.6579)*c_PR^2*corrections(5*car - 2, 1) + (-.1278)*c_PR) - (c_1*(3*24.5*corrections(5*car - 1, 1)^2*corrections(5*car - 2, 1)^2 - 2*0.167*corrections(5*car - 1, 1)^2*corrections(5*car - 2, 1) + 8.10*10^-4*corrections(5*car - 1, 1)^2 - 3*3.10*10^-4*corrections(5*car - 1, 1)*corrections(5*car - 2, 1)^2 + 2*222*corrections(5*car - 1, 1)*corrections(5*car - 2, 1) + 0.352*corrections(5*car - 1, 1)) );
                Fx(5*car - 2, 5*car - 1) = -c_1*(2*24.5*corrections(5*car - 1, 1)*corrections(5*car - 2, 1)^3 - 2*0.167*corrections(5*car - 1, 1)*corrections(5*car - 2, 1)^2 + 2*8.10*10-4*corrections(5*car - 1, 1)*corrections(5*car - 2, 1) - 3.10*10^-4*corrections(5*car - 2, 1)^3 + 222*corrections(5*car - 2, 1)^2 + 0.352*corrections(5*car - 2, 1));
                Fx(5*car - 2, 5*car - 0) = 0;
        
                Je_star = Je + Jtg_current(car) + (gear_ratios(car, KF_interval)^2)*(Jw + 4*M*h^2);
        
                Fx(5*car - 1, 5*car - 4) = 0;                Fx(5*car - 1, 5*car - 3) = 0;
                Fx(5*car - 1, 5*car - 2) = (c_1*c_2/Je_star)*( 3*24.5*corrections(5*car - 1, 1)*corrections(5*car - 2, 1)^2 - 2*0.167*corrections(5*car - 1, 1)*corrections(5*car - 2, 1) + 8.10*10^-4*corrections(5*car - 1, 1) - 3*3.10*10^-4*corrections(5*car - 2, 1)^2 + 2*222*corrections(5*car - 2, 1) + 0.352 );
                Fx(5*car - 1, 5*car - 1) = (1/Je_star)*( (c_1*c_2*(24.5*corrections(5*car - 2, 1)^3 - 0.167*corrections(5*car - 2, 1)^2 + 8.10*10^-4*corrections(5*car - 2, 1) ) - 0.1056 - 2*(4.3*10^-6)*corrections(5*car - 1, 1) - 2*Ca*gear_ratios(car, KF_interval)^3*h^3*corrections(5*car - 1, 1)) );
                Fx(5*car - 1, 5*car - 0) = -gear_ratios(car, KF_interval)/Je_star;
       
                Fx(5*car - 0, 5*car - 4) = 0;                Fx(5*car - 0, 5*car - 3) = 0;                Fx(5*car - 0, 5*car - 2) = 0;                Fx(5*car - 0, 5*car - 1) = 0;
                Fx(5*car - 0, 5*car - 0) = -1/Tau_bv;
            end
            
            phi = I(5*num_of_cars) + Fx*T;            Q = G*Q*G'*T;            sigma = phi*sigma*phi' + Q;
            K = sigma*Fx'*inv(Fx*sigma*Fx' + Rk);            sigma = (I(5*num_of_cars) - K*Fx)*sigma;
    
            corrections(:, 2) = corrections(:, 1) + K*(z - predicted_states(:, KF_interval + 1));
        end
        if( i > 1 )            [KF_interval            i],            correction_iterations(KF_interval) = i;        end
        
        % Force physical and model constraints
        for i = 1:num_of_cars
            if(corrections(5*i, 2) < 0)                corrections(5*i, 2) = 0;            end            % Force Tb > 0
        
            % we constraints (if braking limit we = 0 else w = idle speed)
            if( (corrections(5*i - 1, 2) < 6.5) && (u(3*i - 1, KF_interval) > 0) )                corrections(5*i - 1, 2) = 10^-5;
            elseif(corrections(5*i - 1, 2) < 6.5)                corrections(5*i - 1, 2) = 6.5;            end
        end
    
        corrected_states(:, KF_interval + 1) = corrections(:, 2);
    else
        corrected_states(:, KF_interval + 1) = predicted_states(:, KF_interval + 1);
    end    
    
end

% Post Processing
i = 0;

if(mode_output(1) == 1)
    i = i + 1;    figure(i);
    
    % Plot Throttle
    for j = 1:num_of_cars
        subplot(num_of_cars, 1, j);        hold on;        grid;
        plot((1:KF_iterations)*T, u(3*j - 2, 1:KF_iterations), 'blue');
        title(['Car ', int2str(j), ' : Throttle Angle wrt Time']);    xlabel('Time [s]');    ylabel('\alpha [deg]');
    end
end

if(mode_output(2) == 1)
    i = i + 1;    figure(i);
    
    % Plot Brake Angle
    for j = 1:num_of_cars
        subplot(num_of_cars, 1, j);        hold on;        grid;
        plot((1:KF_iterations)*T, u(3*j - 1, 1:KF_iterations), 'blue');
        title(['Car ', int2str(j), ' : Brake Angle wrt Time']);    xlabel('Time [s]');    ylabel('\beta [deg]');
    end
end

if(mode_output(3) == 1)
    i = i + 1;    figure(i);
    
    % Plot Steering Angle
    for j = 1:num_of_cars
        subplot(num_of_cars, 1, j);        hold on;        grid;
        plot((1:KF_iterations)*T, u(3*j - 0, 1:KF_iterations), 'blue');
        title(['Car ', int2str(j), ' : Steering Angle wrt Time']);    xlabel('Time [s]');    ylabel('\theta [deg]');
    end
end

if(mode_output(4) == 1)
    i = i + 1;    figure(i);
    
    % Plot Positions (x2 vs. x1)
    for j = 1:num_of_cars
        subplot(num_of_cars, 1, j);        hold on;
        
        for k = 1:num_of_cars
            % North
            plot([ -lane_width, -lane_width ],              [ 1000, (0 + lane_width) ], 'black');            plot([ -line_separation, -line_separation ], [ 1000, (0 + lane_width) ], 'yellow');
            plot([ 0, 0 ],                                                   [ 1000, (0 + lane_width) ], 'black');            plot([ line_separation, line_separation ],    [ 1000, (0 + lane_width) ], 'yellow');
            plot([ lane_width, lane_width ],                 [ 1000, (0 + lane_width) ], 'black');
            
            % South
            plot([ -lane_width, -lane_width ],              [ -1000, (0 - lane_width) ], 'black');            plot([ -line_separation, -line_separation ], [ -1000, (0 - lane_width) ], 'yellow');
            plot([ 0, 0 ],                                                   [ -1000, (0 - lane_width) ], 'black');            plot([ line_separation, line_separation ],    [ -1000, (0 - lane_width) ], 'yellow');
            plot([ lane_width, lane_width ],                 [ -1000, (0 - lane_width) ], 'black');
            
            % West
            plot([ -1000, (0 - lane_width) ], [ lane_width, lane_width ], 'black');            plot([ -1000, (0 - lane_width) ], [ line_separation, line_separation ], 'yellow');
            plot([ -1000, (0 - lane_width) ], [ 0, 0 ], 'black');            plot([ -1000, (0 - lane_width) ], [ -line_separation, -line_separation ], 'yellow');
            plot([ -1000, (0 - lane_width) ], [ -lane_width, -lane_width ], 'black');
            
            % East
            plot([ (0 + lane_width), 1000 ], [ lane_width, lane_width ], 'black');            plot([ (0 + lane_width), 1000 ], [ line_separation, line_separation ], 'yellow');
            plot([ (0 + lane_width), 1000 ], [ 0 0 ], 'black');            plot([ (0 + lane_width), 1000 ], [ -line_separation, -line_separation ], 'yellow');
            plot([ (0 + lane_width), 1000 ], [ -lane_width, -lane_width ], 'black');
        end
        
        plot(corrected_states(5*j - 4, 1), corrected_states(5*j - 3, 1), 'red O');
        p = plot(predicted_states(5*j - 4, 1:KF_iterations), predicted_states(5*j - 3, 1:KF_iterations), 'blue');
        c = plot(corrected_states(5*j - 4, 1:KF_iterations), corrected_states(5*j - 3, 1:KF_iterations), 'red');
        o = plot(predicted_opt_states(4*j - 3, 1:KF_iterations), predicted_opt_states(4*j - 2, 1:KF_iterations), 'green');
        title(['Car ', int2str(j), ' : Position']);    xlabel('x_1 [m]');    ylabel('x_2 [m]');
        legend([p c o], 'Predicted', 'Corrected', 'Optimization', 'Location', 'Best');
    end
end

if(mode_output(5) == 1)
    i = i + 1;    figure(i);    count = 0;
    
    % Plot Position [x vs. t]
    for j = 1:num_of_cars
        for k = 1:2
            count = count + 1;
            subplot(num_of_cars, 2, count);        grid;        hold on;
            p = plot((1:KF_iterations)*T, predicted_states(5*j - 5 + k, 1:KF_iterations), 'blue');
            c = plot((1:KF_iterations)*T, corrected_states(5*j - 5 + k, 1:KF_iterations), 'red');
            o = plot((1:KF_iterations)*T, predicted_opt_states(4*j - 4 + k, 1:KF_iterations), 'green');
            title(['Car ', int2str(j), ' : x', int2str(k), ' wrt Time']);    xlabel('Time [s]');    ylabel(['x', int2str(k)]);
            %legend([p c o], 'Predicted', 'Corrected', 'Optimization', 'Location', 'Best');
        end
    end
end

if(mode_output(6) == 1)
    i = i + 1;    figure(i);
    
    % Plot Velocity [MPH]
    for j = 1:num_of_cars
        subplot(num_of_cars, 1, j);        grid;        hold on;
        p = plot((1:KF_iterations)*T, predicted_states(5*j - 1, 1:KF_iterations)*h.*gear_ratios(j, 1:KF_iterations)*(3600/1609.344), 'blue');
        c = plot((1:KF_iterations)*T, corrected_states(5*j - 1, 1:KF_iterations)*h.*gear_ratios(j, 1:KF_iterations)*(3600/1609.344), 'red');
        o = plot((1:KF_iterations)*T, predicted_opt_states(4*j - 1, 1:KF_iterations), 'green');
        title(['Car ', int2str(j), ' : Velocity wrt Time']);    xlabel('Time [s]');    ylabel('V [MPH]');
        legend([p c o], 'Predicted', 'Corrected', 'Optimization', 'Location', 'Best');
    end
end

if(mode_output(7) == 1)
    i = i + 1;    figure(i);
    
    % Plot Velocity [m/s]
    for j = 1:num_of_cars
        subplot(num_of_cars, 1, j);    grid;    hold on;
        p = plot((1:KF_iterations)*T, predicted_states(5*j - 1, 1:KF_iterations).*gear_ratios(j, 1:KF_iterations)*h, 'blue');
        c = plot((1:KF_iterations)*T, corrected_states(5*j - 1, 1:KF_iterations).*gear_ratios(j, 1:KF_iterations)*h, 'red');
        o = plot((1:KF_iterations)*T, predicted_opt_states(4*j - 1, 1:KF_iterations)*1609.344/3600, 'green');
        title(['Car ', int2str(j), ' : Velocity wrt Time']);    xlabel('Time [s]');    ylabel('V [m/s]');
        legend([p c o], 'Predicted', 'Corrected', 'Optimization', 'Location', 'Best');
    end
end

if(mode_output(8) == 1)
    if(num_of_cars > 1)
        i = i + 1;    figure(i);
        count = 0;
    
        if(num_of_cars > 3)        rows = num_of_cars*(num_of_cars - 1)/4;        cols = 2;
        else        rows = num_of_cars*(num_of_cars - 1)/2;        cols = 1;
        end
    
        % Plot Distance between Cars
        for j = 1:(num_of_cars - 1)
            for k = (j + 1):num_of_cars
                predicted_distance = sqrt(( predicted_states(5*j - 4, 1:KF_iterations) - predicted_states(5*k - 4, 1:KF_iterations) ).^2 + ( predicted_states(5*j - 3, 1:KF_iterations) - predicted_states(5*k - 3, 1:KF_iterations) ).^2);
                corrected_distance = sqrt(( corrected_states(5*j - 4, 1:KF_iterations) - corrected_states(5*k - 4, 1:KF_iterations) ).^2 + ( corrected_states(5*j - 3, 1:KF_iterations) - corrected_states(5*k - 3, 1:KF_iterations) ).^2);
                optimization = sqrt(( predicted_opt_states(4*j - 3, 1:KF_iterations) - predicted_opt_states(4*k - 3, 1:KF_iterations) ).^2 + ( predicted_opt_states(4*j - 2, 1:KF_iterations) - predicted_opt_states(4*k - 2, 1:KF_iterations) ).^2);
    
                count = count + 1;
                subplot(rows, cols, count);
                hold on;            grid;
                p = plot((1:KF_iterations)*T, predicted_distance(1:KF_iterations), 'blue');
                c = plot((1:KF_iterations)*T, corrected_distance(1:KF_iterations), 'red');
                o = plot((1:KF_iterations)*T, optimization(1:KF_iterations), 'green');
                collision = plot([0 KF_iterations*T], ( safe_distance(j) + safe_distance(k) )*[1 1], 'black');
                title(['Distance b/w Cars ', int2str(j), ' and ', int2str(k) ' wrt Time']);    xlabel('Time [s]');    ylabel('Distance [m]');
                legend([p c o collision], 'Predicted', 'Corrected', 'Optimization', 'Collision Threshold', 'Location', 'Best');
            end
        end
    end
end

if(mode_output(9) == 1)
    i = i + 1;    figure(i);
    
    % Plot Mass of Air in Intake Manifold
    for j = 1:num_of_cars
        subplot(num_of_cars, 1, j);        grid;    hold on;
        p = plot((1:KF_iterations)*T, predicted_states(5*j - 2, 1:KF_iterations), 'blue');
        c = plot((1:KF_iterations)*T, corrected_states(5*j - 2, 1:KF_iterations), 'red');
        title(['Car ', int2str(j), ' : Mass of Air in Intake Manifold wrt Time']);    xlabel('Time [s]');    ylabel('m_a [kg]');
        legend([p c], 'Predicted', 'Corrected', 'Location', 'Best');
    end
end

if(mode_output(10) == 1)
    i = i + 1;    figure(i);
    
    % Plot Engine Angular Velocity
    for j = 1:num_of_cars
        subplot(num_of_cars, 1, j);    grid;    hold on;
        p = plot((1:KF_iterations)*T, predicted_states(5*j - 1, 1:KF_iterations), 'blue');
        c = plot((1:KF_iterations)*T, corrected_states(5*j - 1, 1:KF_iterations), 'red');
        title(['Car ', int2str(j), ' : Engine Angular Velocity wrt Time']);    xlabel('Time [s]');    ylabel('\omega_e [rad/s]');
        legend([p c], 'Predicted', 'Corrected', 'Location', 'Best');
    end
end

if(mode_output(11) == 1)
    i = i + 1;    figure(i);
    
    % Plot Brake Torque
    for j = 1:num_of_cars
        subplot(num_of_cars, 1, j);    grid;    hold on;
        p = plot((1:KF_iterations)*T, predicted_states(5*j - 0, 1:KF_iterations), 'blue');
        c = plot((1:KF_iterations)*T, corrected_states(5*j - 0, 1:KF_iterations), 'red');
        title(['Car ', int2str(j), ' : Brake Torque wrt Time']);    xlabel('Time [s]');    ylabel('T_b [N*m]');
        legend([p c], 'Predicted', 'Corrected', 'Location', 'Best');
    end
end

if(mode_output(12) == 1)
    i = i + 1;    figure(i);
    
    for j = 1:num_of_cars        for k = 1:KF_iterations            gear_ratios(j, k) = find(Rg == gear_ratios(j, k));        end,    end
        
    % Plot Gear
    for j = 1:num_of_cars
        subplot(num_of_cars, 1, j);    grid;    hold on;
        p = plot((1:KF_iterations)*T, gear_ratios(j, 1:KF_iterations), 'red');
        title(['Car ', int2str(j), ' : Gear wrt Time']);    xlabel('Time [s]');    ylabel('Gear');
    end
end

if(mode_output(13) == 1)
    i = i + 1;    figure(i);
    
    % Plot Phase Plot
    for j = 1:num_of_cars
        subplot(num_of_cars, 1, j);        grid;    hold on;
        p = plot(predicted_states(5*j - 2, 1:KF_iterations), predicted_states(5*j - 1, 1:KF_iterations), 'blue');
        c = plot(corrected_states(5*j - 2, 1:KF_iterations), corrected_states(5*j - 1, 1:KF_iterations), 'red');
        title(['Car ', int2str(j), ' : Engine Angular Velocity wrt Mass of Air in the Intake Manifold']);    xlabel('m_a [kg]');    ylabel('\omega_e [rad/s]');
        legend([p c], 'Predicted', 'Corrected', 'Location', 'Best');
    end
    
    plot(corrected_states(5*j - 2, 1:KF_iterations/10:KF_iterations), corrected_states(5*j - 1, 1:KF_iterations/10:KF_iterations), 'O red');
end

if(mode_output(14) == 1)
    i = i + 1;    figure(i);    grid;    hold on;
            
    % Plot Correction Iterations
    plot((1:KF_iterations)*T, correction_iterations(1:KF_iterations, 1), 'red');
    m = plot([1 KF_iterations]*T, [mean(correction_iterations(1:KF_iterations, 1)) mean(correction_iterations(1:KF_iterations, 1))], 'green');
    title(['Corrector Iterations wrt Time']);    xlabel('Time [s]');    ylabel('Corrector Iterations');
    legend([m], 'Mean', 'Location', 'Best');
end

toc
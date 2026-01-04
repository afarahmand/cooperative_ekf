% Ashil Farahmand
% Recursive Iterated EKF using UCB Engine Model

% Goals
% ------------------
% - change iterated EKF to Newton Method?
% - integrate objective function
% ------------------------------------------------------------------------
% sigma = Cov of prediction state error
% Rk = Cov observation noise/delta
% ------------------------------------------------------------------------

tic
clc
clear
format long

temp = 85;
start = 21.01;
stop = 22.35;
start_t = 6.539;
stop_t = 22.16;
slope = (stop - start)/(stop_t - start_t)
KF_iterations = 4*10^4;      % Number of iterations for KF
integration_iterations = 1;    % Number of integrations per each KF_iteration

T = 1*10^-3;                     % Kalman Filter sampling period

rough = 0;                         % 0 = random noise, x = noise multiplied by value of rough alternates :/- each iteration
no_noise = 1;                   % 0 = random noise, 1 = no noise(for diagnostics on predictor)
gear_lock = 0;                   % 0 = automatic transmission, 1= gear is locked
e_correction = 10^-6;       % Allowable Correction Error

num_of_cars = 2;

mode_output = [0, 1, 0, 1,    0, 0, 0, 1,    0, 1];

% Output Modes
% -----------------------------------------------------
% Index | Property
% 1          throttle angle
% 2            brake angle
% 3       steering direction
% 4              v_mph
% 5             position
% 6                 ma
% 7                 we
% 8                 Tb
% 9  distance b/w car 1 and 2
% 10             v_m/s

% DeclareVariables and set ICs
G = 0;                                                                 %
w = zeros(5*num_of_cars, 1);                         % System Error
W = 0;                                                                 % Cov(w)
z = zeros(5*num_of_cars, 1);                           % Observations
e = zeros(5*num_of_cars, 1);                            % Observation Error

Fx = zeros(5*num_of_cars, 5*num_of_cars); % Jacobian

nominal_controls = zeros(3*num_of_cars);  % Control Vector during one KF iteration
predictions = zeros(5*num_of_cars, integration_iterations + 1);  % Holds the predictions
corrections = zeros(5*num_of_cars, 2);          % Holds the corrected states when iterating the correction step (holds previous and next corrections)

predicted_states = zeros(5*num_of_cars, KF_iterations);
corrected_states = zeros(5*num_of_cars, KF_iterations);
predicted_velocity = zeros(num_of_cars, KF_iterations);
corrected_velocity = zeros(num_of_cars, KF_iterations);

% Assign Initial Conditions
sigma = I(5*num_of_cars);
Rk = I(5*num_of_cars);
K = I(5*num_of_cars);
W = I(5*num_of_cars);
G = I(5*num_of_cars);
Q = I(5*num_of_cars);

% -----------------------------------------------------
% -----------------------------------------------------
% -----------------------------------------------------
% Begin Engine Model Parameters

% sets size of time interval for each integration*****************
delta_t = T/integration_iterations;                           % One KF_interval time is integration_iterations*delta_t = T
k = zeros(4, 5*num_of_cars);                                    % RK4 variables; one col/SV

% Declare Variables
% -----------------------------------------------------
RK_states = zeros(5*num_of_cars, 1);                     % state vector
u = zeros(3*num_of_cars, KF_iterations);               % control vector

Rg_current = zeros(num_of_cars, 1);     % gear dependent constants
Jtg_current = zeros(num_of_cars, 1);

MAX = 0.684;      % [kg/s]

R = 8.3145;           % [J/ mol*K]
Tm = 333;            % [K]
Mair = .02897;      % [kg/mol]
Vm = .00447;       % [m^3]
Patm = 101325;    % [Pa]
c_PR = (R*Tm)/(Patm*Mair*Vm);

Ve = .0049;           % [m^3]
c_1 = Ve/(4*pi*Vm);

normal_shift_speeds = [0; 7; 15; 25; 45];      % Speed to shift
fast_shift_speeds = [11; 20; 35; 55];              % Speeds to shift under max acceleration

Rg = [0.4167; 0.6817; 1; 1.4993; 2.3058];
c_2 = 1018686;      % [N*m/kg]
Ca = 0.53384;
h = 0.33;
Fr = 167.27;
Je = 0.2630;            % [kg*m^2]
Jtg = [0.08202; 0.07592; 0.11388; 0.13150];     % [kg*m^2]
Jw = 5.13;              % [kg*m^2]
M = 2148;               % [kg]

Kb = 300;
Tau_bv = 0.1;        % [s]

% End constants
% -----------------------------------------------------

% Car 1
% Position (-1000, 0) [m], v = 50 [mph] 22.35[m/s], direction = right, gear = 5th
car = 1;
Rg_current(car) = Rg(5);
predictions(5*car - 4, 1) = -1000;
predictions(5*car - 3, 1) = 0;
predictions(5*car - 2, 1) = .003455;
predictions(5*car - 1, 1) = 46.9 * (1609.344/3600) * (1/(h*Rg_current(car)));
predictions(5*car - 0, 1) = 0;
u(3*car - 2, KF_iterations/10:KF_iterations) = temp;

u(3*car - 0, 1:KF_iterations) = 0;

% Car 2
% Position (0, 1000) [m], v = 50 [mph] 22.35[m/s], direction = down, gear = 5th
car = 2;
if(num_of_cars > 1)
Rg_current(car) = Rg(5);
predictions(5*car - 4, 1) = 0;
predictions(5*car - 3, 1) = 1000;
predictions(5*car - 2, 1) = .003455;
predictions(5*car - 1, 1) = 50 * (1609.344/3600) * (1/(h*Rg_current(car)));
predictions(5*car - 0, 1) = 0;
u(3*car - 2, 1:KF_iterations) = 11.77215;
u(3*car - 1, 1:KF_iterations) = 0;
u(3*car, 1:KF_iterations) = 270;
end

predicted_states(:, 1) = predictions(:, 1);
corrected_states(:, 1) = predictions(:, 1);
predicted_velocity(:, 1) = predictions(4:5:5*num_of_cars, 1).*( (3600/1609.344)*(h*Rg_current) );
corrected_velocity(:, 1) = predictions(4:5:5*num_of_cars, 1).*( (3600/1609.344)*(h*Rg_current) );
% End Engine Model Parameters
% -----------------------------------------------------
% -----------------------------------------------------
% -----------------------------------------------------

% Begin to iterate (index 1 holds the initial conditions)
for KF_interval = 1:1:KF_iterations
    
    % Assign nominal controls
    predictions(:, 1) = corrected_states(:, KF_interval);
    nominal_controls = u(:, KF_interval);

    % Prediction
    for integration_interval = 1:1:integration_iterations

        if(gear_lock == 0)
            % Simulate Transmission
            for car = 1:num_of_cars
                v = Rg_current(car)*h*predictions(5*car - 1, integration_interval);
                v_mph = v*(3600/1609.344);                        % Convert m/s -> mph
    
                if( (nominal_controls(3*car - 2) == 85) & (v_mph > normal_shift_speeds(2)) )
                
                    % Down shift for better acceleration                
                    if(v_mph > fast_shift_speeds(4))
                        Rg_current(car) = Rg(5);
                        Jtg_current(car) = Jtg(4);
                    elseif(v_mph > fast_shift_speeds(3))
                        Rg_current(car) = Rg(4);
                        Jtg_current(car) = Jtg(4);
                    elseif(v_mph > fast_shift_speeds(2))
                        Rg_current(car) = Rg(3);
                        Jtg_current(car) = Jtg(3);
                    elseif(v_mph > fast_shift_speeds(1))
                        Rg_current(car) = Rg(2);
                        Jtg_current(car) = Jtg(2);
                    else
                        Rg_current(car) = Rg(1);
                        Jtg_current(car) = Jtg(1);
                    end            
                elseif(v_mph > normal_shift_speeds(5))
                    Rg_current(car) = Rg(5);
                    Jtg_current(car) = Jtg(4);
                elseif(v_mph > normal_shift_speeds(4))
                    Rg_current(car) = Rg(4);
                    Jtg_current(car) = Jtg(4);
                elseif(v_mph > normal_shift_speeds(3))
                    Rg_current(car) = Rg(3);
                    Jtg_current(car) = Jtg(3);
                elseif(v_mph > normal_shift_speeds(2))
                    Rg_current(car) = Rg(2);
                    Jtg_current(car) = Jtg(2);
                else
                    Rg_current(car) = Rg(1);
                    Jtg_current(car) = Jtg(1);
                end
               
                % Adjusts the engine velocity if gear shift occurs
                predictions(5*car - 1, integration_interval) = v/(Rg_current(car)*h);
            end
        end
        
        RK_states = predictions(:, integration_interval);
        
        for j = 1:4
            k(1:5:5*num_of_cars, j) = Rg_current.*h.*RK_states(4:5:(5*num_of_cars)).*cos(nominal_controls(3:3:(3*num_of_cars))*pi/180);            
            k(2:5:5*num_of_cars, j) = Rg_current.*h.*RK_states(4:5:(5*num_of_cars)).*sin(nominal_controls(3:3:(3*num_of_cars))*pi/180);
            
            d_ma_i = MAX*( (nominal_controls(1:3:3*num_of_cars)/85).^2 ).*( (-4.9958)*(c_PR*RK_states(3:5:5*num_of_cars)).^5 + (5.8832)*(c_PR*RK_states(3:5:5*num_of_cars)).^4 + (-1.1218)*(c_PR*RK_states(3:5:5*num_of_cars)).^3 + (-.6579)*(c_PR*RK_states(3:5:5*num_of_cars)).^2 + (-.1278)*(c_PR*RK_states(3:5:5*num_of_cars)) + 1.0104 );
            d_ma_o = c_1*(24.5*RK_states(4:5:5*num_of_cars).*RK_states(3:5:5*num_of_cars).^2 - 0.167*RK_states(4:5:5*num_of_cars).*RK_states(3:5:5*num_of_cars) + (8.10*10^(-4))*RK_states(4:5:5*num_of_cars) - (3.10*10^(-4))*RK_states(3:5:5*num_of_cars).^2 + 222*RK_states(3:5:5*num_of_cars) + 0.352).*RK_states(3:5:5*num_of_cars).*RK_states(4:5:5*num_of_cars);
    
            k(3:5:5*num_of_cars, j) = d_ma_i - d_ma_o;            
            k(4:5:5*num_of_cars, j) = (((c_2*d_ma_o)./RK_states(4:5:(5*num_of_cars))) - (0.1056*RK_states(4:5:(5*num_of_cars)) + 15.10) - ((4.3*10^-6)*RK_states(4:5:(5*num_of_cars)).^2) - (Rg_current.*(RK_states(5:5:(5*num_of_cars)) + Ca*(Rg_current.^2).*(h^3).*(RK_states(4:5:(5*num_of_cars)).^2) + h*Fr)))./(Je + Jtg_current + (Rg_current.^2)*(Jw + 4*M*h^2));    
            k(5:5:5*num_of_cars, j) = (Kb*nominal_controls(2:3:3*num_of_cars) - RK_states(5:5:(5*num_of_cars)))/Tau_bv;
            
            RK_states = RK_states + (delta_t/2)*k(:, j);            
            
            if(j == 3)
                RK_states = RK_states + delta_t*k(:, j);
            end
        end
        
        % Make k1 = k1 + k2 + k3 + k4
        for j = 2:3
            k(:, 1) = k(:, 1) + 2*k(:, j);
        end
        k(:, 1) = k(:, 1) + k(:, 4);
    
        % State Transition
        predictions(:, integration_interval + 1) = predictions(:, integration_interval) + (delta_t/6)*k(:, 1);
    end
    % --------------------------------------------------
    
    % Force we constraints (if braking limit we = 0 else w = idle speed)
    if( (predictions(4, integration_interval + 1) < 6.5) & (u(2, KF_interval) > 0) )
        predictions(4, integration_interval + 1) = 10^-5;
    elseif(predictions(4, integration_interval + 1) < 6.5)
        predictions(4, integration_interval + 1) = 6.5;
    end
    if( (predictions(9, integration_interval + 1) < 6.5) & (u(5, KF_interval) > 0) )
        predictions(9, integration_interval + 1) = 10^-5;
    elseif(predictions(9, integration_interval + 1) < 6.5)
        predictions(9, integration_interval + 1) = 6.5;
    end
    
    predicted_states(:, KF_interval + 1) = predictions(:, integration_iterations + 1);
    predicted_velocity(:, KF_interval + 1) = predictions(4:5:5*num_of_cars, integration_iterations + 1)*h.*Rg_current*(3600/1609.344);
    
    if(no_noise == 0)
        % Assign new noise values
        for i = 1:num_of_cars
            w(5*i - 4) = randn*.1;
            w(5*i - 3) = randn*.1;
            w(5*i - 2) = randn*10^-5;
            w(5*i - 1) = randn*.1;
            w(5*i - 0) = randn*10;
        
            e(5*i - 4) = randn*.1;
            e(5*i - 3) = randn*.1;
            e(5*i - 2) = randn*10^-5;
            e(5*i - 1) = randn*.1;
            e(5*i - 0) = randn*10;
        end
    
        % Creates alternating noise +\- if enabled
        if(rough ~= 0)
            if (KF_interval == 2*floor(KF_interval/2))         %Test for even number
                w = -rough*abs(w);
            else
                w = rough*abs(w);
            end
        end
    
        % Get new observations
        x = predicted_states(:, KF_interval + 1) + G*w;
        z = x + e;
    
        % Force Tb constraints
        if(x(5) < 0) x(5) = 0; end
        if(e(5) < 0) e(5) = 0; end
        if(x(10) < 0) x(10) = 0; end
        if(e(10) < 0) e(10) = 0; end
    
        % Iterated Corrections
        corrections(:, 1) = 0;
        corrections(:, 2) = predicted_states(:, KF_interval + 1);

        i = 0;
        while(max( abs(corrections(:, 2) - corrections(:, 1)) ) > e_correction)
            i = i + 1;
        
            corrections(:, 1) = corrections(:, 2);
        
            % Compute Jacobian (non-zero along block diagonal)
            for car = 1:num_of_cars
                Fx(5*car - 4, 5*car - 4) = 0;
                Fx(5*car - 4, 5*car - 3) = 0;
                Fx(5*car - 4, 5*car - 2) = 0;
                Fx(5*car - 4, 5*car - 1) = Rg_current(car)*h*T*cos(nominal_controls(3*car - 0));
                Fx(5*car - 4, 5*car - 0) = 0;
        
                Fx(5*car - 3, 5*car - 4) = 0;
                Fx(5*car - 3, 5*car - 3) = 0;
                Fx(5*car - 3, 5*car - 2) = 0;
                Fx(5*car - 3, 5*car - 1) = Rg_current(car)*h*T*sin(nominal_controls(3*car - 0));
                Fx(5*car - 3, 5*car - 0) = 0;
        
                Fx(5*car - 2, 5*car - 4) = 0;
                Fx(5*car - 2, 5*car - 3) = 0;
                Fx(5*car - 2, 5*car - 2) = MAX*((nominal_controls(3*car - 2)/85)^2)*(5*(-4.9958)*c_PR^5*corrections(5*car - 2, 1)^4 + 4*(5.8832)*c_PR^4*corrections(5*car - 2, 1)^3 + 3*(-1.1218)*c_PR^3*corrections(5*car - 2, 1)^2 + 2*(-.6579)*c_PR^2*corrections(5*car - 2, 1) + (-.1278)*c_PR) - (c_1*(3*24.5*corrections(5*car - 1, 1)^2*corrections(5*car - 2, 1)^2 - 2*0.167*corrections(5*car - 1, 1)^2*corrections(5*car - 2, 1) + 8.10*10^-4*corrections(5*car - 1, 1)^2 - 3*3.10*10^-4*corrections(5*car - 1, 1)*corrections(5*car - 2, 1)^2 + 2*222*corrections(5*car - 1, 1)*corrections(5*car - 2, 1) + 0.352*corrections(5*car - 1, 1)) );
                Fx(5*car - 2, 5*car - 1) = -c_1*(2*24.5*corrections(5*car - 1, 1)*corrections(5*car - 2, 1)^3 - 2*0.167*corrections(5*car - 1, 1)*corrections(5*car - 2, 1)^2 + 2*8.10*10-4*corrections(5*car - 1, 1)*corrections(5*car - 2, 1) - 3.10*10^-4*corrections(5*car - 2, 1)^3 + 222*corrections(5*car - 2, 1)^2 + 0.352*corrections(5*car - 2, 1));
                Fx(5*car - 2, 5*car - 0) = 0;
        
                Je_star = Je + Jtg_current(car) + (Rg_current(car)^2)*(Jw + 4*M*h^2);
        
                Fx(5*car - 1, 5*car - 4) = 0;
                Fx(5*car - 1, 5*car - 3) = 0;
                Fx(5*car - 1, 5*car - 2) = (1/Je_star)*(c_1*c_2*(3*24.5*corrections(5*car - 1, 1)*corrections(5*car - 2, 1)^2 - 2*0.167*corrections(5*car - 1, 1)*corrections(5*car - 2, 1) + 8.10*10^-4*corrections(5*car - 1, 1) - 3*3.10*10^-4*corrections(5*car - 2, 1)^2 + 2*222*corrections(5*car - 2, 1) + 0.352));
                Fx(5*car - 1, 5*car - 1) = (1/Je_star)*( (c_1*c_2*(24.5*corrections(5*car - 2, 1)^3 - 0.167*corrections(5*car - 2, 1)^2 + 8.10*10^-4*corrections(5*car - 2, 1)) - 0.1056 - 2*(4.3*10^-6)*corrections(5*car - 1, 1) - 2*Rg_current(car)*Ca*Rg_current(car)^2*h^3*corrections(5*car - 1, 1)));
                Fx(5*car - 1, 5*car - 0) = -Rg_current(car);
       
                Fx(5*car - 0, 5*car - 4) = 0;
                Fx(5*car - 0, 5*car - 3) = 0;
                Fx(5*car - 0, 5*car - 2) = 0;
                Fx(5*car - 0, 5*car - 1) = 0;
                Fx(5*car - 0, 5*car - 0) = -1/Tau_bv;
            end
    
            phi = I(5*num_of_cars) + Fx*T;
            Q = G*Q*K'*T;
    
            sigma = phi*sigma*phi' + Q;
            K = sigma*Fx'*inv(Fx*sigma*Fx' + Rk);
            sigma = (I(5*num_of_cars) - K*Fx)*sigma;
    
            corrections(:, 2) = corrections(:, 1) + K*(z - x);
        end
        %i
    
        corrected_states(:, KF_interval + 1) = corrections(:, 2);
        corrected_velocity(:, KF_interval + 1) = corrections(4:5:5*num_of_cars, 2)*h.*Rg_current*(3600/1609.344);
    else
        corrected_states(:, KF_interval + 1) = predicted_states(:, KF_interval + 1);
        corrected_velocity(:, KF_interval + 1) = predicted_velocity(:, KF_interval + 1);
    end
end

% Post Processing
i = 0;

if(mode_output(4) == 1)
    i = i + 1;    figure(i);
    
    % Plot Velocity
    subplot(2, 1, 1);
    grid; hold on;
    pv1 = plot((1:KF_iterations)*T, predicted_velocity(1, 1:KF_iterations), 'blue');
    cv1 = plot((1:KF_iterations)*T, corrected_velocity(1, 1:KF_iterations), 'red');
    title('Car 1: Velocity wrt Time');    xlabel('Time [s]');    ylabel('V [MPH]');
    legend([pv1 cv1], 'Predicted', 'Corrected', 'Location', 'Best');
    
    subplot(2, 1, 2);
    grid; hold on;
    pv2 = plot((1:KF_iterations)*T, predicted_velocity(2, 1:KF_iterations), 'blue');
    cv2 = plot((1:KF_iterations)*T, corrected_velocity(2, 1:KF_iterations), 'red');
    title('Car 2: Velocity wrt Time');    xlabel('Time [s]');    ylabel('V [MPH]');
    legend([pv2 cv2], 'Predicted', 'Corrected', 'Location', 'Best');
end

if(mode_output(5) == 1)
    i = i + 1;    figure(i);
    
    % Plot Positions
    subplot(2, 1, 1);
    grid;    hold on;
    px11 = plot(predicted_states(1, 1:KF_iterations), predicted_states(2, 1:KF_iterations), 'blue');
    cx21 = plot(corrected_states(1, 1:KF_iterations), corrected_states(2, 1:KF_iterations), 'red');
    title('Car 1: Position');    xlabel('x1');    ylabel('x2');
    legend([px11 cx21], 'Predicted', 'Corrected', 'Location', 'Best');
    
    subplot(2, 1, 2);
    grid;    hold on;
    px12 = plot(predicted_states(6, 1:KF_iterations), predicted_states(7, 1:KF_iterations), 'blue');
    cx22 = plot(corrected_states(6, 1:KF_iterations), corrected_states(7, 1:KF_iterations), 'red');
    title('Car 2: Position');    xlabel('x1');    ylabel('x2');
    legend([px12 cx22], 'Predicted', 'Corrected', 'Location', 'Best');
end

if(mode_output(6) == 1)
    i = i + 1;    figure(i);
    
    % Plot Mass of Air in Intake Manifold
    subplot(2, 1, 1);
    grid;    hold on;
    pma1 = plot((1:KF_iterations)*T, predicted_states(3, 1:KF_iterations), 'blue');
    cma1 = plot((1:KF_iterations)*T, corrected_states(3, 1:KF_iterations), 'red');
    title('Car 1: Mass of Air in Intake Manifold wrt Time');    xlabel('Time [s]');    ylabel('m_a [kg]');
    legend([pma1 cma1], 'Predicted', 'Corrected', 'Location', 'Best');
    
    subplot(2, 1, 2);
    grid;    hold on;
    pma2 = plot((1:KF_iterations)*T, predicted_states(8, 1:KF_iterations), 'blue');
    cma2 = plot((1:KF_iterations)*T, corrected_states(8, 1:KF_iterations), 'red');
    title('Car 2: Mass of Air in Intake Manifold wrt Time');    xlabel('Time [s]');    ylabel('m_a [kg]');
    legend([pma2 cma2], 'Predicted', 'Corrected', 'Location', 'Best');
end

if(mode_output(7) == 1)
    i = i + 1;    figure(i);
    
    % Plot Engine Angular Velocity
    subplot(2, 1, 1);
    grid;    hold on;
    pwe1 = plot((1:KF_iterations)*T, predicted_states(4, 1:KF_iterations), 'blue');
    cwe1 = plot((1:KF_iterations)*T, corrected_states(4, 1:KF_iterations), 'red');
    title('Car 1: Engine Angular Velocity wrt Time');    xlabel('Time [s]');    ylabel('w_e [rad/s]');
    legend([pwe1 cwe1], 'Predicted', 'Corrected', 'Location', 'Best');
    
    subplot(2, 1, 2);
    grid;    hold on;
    pwe2 = plot((1:KF_iterations)*T, predicted_states(9, 1:KF_iterations), 'blue');
    cwe2 = plot((1:KF_iterations)*T, corrected_states(9, 1:KF_iterations), 'red');
    title('Car 2: Engine Angular Velocity wrt Time');    xlabel('Time [s]');    ylabel('w_e [rad/s]');
    legend([pwe2 cwe2], 'Predicted', 'Corrected', 'Location', 'Best');
end

if(mode_output(8) == 1)
    i = i + 1;    figure(i);
    
    % Plot Brake Torque
    subplot(2, 1, 1);
    grid;    hold on;
    pTb1 = plot((1:KF_iterations)*T, predicted_states(5, 1:KF_iterations), 'blue');
    cTb1 = plot((1:KF_iterations)*T, corrected_states(5, 1:KF_iterations), 'red');
    title('Car 1: Brake Torque wrt Time');    xlabel('Time [s]');    ylabel('Tb [N*m]');
    legend([pTb1 cTb1], 'Predicted', 'Corrected', 'Location', 'Best');
    
    subplot(2, 1, 2);
    grid;    hold on;
    pTb2 = plot((1:KF_iterations)*T, predicted_states(10, 1:KF_iterations), 'blue');
    cTb2 = plot((1:KF_iterations)*T, corrected_states(10, 1:KF_iterations), 'red');
    title('Car 2: Brake Torque wrt Time');    xlabel('Time [s]');    ylabel('Tb [N*m]');
    legend([pTb2 cTb2], 'Predicted', 'Corrected', 'Location', 'Best');
end

if(mode_output(9) == 1)
    i = i + 1;    figure(i);
    
    % Plot Distance between Cars
    predicted_distance = sqrt(( predicted_states(6, 1:KF_iterations) - predicted_states(1, 1:KF_iterations) ).^2 + ( predicted_states(7, 1:KF_iterations) - predicted_states(2, 1:KF_iterations) ).^2);
    corrected_distance = sqrt(( corrected_states(6, 1:KF_iterations) - corrected_states(1, 1:KF_iterations) ).^2 + ( corrected_states(7, 1:KF_iterations) - corrected_states(2, 1:KF_iterations) ).^2);
    
    grid;    hold on;
    pdistance = plot((1:KF_iterations)*T, predicted_distance(1:KF_iterations), 'blue');
    cdistance = plot((1:KF_iterations)*T, corrected_distance(1:KF_iterations), 'red');
    collision = plot((1:KF_iterations)*T, 4*ones(KF_iterations, 1), 'green');
    title('Distance wrt Time');    xlabel('Time [s]');    ylabel('Distance [m]');
    legend([pdistance cdistance collision], 'Predicted', 'Corrected', 'Collision');
end

if(mode_output(10) == 1)
    i = i + 1;    figure(i);
    
    % Plot Velocity [m/s]
    subplot(2, 1, 1);
    grid;    hold on;
    pvms1 = plot((1:KF_iterations)*T, predicted_velocity(1, 1:KF_iterations)*(1609.344/3600), 'blue');
    cvms1 = plot((1:KF_iterations)*T, corrected_velocity(1, 1:KF_iterations)*(1609.344/3600), 'red');
    title('Car 1: Velocity wrt Time');    xlabel('Time [s]');    ylabel('V [m/s]');
    legend([pvms1 cvms1], 'Predicted', 'Corrected', 'Location', 'Best');
    plot([start_t stop_t], [start stop], 'black')
    
    subplot(2, 1, 2);
    grid;    hold on;
    pvms2 = plot((1:KF_iterations)*T, predicted_velocity(2, 1:KF_iterations)*(1609.344/3600), 'blue');
    cvms2 = plot((1:KF_iterations)*T, corrected_velocity(2, 1:KF_iterations)*(1609.344/3600), 'red');
    title('Car 2: Velocity wrt Time');    xlabel('Time [s]');    ylabel('V [m/s]');
    legend([pvms2 cvms2], 'Predicted', 'Corrected', 'Location', 'Best');
end
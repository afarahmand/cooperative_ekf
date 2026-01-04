% Human vs. DTOA

a = 4.47;                    % Accelerates from 0-30 MPH in 3 sec
t_green = 3:1:120;
v_cruise = 13.41;      % 30 MPH
lane_width = 3.6;
R = 2.5;
headway = 1.5;

n_Human = (.5*a*(v_cruise/a)^2 + v_cruise*(t_green - v_cruise/a) - 2*lane_width - 2*R)/(2*R + headway + v_cruise);
n_DTOA = (.5*a*(v_cruise/a)^2 + v_cruise*(t_green - v_cruise/a - 0.112) - 2*lane_width - 2*R)/(2*R + headway + 0.112*v_cruise);

figure(1); hold on; grid;
Human = plot(t_green, floor(n_Human), 'blue');
DTOA = plot(t_green, floor(n_DTOA), 'red');
title('Number of Vehicles Crossing Intersection wrt Green Interval at One Approach'); xlabel('Green Time [s]'); ylabel('Number of Vehicles');
legend([Human DTOA], 'Traffic Signal', 'Optimization', 'Location', 'Best');

figure(2); hold on; grid;
t = 0:.01:7;
Human_ramp = plot(t, 2.2124*(exp(t) - 1), 'blue');
t = 0:.01:3.22;
DTOA_ramp = plot(t, 250.08*(exp(t) - 1), 'red');
Human = plot([7 20], [2424 2424], 'blue');
DTOA = plot([3.22 20], [6034 6034], 'red');
title('Vehicle Flow Rate wrt Green Time at One Approach'); xlabel('Green Time [s]'); ylabel('Vehicle Flow Rate [veh/h]');
legend([Human DTOA], 'Traffic Signal', 'Optimization', 'Location', 'Best');


% Valid State Ranges

clc
clear

h = 0.33;
Rg = [.4167; .6817; 1; 1.4993; 2.3058; 0];            % Gear Reduction Ratios
e_shift_speed = 0.00;                                           % Error in shift speeds
shift_speeds = [0; 10*(1-e_shift_speed); 15*(1-e_shift_speed); 25*(1-e_shift_speed); 45*(1-e_shift_speed)];     % Speed to shift

v_mph = .1:.1:90;                       % Linear Velocity [mph] (independent)
v = v_mph*(1609.344/3600);
w_e = 0;                                       % Angular Velocity of Engine [rad/s] (dependent)

Rg(6) = Rg(1);         % Start in 1st gear

for i = 1:1:size(v, 2)    
    % Simulates Gear Shifting
    if(v_mph(i) > shift_speeds(5))
        Rg(6) = Rg(5);
    elseif(v_mph(i) > shift_speeds(4))
        Rg(6) = Rg(4);
    elseif(v_mph(i) > shift_speeds(3))
        Rg(6) = Rg(3);
    elseif(v_mph(i) > shift_speeds(2))
        Rg(6) = Rg(2);
    else
        Rg(6) = Rg(1);
    end
    
    w_e(i) = v(i)/(h*Rg(6));
end

hold off;
hold on;
%plot(v_mph, w_e*60/(2*pi), 'blue')
plot(v_mph, w_e, 'red')
title('Angular Velocity vs. Speed');
xlabel('Linear Speed [mph]')
ylabel('Blue - RPMs, Red - rad/s')
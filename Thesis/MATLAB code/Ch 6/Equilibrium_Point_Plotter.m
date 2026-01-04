clc; clear; format long; tic

% For stopping rule ------------------------------
throttle_samples = 85;     error = 1*10^-4;             n = 3*10^5;                          % Maximum iterations

MAX = 0.684; R = 8.3145; Tm = 333; Mair = .02897; Vm = .00447; Patm = 101325; Ve = .0049; c_1 = Ve/(4*pi*Vm);
Rg = [0.4167; 0.6817; 1; 1.4993; 2.3058; 0]; c_2 = 1018686; Ca = 0.53384; h = 0.33; Fr = 167.27; Je = 0.2630;
Jtg = [0.08202;           0.07592;           0.11388;           0.13150;           0]; Jw = 5.13; M = 2148; Tau_bv = 0.1; 

c = (R*Tm)/(Patm*Mair*Vm);
alpha = 1:throttle_samples;
ma = zeros(n, 1); we = zeros(n, 1); Tb = zeros(n, 1);

min_iter = zeros(throttle_samples, 1);
a = zeros(throttle_samples, 1);
b = zeros(throttle_samples, 1);
F = zeros(3, 3);

Rg(6) = Rg(5);  Jtg(5) = Jtg(4);    Je_star = Je + Jtg(5) + (Rg(6)^2)*(Jw + 4*M*h^2);

% Initial Guess
% -----------------------------------------------------
ma(1) = .003;
we(1) = 9;
Tb(1) = 0; d_Tb = 0;
% -----------------------------------------------------

x = zeros(3, n);
x(1:3, 1) = [ma(1); we(1); Tb(1)];

toc
hold on
for j = alpha
for i = 1:n
    % ma
    TC = (alpha(j)/85)^2;
    PR(i) = (R*Tm*ma(i))/(Mair*Vm*Patm);
    PRI(i) = (-4.9958)*PR(i)^5 + (5.8832)*PR(i)^4 + (-1.1218)*PR(i)^3 + (-.6579)*PR(i)^2+ (-.1278)*PR(i) + 1.0104;
    d_ma_i = MAX*TC*PRI(i);
    
    n_vol(i) = (24.5*we(i)*ma(i)^2) - (0.167*we(i)*ma(i)) + ( (8.10*10^(-4))*we(i) ) - ( (3.10*10^(-4))*ma(i)^2 ) + (222*ma(i)) + 0.352;
    d_ma_o = c_1*n_vol(i)*ma(i)*we(i);
    
    d_ma = d_ma_i - d_ma_o;
    
    % we
    Ti = (c_2*d_ma_o)/we(i);
    Tf = 0.1056*we(i) + 15.10;
    Tp = (4.3*10^-6)*we(i)^2;
    Tl = Rg(6)*(0 + Ca*(Rg(6)^2)*(h^3)*(we(i)^2) + h*Fr);
    
    Je_star = Je + Jtg(5) + (Rg(6)^2)*(Jw + 4*M*h^2);
    
    d_we = (Ti - Tf - Tp - Tl)/Je_star;
    
    % Tb
    
    %d_Tb = (0 - Tb(i)) /Tau_bv;
    
    
    F(1, 1) = MAX*(j/85)^2*(5*(-4.9958)*c^5*ma(i)^4 + 4*(5.8832)*c^4*ma(i)^3 + 3*(-1.1218)*c^3*ma(i)^2 + 2*(-.6579)*c^2*ma(i) + (-.1278)*c) - (c_1*(3*24.5*we(i)^2*ma(i)^2 - 2*0.167*we(i)^2*ma(i) + 8.10*10^-4*we(i)^2 - 3*3.10*10^-4*we(i)*ma(i)^2 + 2*222*we(i)*ma(i) + 0.352*we(i)) );
    F(1, 2) = -c_1*(2*24.5*we(i)*ma(i)^3 - 2*0.167*we(i)*ma(i)^2 + 2*8.10*10-4*we(i)*ma(i) - 3.10*10^-4*ma(i)^3 + 222*ma(i)^2 + 0.352*ma(i));
    F(1, 3) = 0;
    
    Je_star = Je + Jtg(5) + (Rg(6)^2)*(Jw + 4*M*h^2);
    F(2, 1) = (1/Je_star)*(c_1*c_2*(3*24.5*we(i)*ma(i)^2 - 2*0.167*we(i)*ma(i) + 8.10*10^-4*we(i) - 3*3.10*10^-4*ma(i)^2 + 2*222*ma(i) + 0.352));
    F(2, 2) = (1/Je_star)*( (c_1*c_2*(24.5*ma(i)^3 - 0.167*ma(i)^2 + 8.10*10^-4*ma(i)) - 0.1056 - 2*(4.3*10^-6)*we(i) - 2*Rg(6)*Ca*Rg(6)^2*h^3*we(i)));
    F(2, 3) = -Rg(6);
    
    F(3, 1) = 0;
    F(3, 2) = 0;
    F(3, 3) = -1/Tau_bv;
    
    f = [d_ma; d_we; d_Tb];
    x(1:3, i + 1) = x(1:3, i) - inv(F)*f;
    
    ma(i + 1) = x(1, i + 1);
    we(i + 1) = x(2, i + 1);
    Tb(i + 1) = x(3, i + 1);
    
    % Stopping Rule
    if( (x(1, i + 1) - x(1, i) < error) && (x(2, i + 1) - x(2, i) < error))
        break;
    end
end
toc
min_iter(j) = i;
    a(j) = ma(i);
    b(j) = we(i);
end


% Post Processing --------------------------------

for j = 1:throttle_samples
    if(min_iter(j) > 10) 
        break;
    end
end

plot(a, b, 'O')
plot(a, b, 'red')
title('Equilibrium Points');
xlabel('m_a');
ylabel('\omega_e');

min_iter(1: j)
j
toc
b(43)
%x(1:3, i)
%eigenvalues = eig(F)
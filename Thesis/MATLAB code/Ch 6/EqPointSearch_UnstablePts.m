% Program Designed to be run immediately after running EqPointSearch (not
% designed to be standalone app

% Look for unstable equilibrium points at -we
alpha = 5;
ma = .01669;
we = 43.8;
n2 = 200;

d_ma = 0;
d_we = 0;
d_ma = zeros(n2, 1);
d_we = zeros(n2, 1);

for k = 1:1:n2
    % ma
    TC = (alpha/85)^2;
    PR = (R*Tm*ma)/(Mair*Vm*Patm);
    PRI = (-4.9958)*PR^5 + (5.8832)*PR^4 + (-1.1218)*PR^3 + (-.6579)*PR^2+ (-.1278)*PR + 1.0104;
    d_ma_i = MAX*TC*PRI;
    
    n_vol = (24.5*we*ma^2) - (0.167*we*ma) + ( (8.10*10^(-4))*we ) - ( (3.10*10^(-4))*ma^2 ) + (222*ma) + 0.352;
    d_ma_o = c_1*n_vol*ma*we;
    
    d_ma(k) = d_ma_i - d_ma_o;
    
    % we
    Ti = (c_2*d_ma_o)/we;
    Tf = 0.1056*we + 15.10;
    Tp = (4.3*10^-6)*we^2;
    Tl = Rg(6)*(Ca*(Rg(6)^2)*(h^3)*(we^2) + h*Fr);
    
    Je_star = Je + Jtg(5) + (Rg(6)^2)*(Jw + 4*M*h^2);
    
    d_we(k) = (Ti - Tf - Tp - Tl)/Je_star;
    
    we = we - 1
end
toc

%figure(4)
%plot(1:n2, d_we, 'green')
%title('d_we');

%figure(3)
%plot(1:n2, d_ma, 'red')
%title('d_ma');

figure(2)
plot(d_ma, d_we, '*');
title(['Operating Points wrt \omega_e alpha = ', alpha])
ylabel('d\omega_e');
xlabel('dm_a');
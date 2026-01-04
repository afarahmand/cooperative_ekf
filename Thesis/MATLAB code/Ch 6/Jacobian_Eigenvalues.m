% Computes Eigenvalues and Analyzes Stability of the Jacobian Matrix
% Run this m-file after Equilbrium Point Plotter

%alpha = 85
%0.00470114670659
%43.56481006480795

%alpha = 60
%0.00467033624071
%43.23145107707402

%alpha = 40
%0.00459331634648
%42.39547321296866

%alpha = 25
%0.00437648305663
%40.01957422208155

%alpha = 10
%0.00299988000001
%23.49456305508211

alpha = 10;
ma = a(alpha);
we = b(alpha);

corrections = [0; 0; ma; we; 0];
nominal_controls = [alpha; 0; 0];


c_PR = (R*Tm)/(Patm*Mair*Vm);

car = 1;

Fx(5*car - 4, 5*car - 4) = 0;                Fx(5*car - 4, 5*car - 3) = 0;                Fx(5*car - 4, 5*car - 2) = 0;
Fx(5*car - 4, 5*car - 1) = Rg(5)*h*cos(nominal_controls(3*car - 0)*pi/180);
Fx(5*car - 4, 5*car - 0) = 0;

Fx(5*car - 3, 5*car - 4) = 0;                Fx(5*car - 3, 5*car - 3) = 0;                Fx(5*car - 3, 5*car - 2) = 0;
Fx(5*car - 3, 5*car - 1) = Rg(5)*h*sin(nominal_controls(3*car - 0)*pi/180);
Fx(5*car - 3, 5*car - 0) = 0;

Fx(5*car - 2, 5*car - 4) = 0;                Fx(5*car - 2, 5*car - 3) = 0;
Fx(5*car - 2, 5*car - 2) = MAX*((nominal_controls(3*car - 2)/85)^2)*(5*(-4.9958)*c_PR^5*corrections(5*car - 2, 1)^4 + 4*(5.8832)*c_PR^4*corrections(5*car - 2, 1)^3 + 3*(-1.1218)*c_PR^3*corrections(5*car - 2, 1)^2 + 2*(-.6579)*c_PR^2*corrections(5*car - 2, 1) + (-.1278)*c_PR) - (c_1*(3*24.5*corrections(5*car - 1, 1)^2*corrections(5*car - 2, 1)^2 - 2*0.167*corrections(5*car - 1, 1)^2*corrections(5*car - 2, 1) + 8.10*10^-4*corrections(5*car - 1, 1)^2 - 3*3.10*10^-4*corrections(5*car - 1, 1)*corrections(5*car - 2, 1)^2 + 2*222*corrections(5*car - 1, 1)*corrections(5*car - 2, 1) + 0.352*corrections(5*car - 1, 1)) );
Fx(5*car - 2, 5*car - 1) = -c_1*(2*24.5*corrections(5*car - 1, 1)*corrections(5*car - 2, 1)^3 - 2*0.167*corrections(5*car - 1, 1)*corrections(5*car - 2, 1)^2 + 2*8.10*10-4*corrections(5*car - 1, 1)*corrections(5*car - 2, 1) - 3.10*10^-4*corrections(5*car - 2, 1)^3 + 222*corrections(5*car - 2, 1)^2 + 0.352*corrections(5*car - 2, 1));
Fx(5*car - 2, 5*car - 0) = 0;

Je_star = Je + Jtg(4) + (Rg(5)^2)*(Jw + 4*M*h^2);

Fx(5*car - 1, 5*car - 4) = 0;                Fx(5*car - 1, 5*car - 3) = 0;
Fx(5*car - 1, 5*car - 2) = (c_1*c_2/Je_star)*( 3*24.5*corrections(5*car - 1, 1)*corrections(5*car - 2, 1)^2 - 2*0.167*corrections(5*car - 1, 1)*corrections(5*car - 2, 1) + 8.10*10^-4*corrections(5*car - 1, 1) - 3*3.10*10^-4*corrections(5*car - 2, 1)^2 + 2*222*corrections(5*car - 2, 1) + 0.352 );
Fx(5*car - 1, 5*car - 1) = (1/Je_star)*( (c_1*c_2*(24.5*corrections(5*car - 2, 1)^3 - 0.167*corrections(5*car - 2, 1)^2 + 8.10*10^-4*corrections(5*car - 2, 1) ) - 0.1056 - 2*(4.3*10^-6)*corrections(5*car - 1, 1) - 2*Ca*Rg(5)^3*h^3*corrections(5*car - 1, 1)) );
Fx(5*car - 1, 5*car - 0) = -Rg(5)/Je_star;

Fx(5*car - 0, 5*car - 4) = 0;                Fx(5*car - 0, 5*car - 3) = 0;                Fx(5*car - 0, 5*car - 2) = 0;                Fx(5*car - 0, 5*car - 1) = 0;
Fx(5*car - 0, 5*car - 0) = -1/Tau_bv;

'Operating Point: ', ma, ' ', we
'Eigenvalues: ', eig(Fx)
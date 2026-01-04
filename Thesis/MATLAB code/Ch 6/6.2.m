EQPoint = plot(0.004701, 43.56, '* blue');
hold on

% Draw Box
NOA = plot([0 0], [0 50], '--');
plot([.005 .005], [0 50], '--')
plot([0 .005], [0 0], '--')
plot([0 .005], [50 50], '--')

x = [-0.004; -0.003; -0.002; -0.0003; -0.0002; -0.0001; 10^-20; 0.0001; 0.002; 0.003; 0.004];
y = ones(1, 11)*-43.5;

BoBoA = plot(x, y, 'O black');
plot(x, y, 'red');
axis([-.004 .005 -50 50])

title(['Basin of Attraction for \alpha = 85^0']);
xlabel('m_a [kg]')
ylabel('\omega_e [rad/s]')
legend([NOA EQPoint BoBoA], 'NOA', 'Equilibrium Point', 'Boundary of Basin of Attraction', 'Location', 'Best');
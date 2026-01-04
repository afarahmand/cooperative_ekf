clc
clear

plot(0, 0, 'Color', [1 1 1])
hold on

EQ_Points_x = [0.004601; 0.002288; 0.002063; 0.00202; 0.002009];
EQ_Points_y = [42.45;       11.74;       5.073;       2.341;     0.5915];
EQ_Points_Color = [.5 0 .2;
                                      1 0 1;
                                      1 .5 0;
                                      0 1 0;
                                      .5 0 1];
for i = 1:5
    EQ_Points = plot(EQ_Points_x(i), EQ_Points_y(i), '*', 'Color', EQ_Points_Color(i, :));
end

% Draw Box
NOA = line([0 0], [0 50]);
line([.005 .005], [0 50])
line([0 .005], [0 0])
line([0 .005], [50 50])

x = [-0.004; -0.003; -0.002; -0.0003; -0.0002; -0.0001; 10^-20; 0.0001; 0.002; 0.003; 0.004];
y = ones(1, 11)*-86.5;

BoBoA = plot(x, y, 'O black');
plot(x, y, 'red');
axis([-.006 .006 -150 60])

title(['Boundaries of Basins of Attraction']);
xlabel('m_a [kg]')
ylabel('\omega_e [rad/s]')
legend([EQPoint NOA BoBoA], 'Equilibrium Point', 'Normal Operating Area', 'Boundary of Basin of Attraction', 'Location', 'Best');
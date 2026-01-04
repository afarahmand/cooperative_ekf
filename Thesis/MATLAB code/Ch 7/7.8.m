

x = [0 2 4]
y = [79 79 42]

plot(x, y, 'red')
hold on
grid
plot([2.23 3.85], [75 45], 'blue O')
axis([0 4 40 85])

title('Velocity wrt Time During Maximum Braking')
xlabel('Time [s]')
ylabel('V [MPH]')
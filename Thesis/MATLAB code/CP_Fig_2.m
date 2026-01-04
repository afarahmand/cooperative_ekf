clc
clear

x = [30; 35; 40; 46; 50];
SS = [6.38; 6.79; 7.16; 7.51; 7.86];
TS = [5.19; 5.52; 5.83; 6.13; 6.43];

grid;   hold on;
SSvCCA = plot(x, SS, 'Color', 'black');
plot(x, SS, 'O', 'Color', 'black');
TSvCCA = plot(x, TS, '--', 'Color', 'black');
plot(x, TS, 's', 'Color', 'black');
xlabel('Initial Vehicle Speed [MPH]', 'Color', 'black');
ylabel('Delay [s]');
legend([SSvCCA TSvCCA], 'Stop Sign', 'Traffic Signal', 'Location', 'Best');
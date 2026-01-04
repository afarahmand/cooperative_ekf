x = 0:.001:85;
y = (x/85).^2;

plot(x, y, 'green')
title('Normalized TC wrt Throttle Angle')
xlabel('Throttle Angle [degrees]');
ylabel('Normalized Throttle Characteristics');
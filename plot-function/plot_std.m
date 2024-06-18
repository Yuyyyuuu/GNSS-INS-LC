%% std
stdfile = 'dataset/NavSTD.txt';
std = load(stdfile);

figure()
plot(std(:, 1), std(:, 2:4))
title('Position STD');
xlabel('Time[s]');
ylabel('pos[m]');
legend('X', 'Y', 'Z');
grid("on");

figure()
plot(std(:, 1), std(:, 5:7))
title('Velocity STD');
xlabel('Time[s]');
ylabel('vel[m/s]');
grid("on");
legend('X', 'Y', 'Z');

figure()
plot(std(:, 1), std(:, 8:10))
title('Attitude STD');
xlabel('Time[s]');
ylabel('att[deg]');
grid("on");
legend('X', 'Y', 'Z');

figure()
plot(std(:, 1), std(:, 11:13))
title('GyroBias STD');
xlabel('Time[s]');
ylabel('gb[deg/h]');
grid("on");
legend('X', 'Y', 'Z');

figure()
plot(std(:, 1), std(:, 14:16))
title('AccelBias STD');
xlabel('Time[s]');
ylabel('ab[mGal]');
grid("on");
legend('X', 'Y', 'Z');

figure()
plot(std(:, 1), std(:, 17:19))
title('GyroScale STD');
xlabel('Time[s]');
ylabel('gs[ppm]');
grid("on");
legend('X', 'Y', 'Z');

figure()
plot(std(:, 1), std(:, 20:22));
title('AccelScale STD');
xlabel('Time[s]');
ylabel('as[ppm]');
grid("on");
legend('X', 'Y', 'Z');

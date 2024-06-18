navpath = "dataset/NavResult_ODONHC.nav";
navdata = load(navpath);

% velocity
figure()
plot(navdata(:, 2), navdata(:, 6:8));
title('Velocity');
legend('North', 'East', 'Down');
xlabel('Time[s]');
ylabel('Vel[m/s]');
grid("on");

% attitude
figure()
plot(navdata(:, 2), navdata(:, 9:11));
title('Attitude');
legend('Roll', 'Pitch', 'Yaw');
xlabel('Time[s]');
ylabel('Att[deg]');
grid("on");

% position
D2R=pi/180.0;
R2D=180.0/pi;
blh = navdata(:, 3:5);
blh(:, 1) = blh(:, 1) * D2R;
blh(:, 2) = blh(:, 2) * D2R;
first_blh = blh(1, 1:3);

RM = getRm(first_blh(1));
RN = getRn(first_blh(1));
h = first_blh(2);
DR = diag([RM + h, (RN + h)*cos(first_blh(1)), -1]);

% blh to ned
pos = zeros(size(blh));
for i = 1:size(pos, 1)
    delta_blh = blh(i, :) - first_blh;
    delta_pos = DR * delta_blh';
    pos(i, :) = delta_pos';
end

%% plane position
figure()
plot(pos(:, 2), pos(:, 1));
title('Position');
xlabel('East[m]');
ylabel('North[m]');
grid("on");

%% height
figure()
plot(navdata(:, 2), navdata(:, 5));
title('Height');
xlabel('Time[s]');
ylabel('Height[m]');
grid("on");

%% 三维可视化轨迹图
figure;
plot3(pos(:, 2), pos(:, 1), pos(:, 3), 'b-', 'LineWidth', 2);
xlabel('E[m]');
ylabel('N[m]');
zlabel('U[m]');
grid on;
view(3);





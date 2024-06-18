format long;
disp("Start Loading reference result!")

%% load data （nav文件第一列为0，不需要用）
temp = load('dataset/NavResult_ODONHC.nav');
result_all = temp(:, 2:end);
temp=load('dataset/REF_NAV.nav');
ref = temp(:, 2:end);

%%  航向角平滑
for i=2:size(result_all, 1)
    if (result_all(i,10) - result_all(i-1, 10)) < -180
        result_all(i:end, 10) = result_all(i:end, 10) + 360;
    end
    if (result_all(i,10) - result_all(i-1, 10)) > 180
        result_all(i:end, 10) = result_all(i:end, 10) - 360;
    end
end

for i=2:size(ref, 1)
    if (ref(i,10) - ref(i-1, 10)) < -180
        ref(i:end, 10) = ref(i:end, 10) + 360;
    end
    if (ref(i,10) - ref(i-1, 10)) > 180
        ref(i:end, 10) = ref(i:end, 10) - 360;
    end
end

% figure()
% plot(result_all(:, 10))

%% 找到数据重合部分
res_start = result_all(1, 1);
res_end = result_all(end, 1);
ref_start = ref(1, 1);
ref_end = ref(end, 1);

if (ref_start > res_start)
    starttime = ref_start;
else 
    starttime = res_start;
end
if (res_end > ref_end)
    endtime = ref_end;
else 
    endtime = res_end;
end

% 按照采样间隔取合适的时间
dt = mean(diff(result_all(:, 1)));
time = starttime:dt:endtime;
time = time';


%% 测试结果和参考结果内插到同样的时刻，然后求差
newresult = zeros(size(time, 1), 10);
newref = zeros(size(time, 1), 10);
error = zeros(size(time, 1), 10);
newresult(:, 1) = time;
newref(:, 1) = time;
error(: ,1) = time;

newresult(:, 2:10) = interp1(result_all(:, 1), result_all(:, 2:10), time);
newref(:, 2:10) = interp1(ref(:, 1), ref(:, 2:10), time);
error(:, 2:10) = newresult(:, 2:10) - newref(:, 2:10);

% 航向角误差处理
for i = 1:size(error, 1)
    if error(i, 10) > 180
        error(i, 10) = error(i, 10) - 360;
    end
    if error(i, 10) < -180
        error(i, 10) = error(i, 10) + 360;
    end
end

%% 位置误差转到ned
D2R=pi/180.0;
R2D=180.0/pi;
first_blh = result_all(1, 2:4);
RM = getRm(first_blh(1) * D2R);
RN = getRn(first_blh(1) * D2R);
h = first_blh(2);
DR = diag([RM + h, (RN + h)*cos(first_blh(1) * D2R), -1]);

error(:, 2:3) = error(:, 2:3) * D2R;
for i = 1:size(error, 1)
    delta_pos = DR * (error(i, 2:4)');
    error(i, 2:4) = delta_pos';
end

figure;
plot(error(:,1),error(:,2:4));
title('Position Error');
xlabel('Time[s]');
ylabel('Error[m]');
legend('North', 'East', 'Down');
grid("on");

% 速度误差
figure;
plot(error(:,1),error(:,5:7));
title('Velocity Error');
xlabel('Time[s]');
ylabel('Error[m/s]');
legend('North', 'East', 'Down');
grid("on");

% 姿态误差
figure;
plot(error(:,1),error(:,8:10));
title('Attitude Error');
xlabel('Time[s]');
ylabel('Error[deg]');
legend('Roll', 'Pitch', 'Yaw');
grid("on");



% %% 找到共同的开始时间点
% for m=1:size(ref,1)
%     for n=1:size(result_all,1)
%         if(abs(ref(m,1)-result_all(n,1))<0.001) 
%             break;
%         end
%     end
%     break;
% end
% 
% %% 取相同时间段
% ref = ref(m:end, :);
% result_all = result_all(n:end, :);
% result_size = size(result_all, 1);
% ref_size = size(ref, 1);
% comsize = min(result_size, ref_size);
% com=zeros(comsize, 10);
% 
% x = 1;
% m = 1;
% n = 1;
% while(1)
%    com(x,2:10)=result_all(n,2:10)-ref(m,2:10);
%    com(x,1)=ref(m,1);
%    x=x+1;
%    n=n+1;
%    m=m+1;
%    if((n>result_size)||(m>ref_size))
%        break;
%    end
% end
% for i=1:size(com,1)
%     if(com(i,10)>180)
%         com(i,10)=com(i,10)-360;
%     end
%     if(com(i,10)<-180)
%         com(i,10)=com(i,10)+360;
%     end
% end
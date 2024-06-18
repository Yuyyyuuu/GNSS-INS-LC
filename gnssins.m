
%% define parameters and load process config
param = Param(); 
cfg = ProcessConfig(); 
cfg.Cbv = Euler2DCM(cfg.installangle); 

%% load data
% imudata
% 共7列：时间(GPS周内秒)，三轴陀螺增量(rad)，三轴加表增量(m/s)   IMU采样频率为200Hz
imufid = fopen(cfg.imufilepath); % 读取参数配置中imu数据文件路径
imudata = fread(imufid, [7, inf], 'double'); % 所有数据读入imudata
fclose(imufid);
imudata = imudata'; % imudata转置，使得每一行为一个时刻的数据..
imustarttime = imudata(1, 1); % imu数据开始时间
imuendtime = imudata(end, 1); % imu数据结束时间

% gnss data
% 共7列：时间(GPS周内秒)，纬经高(deg, deg, m)，北东地位置std(m)
gnssdata = load(cfg.gnssfilepath); % 读取参数配置中gnss数据文件路径
gnssdata(:, 2:3) = gnssdata(:, 2:3) * param.D2R; % 经纬度转为rad单位
% GNSS数据中的列数是否小于13。如果小于13，则表示GNSS数据不包含速度信息
if (size(gnssdata, 2) < 13)
    cfg.usegnssvel = false; % 不使用GNSS速度信息
end
gnssstarttime = gnssdata(1, 1); % gnss数据开始时间
gnssendtime = gnssdata(end, 1); % gnss数据结束时间

% odo data
% 共2列：时间（GPS周内秒），ODO速度（m/s）。ODO采样率为200Hz，更新频率可自己调整算法进行修改
ododata = [];
if (cfg.useodonhc) % 先判断要不要启用odonhc
    odofid = fopen(cfg.odofilepath);
    ododata = fread(odofid, [2, inf], 'double');
    fclose(odofid);
    ododata = ododata'; % 转置使得每一行为一个时刻的数据
end


%% save result
navpath = [cfg.outputfolder, '/NavResult'];
if cfg.usegnssvel
    navpath = [navpath, '_GNSSVEL'];
end
if cfg.useodonhc
    navpath = [navpath, '_ODONHC'];
end
navpath = [navpath, '.nav']; % 生成NavResult.nav用于存储结果
navfp = fopen(navpath, 'wt');

imuerrpath = [cfg.outputfolder, '/ImuError.txt']; % 生成ImuError.txt用于存储结果
imuerrfp = fopen(imuerrpath, 'wt');

stdpath = [cfg.outputfolder, '/NavSTD.txt']; % 生成NavSTD.txt用于存储结果
stdfp = fopen(stdpath, 'wt');


%% get process time
% start time and end time
% 开始的时刻选晚的，结束的时刻选早的（在imu和gnss的开始与结束时刻里）
if imustarttime > gnssstarttime
    starttime = imustarttime;
else
    starttime = gnssstarttime;
end
if imuendtime > gnssendtime
    endtime = gnssendtime;
else
    endtime = imuendtime;
end
% 修改参数配置中的开始时刻，将其调整到imu和gnss有数据的时刻
if cfg.starttime < starttime
    cfg.starttime = starttime;
end
cfg.endtime = endtime;
% 设置odonhc的更新时刻
odoupdatetime = 1;
if cfg.useodonhc
    odoupdatetime = ceil(cfg.starttime) + 0.5; % 第一个更新时刻在0.5s时
    num_to_getvel = 20; % 20个历元平均获取odovel
end
% 设置zero的更新时刻
if cfg.usezero
    zeroupdatetime=ceil(cfg.starttime) + 0.5; % 第一个更新时刻在0.5s时
end
% 零速的时段
ranges = [378300, 378390; 378630, 378693;378754, 378760;378837, 378850; 
          378915, 378924; 378982, 378985;379060, 379083;379116, 379201; 
          379340, 379454;379499, 379521;379568, 379684;379722, 379734; 
          379840, 379847;379947, 379986;380035, 380047;380081, 380121; 
          380308, 380361;380389, 380424;381732, 381788;382346, 382389; 
          383045, 383074];
% data in process interval
% 进行数据切片，只保留需要推算时间段内的imu和gnss的数据
imudata = imudata(imudata(:,1) >= cfg.starttime, :);
imudata = imudata(imudata(:,1) <= cfg.endtime, :);
gnssdata = gnssdata(gnssdata(:, 1) >= cfg.starttime, :);
gnssdata = gnssdata(gnssdata(:, 1) <= cfg.endtime, :);


%% for debug
disp("Start GNSS/INS Processing!");
lastprecent = 0; % 显示解算进度所用


%% initialization 
[kf, navstate] = Initialize(cfg);
laststate = navstate;

% data index preprocess
lastimu = imudata(1, :)';
thisimu = imudata(1, :)';
imudt = thisimu(1, 1) - lastimu(1, 1);
% 找到gnss数据中与imu数据最近的一个（且该时刻的gnss数据晚于imu的数据）
gnssindex = 1;
while gnssdata(gnssindex, 1) < thisimu(1, 1)
    gnssindex = gnssindex + 1;
end

odoindex = 1;
if cfg.useodonhc
    % 与gnss数据一样，将odo的数据与imu数据的时刻对齐
    while ododata(odoindex, 1) < thisimu(1, 1) && odoindex < size(ododata, 1)
        odoindex = odoindex + 1;
    end
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%% MAIN PROCEDD PROCEDURE!
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
for imuindex = 2:size(imudata, 1)-1

    %% set value of last state
    lastimu = thisimu;
    laststate = navstate;
    thisimu = imudata(imuindex, :)';
    imudt = thisimu(1, 1) - lastimu(1, 1);


    %% compensate IMU error
    thisimu(2:4, 1) = (thisimu(2:4, 1) - imudt * navstate.gyrbias)./(ones(3, 1) + navstate.gyrscale);
    thisimu(5:7, 1) = (thisimu(5:7, 1) - imudt * navstate.accbias)./(ones(3, 1) + navstate.accscale);

    
    %% adjust GNSS index
    while (gnssindex <= size(gnssdata, 1) && gnssdata(gnssindex, 1) < lastimu(1, 1))
        gnssindex = gnssindex + 1;
    end
    % check whether gnss data is valid
    if (gnssindex > size(gnssdata, 1))
        disp('GNSS file END!');
        break;
    end

    %% determine whether gnss update is required
    if lastimu(1, 1) == gnssdata(gnssindex, 1)
        % do gnss update for the current state
        thisgnss = gnssdata(gnssindex, :)';
        imudt = thisimu(1, 1) - lastimu(1, 1);
        kf = GNSSUpdate(navstate, thisgnss, kf, cfg.antlever, cfg.usegnssvel, lastimu, imudt);
        [kf, navstate] = ErrorFeedback(kf, navstate);
        gnssindex = gnssindex + 1;
        laststate = navstate;
        
        % do propagation for current imu data
        imudt = thisimu(1, 1) - lastimu(1, 1);
        navstate = InsMech(laststate, lastimu, thisimu);
        kf = InsPropagate(navstate, thisimu, imudt, kf, cfg.corrtime);
    elseif (lastimu(1, 1) <= gnssdata(gnssindex, 1) && thisimu(1, 1) > gnssdata(gnssindex, 1))
        
        % ineterpolate imu to gnss time
        [firstimu, secondimu] = interpolate(lastimu, thisimu, gnssdata(gnssindex, 1));
        
        % do propagation for first imu
        imudt = firstimu(1, 1) - lastimu(1, 1);
        navstate = InsMech(laststate, lastimu, firstimu);
        kf = InsPropagate(navstate, firstimu, imudt, kf, cfg.corrtime);

        % do gnss update
        thisgnss = gnssdata(gnssindex, :)';
        kf = GNSSUpdate(navstate, thisgnss, kf, cfg.antlever, cfg.usegnssvel, firstimu, imudt);
        [kf, navstate] = ErrorFeedback(kf, navstate);
        gnssindex = gnssindex + 1;
        laststate = navstate;
        lastimu = firstimu;

        % do propagation for second imu
        imudt = secondimu(1, 1) - lastimu(1, 1);
        navstate = InsMech(laststate, lastimu, secondimu);
        kf = InsPropagate(navstate, secondimu, imudt, kf, cfg.corrtime);
    else
        %% only do propagation
        % INS mechanization
        navstate = InsMech(laststate, lastimu, thisimu);
        % error propagation
        kf = InsPropagate(navstate, thisimu, imudt, kf, cfg.corrtime);
    end


    if cfg.useodonhc
        %% update odo index
        while ododata(odoindex, 1) < thisimu(1, 1) && odoindex < size(ododata, 1)
            odoindex = odoindex + 1;
        end

        %% odonhc udpate
        if (cfg.useodonhc && lastimu(1, 1) <= odoupdatetime && odoupdatetime < thisimu(1, 1))
            startindex = odoindex - round(num_to_getvel / 2);
            endindex = odoindex + round(num_to_getvel / 2);
            if (startindex < 1)
                startindex = 1;
            end
            if (endindex > size(ododata, 1))
                endindex = size(ododata, 1);
            end
           
            % get odovel and update
            [odovel, valid] = getodovel(ododata(startindex:endindex, :), thisimu(1, 1));
            if valid
                odonhc_vel = [odovel; 0; 0];
                kf = ODONHCUpdate(navstate, odonhc_vel, kf, cfg, thisimu, imudt);
                [kf, navstate] = ErrorFeedback(kf, navstate);
            end
            odoupdatetime = odoupdatetime + 1 / cfg.odoupdaterate;
        end
    end
    %% zero update
    if (cfg.usezero && lastimu(1, 1) <= zeroupdatetime && zeroupdatetime < thisimu(1, 1))
        if any(zeroupdatetime >= ranges(:, 1) & zeroupdatetime <= ranges(:, 2))
            kf = ZeroUpdate(navstate, kf, cfg, thisimu, imudt);
            [kf, navstate] = ErrorFeedback(kf, navstate);
        end
        zeroupdatetime = zeroupdatetime + 1 / cfg.zeroupdaterate;
    end


%     %% GNSS position update 
% 
%     if (thisimu(1, 1) >= gnssdata(gnssindex, 1) || abs(thisimu(1, 1) - gnssdata(gnssindex, 1)) < imudt / 3)    
%         t1 = clock;
%         thisgnss = gnssdata(gnssindex, :)';
%         kf = GNSSUpdate(navstate, thisgnss, kf, cfg.antlever, cfg.usegnssvel, thisimu, imudt);
%         [kf, navstate] = ErrorFeedback(kf, navstate);
%         gnssindex = gnssindex + 1;
%     end
    

    %% save data
    % write navresult to file
    nav = zeros(11, 1);
    nav(2, 1) = navstate.time;
    nav(3:5, 1) = [navstate.pos(1) * param.R2D; navstate.pos(2) * param.R2D; navstate.pos(3)];
    nav(6:8, 1) = navstate.vel;
    nav(9:11, 1) = navstate.att * param.R2D;
    fprintf(navfp, '%2d %12.6f %12.8f %12.8f %8.4f %8.4f %8.4f %8.4f %8.4f %8.4f %8.4f \n', nav);

    % write imu error
    imuerror = zeros(13, 1);
    imuerror(1, 1) = navstate.time;
    imuerror(2:4, 1) = navstate.gyrbias * param.R2D * 3600;
    imuerror(5:7, 1) = navstate.accbias * 1e5;
    imuerror(8:10, 1) = navstate.gyrscale * 1e6;
    imuerror(11:13, 1) = navstate.accscale * 1e6;
    fprintf(imuerrfp, '%12.6f %8.4f %8.4f %8.4f %8.4f %8.4f %8.4f %8.4f %8.4f %8.4f %8.4f %8.4f %8.4f \n', imuerror);

    %write state std
    std = zeros(1, 22);
    std(1) = navstate.time;
    for idx=1:21
        std(idx + 1) = sqrt(kf.P(idx, idx));
    end
    std(8:10) = std(8:10) * param.R2D;
    std(11:13) = std(11:13) * param.R2D *3600;
    std(14:16) = std(14:16) * 1e5;
    std(17:22) = std(17:22) * 1e6;
    fprintf(stdfp, '%12.6f %8.6f %8.6f %8.6f %8.6f %8.6f %8.6f %8.6f %8.6f %8.6f %8.6f %8.6f %8.6f %8.6f %8.6f %8.6f %8.6f %8.6f %8.6f %8.6f %8.6f %8.6f \n', std);


    %% print processing information
    if (imuindex / size(imudata, 1) - lastprecent > 0.01) 
        disp("processing " + num2str(floor(imuindex * 100 / size(imudata, 1))) + " %!");
        lastprecent = imuindex / size(imudata, 1);
    end
end

% close file
fclose(imuerrfp);
fclose(navfp);
fclose(stdfp);

disp("GNSS/INS Integration Processing Finished!");


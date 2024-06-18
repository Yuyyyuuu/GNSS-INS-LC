function cfg = ProcessConfig()

    param = Param();

    %% filepath
    cfg.imufilepath = 'dataset/IMU.bin';
    cfg.gnssfilepath = 'dataset/GNSS.txt';
    cfg.odofilepath = 'dataset/ODO.bin';
    cfg.outputfolder = 'dataset';

    %% configure
    cfg.usegnssvel = false;
    cfg.useodonhc = true;
    cfg.usezero = true;

    %% initial information
    cfg.starttime = 378331;
    cfg.endtime = inf;

    cfg.initpos = [30.4317593979; 114.481069167; 13.183]; % [deg, deg, m]
    cfg.initvel = [0; 0; 0]; % [m/s]
    cfg.initatt = [0.714;-0.041;152.851]; % [deg]

    cfg.initposstd = [0.05; 0.05; 0.1]; %
    cfg.initvelstd = [0.05; 0.05; 0.05]; %[m/s]
    cfg.initattstd = [0.1; 0.1; 0.5]; %[deg]

    % cfg.initposstd = [0.05; 0.05; 0.1]; %
    % cfg.initvelstd = [0.05; 0.05; 0.05]; %[m/s]
    % cfg.initattstd = [0.1; 0.1; 0.5]; %[deg]


    cfg.initgyrbias = [-3700;3400;1000]; % [deg/h]
    cfg.initaccbias = [-10000;3500;-6700]; % [mGal]
    cfg.initgyrscale = [10000;1700;-500]; % [ppm]
    cfg.initaccscale = [0;2000;0]; % [ppm]

    cfg.initgyrbiasstd = [50; 50; 50]; % [deg/h]
    cfg.initaccbiasstd = [250; 250; 250]; % [mGal]
    cfg.initgyrscalestd = [1000; 1000; 1000]; % [ppm]
    cfg.initaccscalestd = [1000; 1000; 1000]; % [ppm]

    % cfg.initgyrbiasstd = [50; 50; 50]; % [deg/h]
    % cfg.initaccbiasstd = [250; 250; 250]; % [mGal]
    % cfg.initgyrscalestd = [1000; 1000; 1000]; % [ppm]
    % cfg.initaccscalestd = [1000; 1000; 1000]; % [ppm]

    cfg.gyrarw = 0.24; % [deg/s/sqrt(h)]
    cfg.accvrw = 0.24; % [m/s/sqrt(h)]
    cfg.gyrbiasstd =50; % [deg/h]
    cfg.accbiasstd = 250; % [mGal]
    cfg.gyrscalestd = 1000; % [ppm]
    cfg.accscalestd = 1000; % [ppm]
    cfg.corrtime = 1; % [h]

    % cfg.gyrarw = 0.24; % [deg/s/sqrt(h)]
    % cfg.accvrw = 0.24; % [m/s/sqrt(h)]
    % cfg.gyrbiasstd = 50; % [deg/h]
    % cfg.accbiasstd = 250; % [mGal]
    % cfg.gyrscalestd = 1000; % [ppm]
    % cfg.accscalestd = 1000; % [ppm]
    % cfg.corrtime = 1; % [h]


    % 安装参数
    cfg.antlever = [0.045;0.46;-0.238]; % [m]
    cfg.odolever = [-0.522;-0.47;1.797]; %[m]
    cfg.installangle = [0; -0.2; 1.2]; %[deg]

    % 观测噪声
    cfg.odonhc_measnoise = [1; 1; 1]; % [m/s]
    cfg.zerov_measnoise=[0.01; 0.01; 0.01]; 
    cfg.zeroa_measnoise=[0.01; 0.01; 0.01]; 

    % odo更新频率
    cfg.odoupdaterate=180;
    % zero更新频率
    cfg.zeroupdaterate=180;

    %% convert unit to standard unit (单位转换)
    cfg.initpos(1) = cfg.initpos(1) * param.D2R;
    cfg.initpos(2) = cfg.initpos(2) * param.D2R;
    cfg.initatt = cfg.initatt * param.D2R;

    cfg.initattstd = cfg.initattstd * param.D2R;

    cfg.initgyrbias = cfg.initgyrbias * param.D2R / 3600;
    cfg.initaccbias = cfg.initaccbias * 1e-5;
    cfg.initgyrscale = cfg.initgyrscale * 1e-6;
    cfg.initaccscale = cfg.initaccscale * 1e-6;
    cfg.initgyrbiasstd = cfg.initgyrbiasstd * param.D2R / 3600;
    cfg.initaccbiasstd = cfg.initaccbiasstd * 1e-5;
    cfg.initgyrscalestd = cfg.initgyrscalestd * 1e-6;
    cfg.initaccscalestd = cfg.initaccscalestd * 1e-6;

    cfg.gyrarw = cfg.gyrarw * param.D2R / 60;
    cfg.accvrw = cfg.accvrw / 60;
    cfg.gyrbiasstd = cfg.gyrbiasstd * param.D2R / 3600;
    cfg.accbiasstd = cfg.accbiasstd * 1e-5;
    cfg.gyrscalestd = cfg.gyrscalestd * 1e-6;
    cfg.accscalestd = cfg.accscalestd * 1e-6;
    cfg.corrtime = cfg.corrtime * 3600;

    cfg.installangle = cfg.installangle * param.D2R;

end


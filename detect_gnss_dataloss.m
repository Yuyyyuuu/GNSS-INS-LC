cfg = ProcessConfig();
gnssdata = load(cfg.gnssfilepath);
% 获取时刻序列
time = gnssdata(:, 1);

% 计算时间间隔的差分
time_diff = diff(time);

% 找到缺失时段的索引
missing_indices = find(time_diff > 1);

% 打印缺失时段的起始和结束时刻
for i = 1:numel(missing_indices)
    start_time = time(missing_indices(i)) + 1;
    end_time = time(missing_indices(i) + 1) - 1;
    fprintf('Missing period: from %d to %d   interval: %d\n', start_time, end_time,end_time-start_time);
end
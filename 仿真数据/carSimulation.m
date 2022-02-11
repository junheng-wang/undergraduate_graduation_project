function [wheeledata, Trackeddata] = carSimulation()
% 车辆仿真函数
    clc; close all;
    %% 直接基本参数
    % 车辆初始位置距离雷达的水平距离：dis = 50-100m
    disCarRadar = 50+(100-50)*rand;
    % 轮式车辆直接参数--------------------------------------------------------------
    % 轮式车半径：wheeledRadius = 0.25-0.5m
    % 轮式车的宽度：wheeleWidth = 1.5-2m
    % 轮式车的平均速度：wheeleVelocity = 5-10m/s
    wheeleRadius = 0.25+(0.5-0.25)*rand;
    wheeleWidth = (1.5+(2-1.5)*rand)/2;
    wheeleVelocity = 5+(10-5)*rand;    
    % 履带车辆直接参数--------------------------------------------------------------
    % 履带车轮部分的半径：trackRadius = 0.3-0.6m
    % 履带车水平部分履带长度：trackLength = 5-7m
    % 履带车车宽：trackWidth = 3-4m
    % 履带车的速度：trackVelocity = 4-10m/s
    trackRadius = 0.3+(0.6-0.3)*rand;
    trackLength = 5+(7-5)*rand;
    trackWidth = 3+(4-3)*rand;
    trackVelocity = 4+(10-4)*rand;    
    
    % 雷达直接参数--------------------------------------------------------------
    % 雷达波长：lambda = 0.02m
    % 雷达的距离分辨率：rangeres = 0.01m
    % 雷达的初始位置坐标：radarloc = [0, 0, 10]
    lambda = 0.02;
    rangeres = 0.01;
    radarloc = [0, 0, 10];
    
    %% 将初始化参数写入carData.txt文件
    carfid = fopen('E:\A.毕业设计\地面运动目标雷达特征提取与智能分类\数据集\wheelecarData.txt', 'at');
    fprintf(carfid,'%d,%d,%d,%d \n', wheeleRadius, wheeleWidth, wheeleVelocity, disCarRadar);
    fclose(carfid);
    carfid = fopen('E:\A.毕业设计\地面运动目标雷达特征提取与智能分类\数据集\TrackedcarData.txt', 'at');
    fprintf(carfid,'%d,%d,%d,%d \n', trackRadius, trackLength, trackVelocity, disCarRadar);
    fclose(carfid);
    
    %% 计算运动轨迹以及时频图
    [wheeledata, ~] = wheeledVehicleTrack(wheeleRadius, wheeleWidth, wheeleVelocity, disCarRadar, ...
        lambda, rangeres, radarloc);
       
    [Trackeddata, ~] = trackedVehicleTrack(trackRadius, trackLength, trackVelocity, trackWidth, disCarRadar, ...
        lambda, rangeres, radarloc);

end
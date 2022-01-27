function [] = carSimulation(num)
for ite = 1:num
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
    % 履带车水平部分履带长度：trackLength = 2.5
    % 履带车的速度：trackVelocity = 10
    trackRadius = 0.3+(0.6-0.3)*rand;
    trackLength = 2+(2.5-2)*rand;
    trackVelocity = 4+(10-4)*rand;    
    
    % 雷达直接参数--------------------------------------------------------------
    % 雷达波长：lambda = 0.02m
    % 雷达的距离分辨率：rangeres = 0.01m
    % 雷达的初始位置坐标：radarloc = [0, 0, 10]
    lambda = 0.06;
    rangeres = 0.01;
    radarloc = [0, 0, 10];
    np = 2048;
    
    %% 将初始化参数写入carData.txt文件
    carfid = fopen('E:\R.毕业设计\地面运动目标雷达特征提取与智能分类\数据集\wheelecarData.txt', 'at');
    fprintf(carfid,'%d,%d,%d,%d \n', wheeleRadius, wheeleWidth, wheeleVelocity, disCarRadar);
    carfid = fopen('E:\R.毕业设计\地面运动目标雷达特征提取与智能分类\数据集\TrackedcarData.txt', 'at');
    fprintf(carfid,'%d,%d,%d,%d \n', trackRadius, trackLength, trackVelocity, disCarRadar);
    
    %% 计算运动轨迹以及时频图
    [wheeledata, wheeleT] = wheeledVehicleTrack(wheeleRadius, wheeleWidth, wheeleVelocity, disCarRadar, ...
        lambda, rangeres, radarloc);
    wheelecarStftImg = myStft(wheeledata, wheeleT, np, 'wheeled car');
    figure;
    imshow(wheelecarStftImg);
    tmp = strcat(['E:\R.毕业设计\地面运动目标雷达特征提取与智能分类' ...
        '\数据集\WheeledVehicle\wheelecar'], num2str(ite), '.tif');
    imwrite(wheelecarStftImg, tmp);    
    
    [Trackeddata, TrackedT] = TrackedVehicleTrack(trackRadius, trackLength, trackVelocity, disCarRadar, ...
        lambda, rangeres, radarloc);
    TrackedcarStftImg = myStft(Trackeddata, TrackedT, np, 'Tracked car');
    figure;
    imshow(TrackedcarStftImg);
    tmp = strcat(['E:\R.毕业设计\地面运动目标雷达特征提取与智能分类' ...
        '\数据集\TrackedVehicle\Trackedcar'],num2str(ite),'.tif');
    imwrite(TrackedcarStftImg, tmp);
end
end
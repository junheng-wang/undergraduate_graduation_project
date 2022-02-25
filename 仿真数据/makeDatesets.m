clc; clear; close all;
%% 车辆仿真
% 轮式车、履带车各仿真：num = 30个
num = 30;
% carSimulate：仿真函数，其返回值为：位移距离图像
for ite = 1:num
    [wheeledata, Trackeddata] = carSimulation();
    % 得到回波并添加噪声
    tempw = sum(wheeledata); 
    echoSignalw(ite, :) = awgn(tempw, 15);    % 信噪比30db
    % 保存时域回波信号
    tmp = strcat(['E:\A.毕业设计\地面运动目标雷达特征提取与智能分类' ...
        '\数据集\WheeledVehicle\echoSignalw'], num2str(ite), '.mat');
    tmpechoSignalw = echoSignalw(ite, :);
    save(tmp, 'tmpechoSignalw');

    tempt = sum(Trackeddata); 
    echoSignalt(ite, :) = awgn(tempt, 15);    % 信噪比30db
    % 保存时域回波信号
    tmp = strcat(['E:\A.毕业设计\地面运动目标雷达特征提取与智能分类' ...
        '\数据集\TrackedVehicle\echoSignalt'],num2str(ite),'.mat');
    tmpechoSignalt = echoSignalt(ite, :);
    save(tmp, 'tmpechoSignalt');
end


%% 时频图
% 方法1：使用STFT算法
% wheelecarStftImg = myStft(wheeledata, 0.15, 2048, 'wheeled car');
% figure;
% imshow(wheelecarStftImg);
% tmp = strcat(['E:\R.毕业设计\地面运动目标雷达特征提取与智能分类' ...
%     '\数据集\WheeledVehicle\wheelecar'], num2str(ite), '.tif');
% imwrite(wheelecarStftImg, tmp);
% 
% TrackedcarStftImg = myStft(Trackeddata, 0.15, 2048, 'Tracked car');
% figure;
% imshow(TrackedcarStftImg);
% tmp = strcat(['E:\R.毕业设计\地面运动目标雷达特征提取与智能分类' ...
%     '\数据集\TrackedVehicle\Trackedcar'],num2str(ite),'.tif');
% imwrite(TrackedcarStftImg, tmp);

% 方法2使用小波变换
for ite = 1:num
    TFDiagramw = waveletTFA(echoSignalw(ite, :), 0.15, 2048);
    tmp = strcat(['E:\A.毕业设计\地面运动目标雷达特征提取与智能分类' ...
        '\数据集\WheeledVehicle\wheelecar'], num2str(ite), '.tif');
    imwrite(TFDiagramw, tmp);

    TFDiagramt = waveletTFA(echoSignalt(ite, :), 0.15, 2048);
    tmp = strcat(['E:\A.毕业设计\地面运动目标雷达特征提取与智能分类' ...
        '\数据集\TrackedVehicle\Trackedcar'],num2str(ite),'.tif');
    imwrite(TFDiagramt, tmp);
end

%% 制作路径+类别的txt文件
% 轮式车标签：classw = 1，履带车标签：classt = 2;
classw = 1;
classt = 2;
% 读入图像的路径
for ite = 1:num
    strw = ['E:\A.毕业设计\地面运动目标雷达特征提取与智能分类\数据集\' ...
        'WheeledVehicle\wheelecar', num2str(ite), '.tif'];
    mydata{ite, :} = {strw, classw};
    strt = ['E:\A.毕业设计\地面运动目标雷达特征提取与智能分类\数据集\' ...
        'TrackedVehicle\Trackedcar', num2str(ite), '.tif'];
    mydata{ite+num, :} = {strt, classt};

    strechow = ['E:\A.毕业设计\地面运动目标雷达特征提取与智能分类\数据集\' ...
        'WheeledVehicle\echoSignalw', num2str(ite), '.mat'];
    mydataecho{ite, :} = {strechow, classw};
    strechot = ['E:\A.毕业设计\地面运动目标雷达特征提取与智能分类\数据集\' ...
        'TrackedVehicle\echoSignalt', num2str(ite), '.mat'];
    mydataecho{ite+num, :} = {strechot, classt};
end
% 打乱顺序
numRandom = randperm(num*2);
%
carfid = fopen('E:\A.毕业设计\地面运动目标雷达特征提取与智能分类\数据集\mydataset.txt', 'at');
carfidEcho = fopen('E:\A.毕业设计\地面运动目标雷达特征提取与智能分类\数据集\mydatasetEcho.txt', 'at');
for ite = 1:num*2
    fprintf(carfid, '%s, %d \n', mydata{numRandom(ite),1}{1,1}, mydata{numRandom(ite),1}{1,2});
    fprintf(carfidEcho, '%s, %d \n', mydataecho{numRandom(ite),1}{1,1}, mydataecho{numRandom(ite),1}{1,2});
end
fclose(carfid);
fclose(carfidEcho);











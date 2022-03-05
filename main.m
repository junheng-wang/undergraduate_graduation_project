clc; clear; close all;
%% 划分数据集——训练集70%，测试集：30%
num = 20; % 总数据的数量
trainnum = round(num*0.7);
testnum = num - trainnum;
% 读取txt
% filename = 'E:\A.毕业设计\地面运动目标雷达特征提取与智能分类\数据集\mydataset.txt';
filename = 'E:\A.毕业设计\地面运动目标雷达特征提取与智能分类\数据集\mydatasetEcho.txt';
temp = readtable(filename, 'ReadVariableNames', false, 'Delimiter', ',' );
% 获取训练集标签
trainLabel = temp{1:trainnum, 2};
% 获取测试集标签
testLabel = temp{trainnum+1:num, 2};
% 计算特征并保持
% carfid = fopen('E:\A.毕业设计\地面运动目标雷达特征提取与智能分类\数据集\carFeature.txt', 'at');
carfid = fopen('E:\A.毕业设计\地面运动目标雷达特征提取与智能分类\数据集\carFeature4.txt', 'at');
for ite = 1:num
%     img = double(imread(temp{ite, 1}{1, :}));
    % 第一种特征提取方法：熵信息、灰度共生矩阵
%     [varphi1, varphi2, varphi3, varphi4, varphi5] = feature2(img);
%     fprintf(carfid,'%d,%d,%d,%d,%d\n', varphi1, varphi2, varphi3, varphi4, varphi5);
    % 第二种特征提取方法：矩阵的奇异值——分类结果不行
%     [svdValue1, svdValue2, svdValue3, svdValue4, svdValue5] = feature3(img);
%     fprintf(carfid,'%d,%d,%d,%d,%d\n', svdValue1, svdValue2, svdValue3, svdValue4, svdValue5);
    % 第三种特征提取方法：特征谱
    strechoSignal = temp{ite, 1}{1, :};
    [fValue1, fValue2, fValue3] = feature4(strechoSignal);
    fprintf(carfid,'%d,%d,%d\n', fValue1, fValue2, fValue3);
end
fclose(carfid);

%% 模型评价指标
% 第一：使用libsvm进行分类
filename = 'E:\A.毕业设计\地面运动目标雷达特征提取与智能分类\数据集\carFeature4.txt';
carfeature = load(filename);
% 训练、测试特征并特征归一化
carfeature = carfeature./max(carfeature);
traincarfeature = carfeature(1:trainnum, :);
testcarfeature = carfeature(trainnum+1:num, :);
model = svmtrain(trainLabel, traincarfeature, '-s 0 -t 0 -c 1 -g 0.1');
[predict_label, acc, dec_values] = svmpredict(testLabel, testcarfeature, model);

% ttt = carfeature(:,1);
% scatter(1:60, ttt);
% text(1:60, ttt,arrayfun(@(x)['  ' num2str(x)], [trainLabel; testLabel]', 'UniformOutput',0));

% 分类指标构建

% 参考：https://blog.csdn.net/qigeyonghuming_1/article/details/97934871

% predict_label：是一个1xN的行向量，N是测试图片的张数，
%                例如测试集有919张图片，那么predict_label就是1x919的向量
% num_in_class ：这参数代表了测试集每个类的数目
%                例如我在分类代码中写：num_in_class=[68,136,170,76,178,71,140,80];
%                意味着我的第一类有68张图片，第2类有136张。。。
%                当然该变量也可以不叫num_in_class,你可以自己取名字，
%                只要知道这是同一个东西，把它放在参数2的位置即可。
% name_class   ：这个参数代表了每个类的名字，
%                例如我在分类代码中写name_class={'1','2','3','4','5','6','7','8'};
%                意味着我的第一类叫 1，这个参数三和参数二要对应。
% 
compute_confusion_matrix(testLabel,predict_label);

% num_in_class = [6, 12];
% name_class = {'1', '2'};
% % 第二：绘制混淆矩阵
% [confusion_matrix] = compute_confusion_matrix( ...
%     predict_label', num_in_class, name_class);
% % 第三：绘制ROC值并计算AUC
% [X,Y,T,AUC,OPTROCPT] = perfcurve(testLabel, dec_values, '1');
% %[X,Y] = perfcurve(labels,scores,posclass,'param1', val1,'param2',val2,...)
% %labels:目标标签 scores:决策值 posclass：正类标签
% %'param'和val可以定义X和Y的输出值，具体可以看函数帮助，默认是定义X轴为FPR，Y轴为TPR
% figure;
% plot(X,Y);
% hold on;
% plot(OPTROCPT(1),OPTROCPT(2),'ro');
% xlabel('FPR');
% ylabel('TPR');
% title('输出ROC曲线')
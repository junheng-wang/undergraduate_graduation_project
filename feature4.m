function [fValue1, fValue2, fValue3] = feature4(strechoSignal)
% 计算时域信号的特征谱
inputStruct = load(strechoSignal);
val_names = fieldnames(inputStruct);
echoSignal = getfield(inputStruct, val_names{1});
N = length(echoSignal);
% 窗长W = 1
W = 1;
% 截取信号的窗长P = 1024
P = 1024;
% 第一步：进行滑窗，构建新的信号矩阵
for i = 1:N-P+1
    echo1 = echoSignal(i:P+i-1)';
    if i == 1
        echo = echo1;
    else
        echo = [echo, echo1];
    end
end

% 第二步：求解矩新信号矩阵的自相关矩阵
Recho = echo*echo';

% 第三步：对自相关矩阵进行特征值分解，将特征值按由大到小排序
eigenValue = eig(Recho);
eigenValue = sort(eigenValue, 'descend');


% 特征谱直接作为特征向量往往会由于维度过高而带来计算负担，因此在特征谱的
% 基础上提取4维特征来表征特征谱散布状况

% 散布特征 1：大特征值个数
% 计算贡献率达到98%的特征值个数，即计算目标能量集中程度，98%作为一个经验数据来限定标准。
k = 1;
while sum(eigenValue(1:k))/sum(eigenValue) <= 0.98
    k = k+1;
end
fValue1 = k-1;

% 散布特征 2：特征值归一化和
% 通过计算归一化特征值总和的大小来确定目标的微多普勒信息丰富程度的量。
fValue2 = sum(eigenValue)/max(eigenValue);

% 散布特征 3：最大特征值所占比例
% 较大微动部分能量占总能量的比重。目标自身微动部件产生较强的微动部分的能量比。
fValue3 = max(eigenValue)/sum(eigenValue);





end
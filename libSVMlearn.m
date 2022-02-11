clc; clear; close all;
data = [176, 70; 180, 80; 185, 85; 177, 78; 190, 88; 165, 50;
    161, 45; 163, 47; 160, 44; 166, 49; 168, 50; 162, 44];
label = [1; 1; 1; 1; 1; 1; 2; 2; 2; 2; 2; 2];
model = svmtrain(label, data, '-s 0 -t 2 -c 1 -g 0.1');
testdata = [190, 85; 164, 49; 178, 77; 195,91;
    160, 44; 161, 44; 161, 46; 159, 43; 155, 44];
testdatalabel = [1; 1; 1; 1; 2; 2; 2; 2; 2];
% [predict_label, acc, dec_values] = svmpredict(testdatalabel, testdata, model);
[predict_label, acc, dec_values] = svmpredict(label, data, model);

%% 分类指标构建

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

num_in_class = [4, 5];
name_class = {'1', '2'};
[confusion_matrix] = compute_confusion_matrix( ...
    predict_label', num_in_class, name_class);
 
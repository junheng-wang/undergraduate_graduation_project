function [varphi1, varphi2, varphi3, varphi4, varphi5] = feature2(img)
% , varphi2, varphi3, varphi4, varphi5
% 计算图像熵信息
varphi1 = entropy(img);
% 灰度共生矩阵
glcm = graycomatrix(img);
properties = graycoprops(glcm, 'all');
varphi2 = properties.Contrast;
varphi3 = properties.Correlation;
varphi4 = properties.Energy;
varphi5 = properties.Homogeneity;

% glcm——灰度共生矩阵（m*n*p维）
% properties——要求计算的属性
% 
% 对glcm进行正规化（变换到[0，1]区间）
% graycoprops然后处理正规化后的glcm
% 
% properties有以下几种：
% 'Contrast'——对比度
% 'Correlation'——相关性
% 'Energy'——能量
% 'Homogeneity'——均匀性

end
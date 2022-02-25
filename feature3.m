function [svdValue1, svdValue2, svdValue3, svdValue4, svdValue5] = feature3(img)
% 计算图像矩阵的奇异值
% svd()函数以降序顺序返回矩阵的奇异值
svdValue = svd(img);
svdValue1 = svdValue(1);
svdValue2 = svdValue(2);
svdValue3 = svdValue(3);
svdValue4 = svdValue(4);
svdValue5 = svdValue(5);

end
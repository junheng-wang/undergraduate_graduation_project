function [varphi1, varphi2, varphi3, varphi4, varphi5] = calfeature(img)
% 计算复合不变矩特征

[ir, ic] = size(img);
x = 0:ir-1; y = 0:ic-1;
% 计算00、01、10阶矩
m00 = sum(sum(((x.^0)'*(y.^0)).*img));
m01 = sum(sum(((x.^0)'*y).*img));
m10 = sum(sum((x'*(y.^0)).*img));

% 求图像img质心坐标
x0 = m10/m00; y0 = m01/m00;

% 计算00、03、12、21、30阶中心距
u00 = sum(sum(img));
u02 = sum(sum(((x-x0).^0)'*((y-y0).^2).*img));
u03 = sum(sum(((x-x0).^0)'*((y-y0).^3).*img));
u05 = sum(sum(((x-x0).^0)'*((y-y0).^5).*img));
u11 = sum(sum(((x-x0).^1)'*((y-y0).^1).*img));
u12 = sum(sum(((x-x0).^1)'*((y-y0).^2).*img));
u14 = sum(sum(((x-x0).^1)'*((y-y0).^4).*img));
u20 = sum(sum(((x-x0).^2)'*((y-y0).^0).*img));
u21 = sum(sum(((x-x0).^2)'*((y-y0).^1).*img));
u23 = sum(sum(((x-x0).^2)'*((y-y0).^3).*img));
u30 = sum(sum(((x-x0).^3)'*((y-y0).^0).*img));
u32 = sum(sum(((x-x0).^3)'*((y-y0).^2).*img));
u41 = sum(sum(((x-x0).^4)'*((y-y0).^1).*img));
u50 = sum(sum(((x-x0).^5)'*((y-y0).^0).*img));

% 计算归一化03、12、21、30阶中心距
n02 = u02/(u00^((0+2+2)/2));
n03 = u03/(u00^((0+3+2)/2));
n05 = u05/(u00^((0+5+2)/2));
n11 = u11/(u00^((1+1+2)/2));
n12 = u12/(u00^((1+2+2)/2));
n14 = u14/(u00^((1+4+2)/2));
n20 = u20/(u00^((2+0+2)/2));
n21 = u21/(u00^((2+1+2)/2));
n23 = u23/(u00^((2+3+2)/2));
n30 = u30/(u00^((3+0+2)/2));
n32 = u32/(u00^((3+2+2)/2));
n41 = u41/(u00^((4+1+2)/2));
n50 = u50/(u00^((5+0+2)/2));

% 计算复合不变矩特征
varphi1 = (n30-3*n12)^2+(3*n21-n03)^2;
varphi2 = (n30+n12)^2+(n21+n03)^2;
varphi3 = (n30-3*n12)*(n30+n12)*((n30+n12)^2-3*(n21+n03)^2) + ...
    (3*n21-n03)*(n21+n03)*(3*(n30+n12)^2-(n21+n03)^2);
varphi4 = (3*n21-n03)*(n30+n12)*((n30+n12)^2-3*(n21+n03)^2) - ...
    (n30-3*n12)*(n21+n03)*(3*(n30+n12)^2-(n21+n03)^2);
varphi5 = (n50-10*n32+5*n14-10*(n20*n30-n30*n02-3*n12*n20 + ...
    3*n12*n02-6*n11*n21+2*n11*n03))^2 + (n05-10*n23+5*n41-10*(n02*n03 - ...
    n30*n20-3*n21*n02+3*n21*20-6*n11*n12+2*n11*n30))^2;










end
function [data, T] = wheeledVehicleTrack(wheeleRadius, wheeleWidth, wheeleVelocity, disCarRadar, ...
    lambda, rangeres, radarloc)
% 轮式车角速度：angularVelocity
% 轮式车一周的距离：circum
% 轮式车一个周期的实际时间：T
angularVelocity = wheeleVelocity/wheeleRadius;
circum = 2*wheeleRadius*pi;
T = 2*pi*wheeleRadius/wheeleVelocity;
% 雷达间接参数--------------------------------------------------------------
% 雷达的距离单元数：nr
% 一个距离单元内的采样点个数：np
nr = round(2*sqrt((radarloc(1)+disCarRadar+wheeleRadius)^2+radarloc(2)^2+(radarloc(3))^2)/rangeres);
np = 2048;
% 采样设置------------------------------------------------------------------
% 每个周期的采样个数：nt = 2048
% 每个采样点对应的时间向量：t（取0.15s即150ms时间）
nt = 2048;
t = 0:0.5/nt:0.5-0.5/nt;
% 0.15s前进的距离：movef
movef = 0.5*wheeleVelocity;
% 根据距离单元和采样点构造的二维存储数据矩阵：data
data = zeros(nr, np);
%% 车轮上散射点的运动坐标
coordinateXYZ.P00 = coordinateXYZP0(disCarRadar, wheeleRadius, movef, nt, t);

% 第一个轮胎使用4个散射点进行仿真
[XYZP11, XYZP12, XYZP13, XYZP14] = coordinateXYZP1(wheeleRadius, wheeleWidth, angularVelocity, t);
coordinateXYZ.P11 = XYZP11+[coordinateXYZ.P00(1,:); zeros(1, np); zeros(1, np)];
coordinateXYZ.P12 = XYZP12+[coordinateXYZ.P00(1,:); zeros(1, np); zeros(1, np)];
coordinateXYZ.P13 = XYZP13+[coordinateXYZ.P00(1,:); zeros(1, np); zeros(1, np)];
coordinateXYZ.P14 = XYZP14+[coordinateXYZ.P00(1,:); zeros(1, np); zeros(1, np)];

% 第二个轮胎使用4个散射点进行仿真
[XYZP21, XYZP22, XYZP23, XYZP24] = coordinateXYZP2(wheeleRadius, wheeleWidth, angularVelocity, t);
coordinateXYZ.P21 = XYZP21+[coordinateXYZ.P00(1,:); zeros(1, np); zeros(1, np)];
coordinateXYZ.P22 = XYZP22+[coordinateXYZ.P00(1,:); zeros(1, np); zeros(1, np)];
coordinateXYZ.P23 = XYZP23+[coordinateXYZ.P00(1,:); zeros(1, np); zeros(1, np)];
coordinateXYZ.P24 = XYZP24+[coordinateXYZ.P00(1,:); zeros(1, np); zeros(1, np)];

%% 计算雷达回波
% 主体散射点回波
for k = 1:np
    r_dist00(:,k) = abs(coordinateXYZ.P00(:,k)-radarloc(:));
    distances00(k) = sqrt(r_dist00(1,k).^2+r_dist00(2,k).^2+r_dist00(3,k).^2);
    PHs00 = 1*(exp(-1i*4*pi*distances00(k)/lambda));
    data(floor(distances00(k)/rangeres), k) = data(floor(distances00(k)/rangeres), k) + PHs00;  
end
Weight = 0.6*rand(1,8);
% 第一个轮胎的4个散射点回波

for k = 1:np
    r_dist11(:,k) = abs(coordinateXYZ.P11(:,k)-radarloc(:));
    r_dist12(:,k) = abs(coordinateXYZ.P12(:,k)-radarloc(:));
    r_dist13(:,k) = abs(coordinateXYZ.P13(:,k)-radarloc(:));
    r_dist14(:,k) = abs(coordinateXYZ.P14(:,k)-radarloc(:));
    distances11(k) = sqrt(r_dist11(1,k).^2+r_dist11(2,k).^2+r_dist11(3,k).^2);
    distances12(k) = sqrt(r_dist12(1,k).^2+r_dist12(2,k).^2+r_dist12(3,k).^2);
    distances13(k) = sqrt(r_dist13(1,k).^2+r_dist13(2,k).^2+r_dist13(3,k).^2);
    distances14(k) = sqrt(r_dist14(1,k).^2+r_dist14(2,k).^2+r_dist14(3,k).^2);
    PHs11 = Weight(1)*(exp(-1i*4*pi*distances11(k)/lambda));
    PHs12 = Weight(2)*(exp(-1i*4*pi*distances12(k)/lambda));
    PHs13 = Weight(3)*(exp(-1i*4*pi*distances13(k)/lambda));
    PHs14 = Weight(4)*(exp(-1i*4*pi*distances14(k)/lambda));
    data(floor(distances11(k)/rangeres), k) = data(floor(distances11(k)/rangeres), k) + PHs11;
    data(floor(distances12(k)/rangeres), k) = data(floor(distances12(k)/rangeres), k) + PHs12;
    data(floor(distances13(k)/rangeres), k) = data(floor(distances13(k)/rangeres), k) + PHs13;
    data(floor(distances14(k)/rangeres), k) = data(floor(distances14(k)/rangeres), k) + PHs14;  
end
% 第二个轮胎的5个散射点回波
for k = 1:np
    r_dist21(:,k) = abs(coordinateXYZ.P21(:,k)-radarloc(:));
    r_dist22(:,k) = abs(coordinateXYZ.P22(:,k)-radarloc(:));
    r_dist23(:,k) = abs(coordinateXYZ.P23(:,k)-radarloc(:));
    r_dist24(:,k) = abs(coordinateXYZ.P24(:,k)-radarloc(:));
    distances21(k) = sqrt(r_dist21(1,k).^2+r_dist21(2,k).^2+r_dist21(3,k).^2);
    distances22(k) = sqrt(r_dist22(1,k).^2+r_dist22(2,k).^2+r_dist22(3,k).^2);
    distances23(k) = sqrt(r_dist23(1,k).^2+r_dist23(2,k).^2+r_dist23(3,k).^2);
    distances24(k) = sqrt(r_dist24(1,k).^2+r_dist24(2,k).^2+r_dist24(3,k).^2);
    PHs21 = Weight(5)*(exp(-1i*4*pi*distances21(k)/lambda));
    PHs22 = Weight(6)*(exp(-1i*4*pi*distances22(k)/lambda));
    PHs23 = Weight(7)*(exp(-1i*4*pi*distances23(k)/lambda));
    PHs24 = Weight(8)*(exp(-1i*4*pi*distances24(k)/lambda));
    data(floor(distances21(k)/rangeres), k) = data(floor(distances21(k)/rangeres), k) + PHs21;
    data(floor(distances22(k)/rangeres), k) = data(floor(distances22(k)/rangeres), k) + PHs22;
    data(floor(distances23(k)/rangeres), k) = data(floor(distances23(k)/rangeres), k) + PHs23;
    data(floor(distances24(k)/rangeres), k) = data(floor(distances24(k)/rangeres), k) + PHs24;  
end

% 显示运动范围
figure;
colormap(hot(256));
imagesc([1, np], [0, nr*rangeres], 20*log10(abs(data)+eps));
xlabel('Pulses');
ylabel('Range (m)');
title('Range Profiles');
axis xy;
clim = get(gca,'CLim');
set(gca,'CLim',clim(2) + [-40 0]);
colorbar;
drawnow;
end

% =========================================================================
function XYZP0 = coordinateXYZP0(disCarRadar, wheeleRadius, movef, nt, t)
x0 = disCarRadar:-movef/nt:disCarRadar-movef+movef/nt;
y0 = zeros(1, length(t));
z0 = 2*wheeleRadius+zeros(1, length(t));

XYZP0 = [x0; y0; z0];

end

% =========================================================================
function [XYZP11, XYZP12, XYZP13, XYZP14] = coordinateXYZP1(wheeleRadius, wheeleWidth, angularVelocity, t)
% 随机初始相位
theta = rand*pi/2;
% 第一个散射点P1:初始相位为theta
x11 = wheeleRadius*cos(angularVelocity*t+theta);
y11 = wheeleWidth+zeros(1, length(t));
z11 = wheeleRadius+wheeleRadius*sin(angularVelocity*t+theta);
% 第二个散射点P2:初始相位为theta+pi/2
x12 = wheeleRadius*cos(angularVelocity*t+(theta+pi/2));
y12 = wheeleWidth+zeros(1, length(t));
z12 = wheeleRadius+wheeleRadius*sin(angularVelocity*t+(theta+pi/2));
% 第三个散射点P3:初始相位为theta+pi/2
x13 = wheeleRadius*cos(angularVelocity*t+(theta+pi));
y13 = wheeleWidth+zeros(1, length(t));
z13 = wheeleRadius+wheeleRadius*sin(angularVelocity*t+(theta+pi));
% 第四个散射点P1:初始相位为theta+3*pi/2
x14 = wheeleRadius*cos(angularVelocity*t+(theta+3*pi/2));
y14 = wheeleWidth+zeros(1, length(t));
z14 = wheeleRadius+wheeleRadius*sin(angularVelocity*t+(theta+3*pi/2));

XYZP11 = [x11; y11; z11];
XYZP12 = [x12; y12; z12];
XYZP13 = [x13; y13; z13];
XYZP14 = [x14; y14; z14];

end

% =========================================================================
function [XYZP21, XYZP22, XYZP23, XYZP24] = coordinateXYZP2(wheeleRadius, wheeleWidth, angularVelocity, t)
% 随机初始相位
theta = rand*pi/2;
% 第一个散射点P1:初始相位为theta
x21 = wheeleRadius*cos(angularVelocity*t+theta);
y21 = -wheeleWidth+zeros(1, length(t));
z21 = wheeleRadius+wheeleRadius*sin(angularVelocity*t+theta);
% 第二个散射点P2:初始相位为theta+pi/2
x22 = wheeleRadius*cos(angularVelocity*t+(theta+pi/2));
y22 = -wheeleWidth+zeros(1, length(t));
z22 = wheeleRadius+wheeleRadius*sin(angularVelocity*t+(theta+pi/2));
% 第三个散射点P3:初始相位为theta+pi
x23 = wheeleRadius*cos(angularVelocity*t+(theta+pi));
y23 = -wheeleWidth+zeros(1, length(t));
z23 = wheeleRadius+wheeleRadius*sin(angularVelocity*t+(theta+pi));
% 第四个散射点P1:初始相位为theta+3*pi/2
x24 = wheeleRadius*cos(angularVelocity*t+(theta+3*pi/2));
y24 = -wheeleWidth+zeros(1, length(t));
z24 = wheeleRadius+wheeleRadius*sin(angularVelocity*t+(theta+3*pi/2));

XYZP21 = [x21; y21; z21];
XYZP22 = [x22; y22; z22];
XYZP23 = [x23; y23; z23];
XYZP24 = [x24; y24; z24];

end

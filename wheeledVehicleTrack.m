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
% 每个采样点对应的时间向量：t（将实际周期T转换到1）
nt = 2048;
t = 0:1/nt:1-1/nt;
% 根据距离单元和采样点构造的二维存储数据矩阵：data
data = zeros(nr, np);
%% 车轮上散射点的运动坐标
x00 = disCarRadar:-circum/nt:disCarRadar-circum+circum/nt;
y00 = zeros(1, length(t));
z00 = 2*wheeleRadius+zeros(1, length(t));

% 第一个轮胎使用4个散射点进行仿真
% 第一个散射点P1:初始相位为0
x11 = wheeleRadius*cos(T*angularVelocity*t)+x00;
y11 = wheeleWidth+zeros(1, length(t));
z11 = wheeleRadius+wheeleRadius*sin(T*angularVelocity*t);
% 第二个散射点P2:初始相位为pi/2
x12 = wheeleRadius*cos(T*angularVelocity*t+pi/2)+x00;
y12 = wheeleWidth+zeros(1, length(t));
z12 = wheeleRadius+wheeleRadius*sin(T*angularVelocity*t+pi/2);
% 第三个散射点P3:初始相位为pi
x13 = wheeleRadius*cos(T*angularVelocity*t+pi)+x00;
y13 = wheeleWidth+zeros(1, length(t));
z13 = wheeleRadius+wheeleRadius*sin(T*angularVelocity*t+pi);
% 第四个散射点P1:初始相位为3pi/2
x14 = wheeleRadius*cos(T*angularVelocity*t+3*pi/2)+x00;
y14 = wheeleWidth+zeros(1, length(t));
z14 = wheeleRadius+wheeleRadius*sin(T*angularVelocity*t+3*pi/2);

% 第二个轮胎使用4个散射点进行仿真
% 随机初始相位
theta2 = pi/3;
% 第一个散射点P1:初始相位为0
x21 = wheeleRadius*cos(T*angularVelocity*t+theta2)+x00;
y21 = -wheeleWidth+zeros(1, length(t));
z21 = wheeleRadius+wheeleRadius*sin(T*angularVelocity*t+theta2);
% 第二个散射点P2:初始相位为pi/2
x22 = wheeleRadius*cos(T*angularVelocity*t+pi/2+theta2)+x00;
y22 = -wheeleWidth+zeros(1, length(t));
z22 = wheeleRadius+wheeleRadius*sin(T*angularVelocity*t+pi/2+theta2);
% 第三个散射点P3:初始相位为pi
x23 = wheeleRadius*cos(T*angularVelocity*t+pi+theta2)+x00;
y23 = -wheeleWidth+zeros(1, length(t));
z23 = wheeleRadius+wheeleRadius*sin(T*angularVelocity*t+pi+theta2);
% 第四个散射点P1:初始相位为3pi/2
x24 = wheeleRadius*cos(T*angularVelocity*t+3*pi/2+theta2)+x00;
y24 = -wheeleWidth+zeros(1, length(t));
z24 = wheeleRadius+wheeleRadius*sin(T*angularVelocity*t+3*pi/2+theta2);

coordinateXYZ.P00 = [x00; y00; z00];

coordinateXYZ.P11 = [x11; y11; z11];
coordinateXYZ.P12 = [x12; y12; z12];
coordinateXYZ.P13 = [x13; y13; z13];
coordinateXYZ.P14 = [x14; y14; z14];

coordinateXYZ.P21 = [x21; y21; z21];
coordinateXYZ.P22 = [x22; y22; z22];
coordinateXYZ.P23 = [x23; y23; z23];
coordinateXYZ.P24 = [x24; y24; z24];

%% 计算雷达回波
% 主体散射点回波
for k = 1:np
    r_dist00(:,k) = abs(coordinateXYZ.P00(:,k)-radarloc(:));
    distances00(k) = sqrt(r_dist00(1,k).^2+r_dist00(2,k).^2+r_dist00(3,k).^2);
    PHs00 = 1*(exp(-1i*4*pi*distances00(k)/lambda));
    data(floor(distances00(k)/rangeres), k) = data(floor(distances00(k)/rangeres), k) + PHs00;  
end
Weight = rand(1,8);
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
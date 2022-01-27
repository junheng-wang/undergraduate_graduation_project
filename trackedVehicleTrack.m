function [data, T] = trackedVehicleTrack(trackRadius, trackLength, trackVelocity, disCarRadar, ...
    lambda, rangeres, radarloc)
%% 履带车间接参数
% 履带车履带周长：Circum
% 履带车轮部分角速度：w
% 履带车总周期：T
Circum = 2*pi*trackRadius+2*disCarRadar;
w = trackVelocity/trackRadius;
T = Circum/trackVelocity;

nt = 2048;
t = 0:1/nt:1-1/nt;

% 雷达间接参数--------------------------------------------------------------
% 雷达的距离单元数：nr
% 一个距离单元内的采样点个数：np
% 根据距离单元和采样点构造的二维存储数据矩阵：data
nr = round(2*sqrt((radarloc(1)+trackLength+trackRadius)^2+radarloc(2)^2+(radarloc(3))^2)/rangeres);
np = 2048;
data = zeros(nr, np);

%% 计算散射点的轨迹
% 整体散射点的运动轨迹
coordinateXYZ.P00 = coordinateXYZP0(trackLength, trackRadius, Circum, nt, t);
% 第一个履带4个散射点运动轨迹
coordinateXYZ.P11 = coordinateXYZP11(trackRadius, disCarRadar, Circum, w, Tz, nt, t) + ...
    [coordinateXYZ.P00(1,:); zeros(1, np); zeros(1, np)];
coordinateXYZ.P12 = coordinateXYZP12(trackRadius, disCarRadar, Circum, w, Tz, nt, t) + ...
    [coordinateXYZ.P00(1,:); zeros(1, np); zeros(1, np)];
coordinateXYZ.P13 = coordinateXYZP13(trackRadius, disCarRadar, Circum, w, Tz, nt, t) + ...
    [coordinateXYZ.P00(1,:); zeros(1, np); zeros(1, np)];
coordinateXYZ.P14 = coordinateXYZP14(trackRadius, disCarRadar, Circum, w, Tz, nt, t) + ...
    [coordinateXYZ.P00(1,:); zeros(1, np); zeros(1, np)];
% 第二个履带4个散射点运动轨迹
coordinateXYZ.P21 = coordinateXYZP21(trackRadius, disCarRadar, Circum, w, Tz, nt, t) + ...
    [coordinateXYZ.P00(1,:); zeros(1, np); zeros(1, np)];
coordinateXYZ.P22 = coordinateXYZP22(trackRadius, disCarRadar, Circum, w, Tz, nt, t) + ...
    [coordinateXYZ.P00(1,:); zeros(1, np); zeros(1, np)];
coordinateXYZ.P23 = coordinateXYZP23(trackRadius, disCarRadar, Circum, w, Tz, nt, t) + ...
    [coordinateXYZ.P00(1,:); zeros(1, np); zeros(1, np)];
coordinateXYZ.P24 = coordinateXYZP24(trackRadius, disCarRadar, Circum, w, Tz, nt, t) + ...
    [coordinateXYZ.P00(1,:); zeros(1, np); zeros(1, np)];

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
% 第二个轮胎的4个散射点回波
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
function XYZP0 = coordinateXYZP0(l, r, Circum, nt, t)
x0 = l:-Circum/nt:l-Circum+Circum/nt;
y0 = zeros(1, length(t));
z0 = 2*r+zeros(1, length(t));

XYZP0 = [x0; y0; z0];

end

% =========================================================================
function XYZP11 = coordinateXYZP11(r, lt, Circum, w, Tz, nt, t)
% 散射点P1的第①部分
ratio1 = (1/4)*2*pi*r/Circum;
num1 = round(ratio1*nt);
x11 = r*cos(Tz*w*t(1:num1)+pi);
y11 = zeros(1, num1);
z11 = r+r*sin(Tz*w*t(1:num1)+pi);
% 散射点P1的第②部分
ratio2 = lt/Circum;
num2 = round(ratio2*nt);
x12 = 0:lt/num2:lt-lt/num2;
y12 = zeros(1, num2);
z12 = zeros(1, num2);
% 散射点P1的第③部分
ratio3 = (1/2)*2*pi*r/Circum;
num3 = round(ratio3*nt);
x13 = lt+r*cos(Tz*w*(t(num1+num2+1:num1+num2+num3)-num2*1/nt)+pi);
y13 = zeros(1, num3);
z13 = r+r*sin(Tz*w*(t(num1+num2+1:num1+num2+num3)-num2*1/nt)+pi);
% 散射点P1的第④部分
ratio4 = lt/Circum;
num4 = round(ratio4*nt);
x14 = lt:-lt/num4:lt/num2;
y14 = zeros(1, num4);
z14 = 2*r+zeros(1, num4);
% 散射点P1的第⑤部分
x15 = r*cos(Tz*w*(t(num1+num2+num3+num4+1:nt)- ...
    (num2+num4)*1/nt)+pi);
y15 = zeros(1, nt-num1-num2-num3-num4);
z15 = r+r*sin(Tz*w*(t(num1+num2+num3+num4+1:nt)- ...
    (num2+num4)*1/nt)+pi);

x = [x11, x12, x13, x14, x15];
y = [y11, y12, y13, y14, y15];
z = [z11, z12, z13, z14, z15];

XYZP11 = [x; y; z];

end

function XYZP12 = coordinateXYZP12(r, lt, Circum, w, Tz, nt, t)
% 散射点P2的第①部分
ratio1 = lt/2/Circum;
num1 = round(ratio1*nt);
x11 = (lt/2):lt/2/num1:(lt-lt/2/num1);
y11 = zeros(1, num1);
z11 = zeros(1, num1);
% 散射点P2的第②部分
ratio2 = pi*r/Circum;
num2 = round(ratio2*nt);
x12 = lt+r*cos(Tz*w*(t(num1+1:num1+num2)-num1*1/nt)+3*pi/2);
y12 = zeros(1, num2);
z12 = r+r*sin(Tz*w*(t(num1+1:num1+num2)-num1*1/nt)+3*pi/2);
% 散射点P2的第③部分
ratio3 = lt/Circum;
num3 = round(ratio3*nt);
x13 = lt:-lt/num3:lt/num3;
y13 = zeros(1, num3);
z13 = 2*r+zeros(1, num3);
% 散射点P2的第④部分
ratio4 = pi*r/Circum;
num4 = round(ratio4*nt);
x14 = r*cos(Tz*w*(t(num1+num2+num3+1:num1+num2+num3+num4)- ...
    (num1+num3)*1/nt)+3*pi/2);
y14 = zeros(1, num2);
z14 = r+r*sin(Tz*w*(t(num1+num2+num3+1:num1+num2+num3+num4)- ...
    (num1+num3)*1/nt)+3*pi/2);
% 散射点P2的第⑤部分
x15 = 0:lt/2/(nt-num1-num2-num3-num4):lt/2-lt/2/(nt-num1-num2-num3-num4);
y15 = zeros(1, nt-num1-num2-num3-num4);
z15 = zeros(1, nt-num1-num2-num3-num4);

x = [x11, x12, x13, x14, x15];
y = [y11, y12, y13, y14, y15];
z = [z11, z12, z13, z14, z15];

XYZP12 = [x; y; z];

end

function XYZP13 = coordinateXYZP13(r, lt, Circum, w, Tz, nt, t)
% 散射点P3的第①部分
ratio1 = (1/4)*2*pi*r/Circum;
num1 = round(ratio1*nt);
x11 = lt+r*cos(Tz*w*t(1:num1));
y11 = zeros(1, num1);
z11 = r+r*sin(Tz*w*t(1:num1));
% 散射点P3的第②部分
ratio2 = lt/Circum;
num2 = round(ratio2*nt);
x12 = lt:-lt/num2:lt/num2;
y12 = zeros(1, num2);
z12 = 2*r+zeros(1, num2);
% 散射点P3的第③部分
ratio3 = (1/2)*2*pi*r/Circum;
num3 = round(ratio3*nt);
x13 = r*cos(Tz*w*(t(num1+num2+1:num1+num2+num3)-num2*1/nt));
y13 = zeros(1, num3);
z13 = r+r*sin(Tz*w*(t(num1+num2+1:num1+num2+num3)-num2*1/nt));
% 散射点P3的第④部分
ratio4 = lt/Circum;
num4 = round(ratio4*nt);
x14 = 0:lt/num4:lt-lt/num2;
y14 = zeros(1, num4);
z14 = zeros(1, num4);
% 散射点P3的第⑤部分
x15 = lt+r*cos(Tz*w*(t(num1+num2+num3+num4+1:nt)- ...
    (num2+num4)*1/nt));
y15 = zeros(1, nt-num1-num2-num3-num4);
z15 = r+r*sin(Tz*w*(t(num1+num2+num3+num4+1:nt)- ...
    (num2+num4)*1/nt));

x = [x11, x12, x13, x14, x15];
y = [y11, y12, y13, y14, y15];
z = [z11, z12, z13, z14, z15];

XYZP13 = [x; y; z];

end

function XYZP14 = coordinateXYZP14(r, lt, Circum, w, Tz, nt, t)
% 散射点P4的第①部分
ratio1 = lt/2/Circum;
num1 = round(ratio1*nt);
x11 = (lt/2):-lt/2/num1:lt/2/num1;
y11 = zeros(1, num1);
z11 = 2*r+zeros(1, num1);
% 散射点P4的第②部分
ratio2 = pi*r/Circum;
num2 = round(ratio2*nt);
x12 = r*cos(Tz*w*(t(num1+1:num1+num2)-num1*1/nt)+pi/2);
y12 = zeros(1, num2);
z12 = r+r*sin(Tz*w*(t(num1+1:num1+num2)-num1*1/nt)+pi/2);
% 散射点P4的第③部分
ratio3 = lt/Circum;
num3 = round(ratio3*nt);
x13 = 0:lt/num3:lt-lt/num3;
y13 = zeros(1, num3);
z13 = zeros(1, num3);
% 散射点P4的第④部分
ratio4 = pi*r/Circum;
num4 = round(ratio4*nt);
x14 = lt+r*cos(Tz*w*(t(num1+num2+num3+1:num1+num2+num3+num4)- ...
    (num1+num3)*1/nt)+pi/2);
y14 = zeros(1, num2);
z14 = r+r*sin(Tz*w*(t(num1+num2+num3+1:num1+num2+num3+num4)- ...
    (num1+num3)*1/nt)+pi/2);
% 散射点P4的第⑤部分
x15 = lt:-lt/2/(nt-num1-num2-num3-num4):lt/2+lt/2/(nt-num1-num2-num3-num4);
y15 = zeros(1, nt-num1-num2-num3-num4);
z15 = 2*r+zeros(1, nt-num1-num2-num3-num4);

x = [x11, x12, x13, x14, x15];
y = [y11, y12, y13, y14, y15];
z = [z11, z12, z13, z14, z15];
XYZP14 = [x; y; z];

end

% =========================================================================
function XYZP21 = coordinateXYZP21(r, lt, Circum, w, Tz, nt, t)
% 散射点P1的第①部分
ratio1 = pi*r/Circum;
num1 = round(ratio1*nt);
x21 = r*cos(Tz*w*t(1:num1)+pi/2);
y21 = zeros(1, num1);
z21 = r+r*sin(Tz*w*t(1:num1)+pi/2);
% 散射点P1的第②部分
ratio2 = lt/Circum;
num2 = round(ratio2*nt);
x22 = 0:lt/num2:lt-lt/num2;
y22 = zeros(1, num2);
z22 = zeros(1, num2);
% 散射点P1的第③部分
ratio3 = pi*r/Circum;
num3 = round(ratio3*nt);
x23 = lt+r*cos(Tz*w*(t(num1+num2+1:num1+num2+num3)-num2*1/nt)+pi/2);
y23 = zeros(1, num3);
z23 = r+r*sin(Tz*w*(t(num1+num2+1:num1+num2+num3)-num2*1/nt)+pi/2);
% 散射点P1的第④部分
ratio4 = lt/Circum;
num4 = round(ratio4*nt);
x24 = lt:-lt/num4:lt/num2;
y24 = zeros(1, num4);
z24 = 2*r+zeros(1, num4);
x = [x21, x22, x23, x24]; 
y = [y21, y22, y23, y24];
z = [z21, z22, z23, z24];

XYZP21 = [x; y; z];

end

function XYZP22 = coordinateXYZP22(r, lt, Circum, w, Tz, nt, t)
% 散射点P2的第①部分
ratio1 = lt/Circum;
num1 = round(ratio1*nt);
x21 = 0:lt/num1:(lt-lt/num1);
y21 = zeros(1, num1);
z21 = zeros(1, num1);
% 散射点P2的第②部分
ratio2 = pi*r/Circum;
num2 = round(ratio2*nt);
x22 = lt+r*cos(Tz*w*(t(num1+1:num1+num2)-num1*1/nt)+3*pi/2);
y22 = zeros(1, num2);
z22 = r+r*sin(Tz*w*(t(num1+1:num1+num2)-num1*1/nt)+3*pi/2);
% 散射点P2的第③部分
ratio3 = lt/Circum;
num3 = round(ratio3*nt);
x23 = lt:-lt/num3:lt/num3;
y23 = zeros(1, num3);
z23 = 2*r+zeros(1, num3);
% 散射点P2的第④部分
ratio4 = pi*r/Circum;
num4 = round(ratio4*nt);
x24 = r*cos(Tz*w*(t(num1+num2+num3+1:num1+num2+num3+num4)- ...
    (num1+num3)*1/nt)+3*pi/2);
y24 = zeros(1, num2);
z24 = r+r*sin(Tz*w*(t(num1+num2+num3+1:num1+num2+num3+num4)- ...
    (num1+num3)*1/nt)+3*pi/2);
x = [x21, x22, x23, x24];
y = [y21, y22, y23, y24];
z = [z21, z22, z23, z24];

XYZP22 = [x; y; z];

end

function XYZP23 = coordinateXYZP23(r, lt, Circum, w, Tz, nt, t)
% 散射点P3的第①部分
ratio1 = pi*r/Circum;
num1 = round(ratio1*nt);
x21 = lt+r*cos(Tz*w*t(1:num1)+3*pi/2);
y21 = zeros(1, num1);
z21 = r+r*sin(Tz*w*t(1:num1)+3*pi/2);
% 散射点P3的第②部分
ratio2 = lt/Circum;
num2 = round(ratio2*nt);
x22 = lt:-lt/num2:lt/num2;
y22 = zeros(1, num2);
z22 = 2*r+zeros(1, num2);
% 散射点P3的第③部分
ratio3 = pi*r/Circum;
num3 = round(ratio3*nt);
x23 = r*cos(Tz*w*(t(num1+num2+1:num1+num2+num3)-num2*1/nt)+3*pi/2);
y23 = zeros(1, num3);
z23 = r+r*sin(Tz*w*(t(num1+num2+1:num1+num2+num3)-num2*1/nt)+3*pi/2);

% 散射点P3的第④部分
ratio4 = lt/Circum;
num4 = round(ratio4*nt);
x24 = 0:lt/num4:lt-lt/num2;
y24 = zeros(1, num4);
z24 = zeros(1, num4);
x = [x21, x22, x23, x24];
y = [y21, y22, y23, y24];
z = [z21, z22, z23, z24];

XYZP23 = [x; y; z];

end

function XYZP24 = coordinateXYZP24(r, lt, Circum, w, Tz, nt, t)
% 散射点P4的第①部分
ratio1 = lt/Circum;
num1 = round(ratio1*nt);
x21 = lt:-lt/num1:lt/num1;
y21 = zeros(1, num1);
z21 = 2*r+zeros(1, num1);
% 散射点P4的第②部分
ratio2 = pi*r/Circum;
num2 = round(ratio2*nt);
x22 = r*cos(Tz*w*(t(num1+1:num1+num2)-num1*1/nt)+pi/2);
y22 = zeros(1, num2);
z22 = r+r*sin(Tz*w*(t(num1+1:num1+num2)-num1*1/nt)+pi/2);
% 散射点P4的第③部分
ratio3 = lt/Circum;
num3 = round(ratio3*nt);
x23 = 0:lt/num3:lt-lt/num3;
y23 = zeros(1, num3);
z23 = zeros(1, num3);
% 散射点P4的第④部分
ratio4 = pi*r/Circum;
num4 = round(ratio4*nt);
x24 = lt+r*cos(Tz*w*(t(num1+num2+num3+1:num1+num2+num3+num4)- ...
    (num1+num3)*1/nt)+pi/2);
y24 = zeros(1, num2);
z24 = r+r*sin(Tz*w*(t(num1+num2+num3+1:num1+num2+num3+num4)- ...
    (num1+num3)*1/nt)+pi/2);
x = [x21, x22, x23, x24];
y = [y21, y22, y23, y24];
z = [z21, z22, z23, z24];

XYZP24 = [x; y; z];

end













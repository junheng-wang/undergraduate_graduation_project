function [data, T] = trackedVehicleTrack(trackRadius, trackLength, trackVelocity, trackWidth, disCarRadar, ...
    lambda, rangeres, radarloc)
%% 履带车间接参数
% 履带车履带周长：Circum
% 履带车轮部分角速度：w
% 履带车总周期：T
Circum = 2*pi*trackRadius+2*disCarRadar;
w = trackVelocity/trackRadius;
T = Circum/trackVelocity;
trackWidth =trackWidth+1;
nt = 2048;
t = 0:0.5/nt:0.5-0.5/nt;
% 0.5s前进的距离：movef
movef = 0.5*trackVelocity;

% 雷达间接参数--------------------------------------------------------------
% 雷达的距离单元数：nr
% 一个距离单元内的采样点个数：np
% 根据距离单元和采样点构造的二维存储数据矩阵：data
nr = round(2*sqrt((radarloc(1)+disCarRadar+trackRadius)^2+radarloc(2)^2+(radarloc(3))^2)/rangeres);
np = 2048;
data = zeros(nr, np);

%% 计算散射点的轨迹
% 整体散射点的运动轨迹
coordinateXYZ.P00 = coordinateXYZP0(trackRadius, movef, disCarRadar, nt);
% 第一个履带4个散射点运动轨迹
coordinateXYZ.P11 = coordinateXYZP11(trackRadius, trackWidth, movef, w, nt, t) + ...
    [coordinateXYZ.P00(1,:); zeros(1, np); zeros(1, np)];
coordinateXYZ.P12 = coordinateXYZP12(trackRadius, trackLength, trackWidth, movef, w, nt, t) + ...
    [coordinateXYZ.P00(1,:); zeros(1, np); zeros(1, np)];
coordinateXYZ.P13 = coordinateXYZP13(trackRadius, trackLength, trackWidth, movef, w, nt, t) + ...
    [coordinateXYZ.P00(1,:); zeros(1, np); zeros(1, np)];
coordinateXYZ.P14 = coordinateXYZP14(trackRadius, trackLength, trackWidth, movef, w, nt, t) + ...
    [coordinateXYZ.P00(1,:); zeros(1, np); zeros(1, np)];
% 第二个履带4个散射点运动轨迹
coordinateXYZ.P21 = coordinateXYZP21(trackRadius, trackWidth, movef, w, nt, t) + ...
    [coordinateXYZ.P00(1,:); zeros(1, np); zeros(1, np)];
coordinateXYZ.P22 = coordinateXYZP22(trackRadius, trackLength, trackWidth, movef, w, nt, t) + ...
    [coordinateXYZ.P00(1,:); zeros(1, np); zeros(1, np)];
coordinateXYZ.P23 = coordinateXYZP23(trackRadius, trackLength, trackWidth, movef, w, nt, t) + ...
    [coordinateXYZ.P00(1,:); zeros(1, np); zeros(1, np)];
coordinateXYZ.P24 = coordinateXYZP24(trackRadius, trackLength, trackWidth, movef, w, nt, t) + ...
    [coordinateXYZ.P00(1,:); zeros(1, np); zeros(1, np)];

%% 计算雷达回波
% 主体散射点回波
for k = 1:np
    r_dist00(:,k) = abs(coordinateXYZ.P00(:,k)-radarloc(:));
    distances00(k) = sqrt(r_dist00(1,k).^2+r_dist00(2,k).^2+r_dist00(3,k).^2);
    PHs00 = 1*(exp(-1i*4*pi*distances00(k)/lambda));
    data(floor(distances00(k)/rangeres), k) = data(floor(distances00(k)/rangeres), k) + PHs00;  
end
Weight = 0.4+(1-0.4)*rand(1,8);
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
function XYZP0 = coordinateXYZP0(trackRadius, movef, disCarRadar, nt)
x0 = disCarRadar:-movef/nt:disCarRadar-movef+movef/nt;
y0 = zeros(1, nt);
z0 = 2*trackRadius+zeros(1, nt);

XYZP0 = [x0; y0; z0];

end

% =========================================================================
function XYZP11 = coordinateXYZP11(trackRadius, trackWidth, movef, w, nt, t)
% 散射点P1
theta = rand*pi;
tempmove = (3*pi/2-theta)*trackRadius;
if tempmove >= movef
    x11 = trackRadius*cos(w*t+(theta+pi/2));
    y11 = trackWidth + zeros(1, nt);
    z11 = trackRadius+trackRadius*sin(w*t+(theta+pi/2));
    x = x11;
    y = y11;
    z = z11;
else
    num1 = floor(tempmove*nt/movef);
    x11 = trackRadius*cos(w*t(1:num1)+(theta+pi/2));
    y11 = trackWidth + zeros(1, num1);
    z11 = trackRadius+trackRadius*sin(w*t(1:num1)+(theta+pi/2));

    num2 = nt-num1;
    x12 = 0:(movef-tempmove)/num2:(movef-tempmove)-(movef-tempmove)/num2;
    y12 = trackWidth + zeros(1, num2);
    z12 = zeros(1, num2);
    x = [x11, x12];
    y = [y11, y12];
    z = [z11, z12];
end

XYZP11 = [x; y; z];

end

function XYZP12 = coordinateXYZP12(trackRadius, trackLength, trackWidth, movef, w, nt, t)
% 散射点P2
halftrackLength = trackLength/2;
if halftrackLength >= movef
    x11 = halftrackLength:movef/nt:halftrackLength+movef-movef/nt;
    y11 = trackWidth + zeros(1, nt);
    z11 = zeros(1, nt);
    x = x11;
    y = y11;
    z = z11;
elseif (halftrackLength+pi*trackRadius) >= movef
    num1 = floor(halftrackLength*nt/movef);
    x11 = halftrackLength:halftrackLength/num1:trackLength-halftrackLength/num1;
    y11 = trackWidth + zeros(1, num1);
    z11 = zeros(1, num1);

    num2 = nt - num1;
    x12 = trackLength + trackRadius*cos(w*t(1:num2)+3*pi/2);
    y12 = trackWidth + zeros(1, num2);
    z12 = trackRadius+trackRadius*sin(w*t(1:num2)+3*pi/2);
    x = [x11, x12];
    y = [y11, y12];
    z = [z11, z12];
else
    num1 = floor(halftrackLength*nt/movef);
    x11 = halftrackLength:halftrackLength/num1:trackLength-halftrackLength/num1;
    y11 = trackWidth + zeros(1, num1);
    z11 = zeros(1, num1);

    num2 = floor(pi*trackRadius/movef);
    x12 = trackLength + trackRadius*cos(w*t(1:num2)+3*pi/2);
    y12 = trackWidth + zeros(1, num2);
    z12 = trackRadius+trackRadius*sin(w*t(1:num2)+3*pi/2);

    num3 = nt-num1-num2;
    % 剩余的距离
    dist3 = movef-halftrackLength-pi*trackRadius;
    x13 = trackLength:-dist3/num3:trackLength-dist3+dist3/num3;
    y13 = trackWidth + zeros(1, num3);
    z13 = 2*trackRadius+zeros(1, num3);

    x = [x11, x12, x13];
    y = [y11, y12, y13];
    z = [z11, z12, z13];
end
XYZP12 = [x; y; z];

end

function XYZP13 = coordinateXYZP13(trackRadius, trackLength, trackWidth, movef, w, nt, t)
% 散射点P3
randnum = rand;
theta = (randnum<=0.5)*pi/2*rand + (randnum>0.5)*(pi/2*rand+3*pi/2);
tempmove = (randnum<=0.5)*(pi/2-theta)*trackRadius + (randnum>0.5)*(2*pi-theta+pi/2)*trackRadius;
if tempmove >= movef
    x11 = trackRadius*cos(w*t+theta);
    y11 = trackWidth + zeros(1, nt);
    z11 = trackRadius+trackRadius*sin(w*t+theta);
    x = x11;
    y = y11;
    z = z11;
else
    num1 = floor(tempmove*nt/movef);
    x11 = trackRadius*cos(w*t(1:num1)+theta);
    y11 = trackWidth + zeros(1, num1);
    z11 = trackRadius+trackRadius*sin(w*t(1:num1)+theta);

    num2 = nt-num1;
    x12 = trackLength:-(movef-tempmove)/num2:(trackLength-(movef-tempmove))+(movef-tempmove)/num2;
    y12 = trackWidth + zeros(1, num2);
    z12 = 2*trackRadius+zeros(1, num2);
    x = [x11, x12];
    y = [y11, y12];
    z = [z11, z12];
end

XYZP13 = [x; y; z];

end

function XYZP14 = coordinateXYZP14(trackRadius, trackLength, trackWidth, movef, w, nt, t)
% 散射点P4
halftrackLength = trackLength/2;
if halftrackLength >= movef
    x11 = halftrackLength:-movef/nt:halftrackLength-movef+movef/nt;
    y11 = trackWidth + zeros(1, nt);
    z11 = 2*trackRadius+zeros(1, nt);
    x = x11;
    y = y11;
    z = z11;
elseif (halftrackLength+pi*trackRadius) >= movef
    num1 = floor(halftrackLength*nt/movef);
    x11 = halftrackLength:-halftrackLength/num1:halftrackLength/num1;
    y11 = trackWidth + zeros(1, num1);
    z11 = 2*trackRadius+zeros(1, num1);

    num2 = nt - num1;
    x12 = trackRadius*cos(w*t(1:num2)+pi/2);
    y12 = trackWidth + zeros(1, num2);
    z12 = trackRadius+trackRadius*sin(w*t(1:num2)+pi/2);
    x = [x11, x12];
    y = [y11, y12];
    z = [z11, z12];
else
    num1 = floor(halftrackLength*nt/movef);
    x11 = halftrackLength:-halftrackLength/num1:halftrackLength/num1;
    y11 = trackWidth + zeros(1, num1);
    z11 = 2*trackRadius+zeros(1, num1);

    num2 = floor(pi*trackRadius/movef);
    x12 = trackLength + trackRadius*cos(w*t(1:num2)+pi/2);
    y12 = trackWidth + zeros(1, num2);
    z12 = trackRadius+trackRadius*sin(w*t(1:num2)+pi/2);

    num3 = nt-num1-num2;
    % 剩余的距离
    dist3 = movef-halftrackLength-pi*trackRadius;
    x13 = 0:dist3/num3:dist3-dist3/num3;
    y13 = trackWidth + zeros(1, num3);
    z13 = zeros(1, num3);

    x = [x11, x12, x13];
    y = [y11, y12, y13];
    z = [z11, z12, z13];
end
XYZP14 = [x; y; z];

end

% =========================================================================
function XYZP21 = coordinateXYZP21(trackRadius, trackWidth, movef, w, nt, t)
% 散射点P1
theta = rand*pi;
tempmove = (3*pi/2-theta)*trackRadius;
if tempmove >= movef
    x21 = trackRadius*cos(w*t+(theta+pi/2));
    y21 = -trackWidth + zeros(1, nt);
    z21 = trackRadius+trackRadius*sin(w*t+(theta+pi/2));
    x = x21;
    y = y21;
    z = z21;
else
    num1 = floor(tempmove*nt/movef);
    x21 = trackRadius*cos(w*t(1:num1)+(theta+pi/2));
    y21 = -trackWidth + zeros(1, num1);
    z21 = trackRadius+trackRadius*sin(w*t(1:num1)+(theta+pi/2));

    num2 = nt-num1;
    x22 = 0:(movef-tempmove)/num2:(movef-tempmove)-(movef-tempmove)/num2;
    y22 = -trackWidth + zeros(1, num2);
    z22 = zeros(1, num2);
    x = [x21, x22];
    y = [y21, y22];
    z = [z21, z22];
end

XYZP21 = [x; y; z];

end

function XYZP22 = coordinateXYZP22(trackRadius, trackLength, trackWidth, movef, w, nt, t)
% 散射点P2
halftrackLength = trackLength/2;
if halftrackLength >= movef
    x11 = halftrackLength:movef/nt:halftrackLength+movef-movef/nt;
    y11 = -trackWidth + zeros(1, nt);
    z11 = zeros(1, nt);
    x = x11;
    y = y11;
    z = z11;
elseif (halftrackLength+pi*trackRadius) >= movef
    num1 = floor(halftrackLength*nt/movef);
    x11 = halftrackLength:halftrackLength/num1:trackLength-halftrackLength/num1;
    y11 = -trackWidth + zeros(1, num1);
    z11 = zeros(1, num1);

    num2 = nt - num1;
    x12 = trackLength + trackRadius*cos(w*t(1:num2)+3*pi/2);
    y12 = -trackWidth + zeros(1, num2);
    z12 = trackRadius+trackRadius*sin(w*t(1:num2)+3*pi/2);
    x = [x11, x12];
    y = [y11, y12];
    z = [z11, z12];
else
    num1 = floor(halftrackLength*nt/movef);
    x11 = halftrackLength:halftrackLength/num1:trackLength-halftrackLength/num1;
    y11 = -trackWidth + zeros(1, num1);
    z11 = zeros(1, num1);

    num2 = floor(pi*trackRadius/movef);
    x12 = trackLength + trackRadius*cos(w*t(1:num2)+3*pi/2);
    y12 = -trackWidth + zeros(1, num2);
    z12 = trackRadius+trackRadius*sin(w*t(1:num2)+3*pi/2);

    num3 = nt-num1-num2;
    % 剩余的距离
    dist3 = movef-halftrackLength-pi*trackRadius;
    x13 = trackLength:-dist3/num3:trackLength-dist3+dist3/num3;
    y13 = -trackWidth + zeros(1, num3);
    z13 = 2*trackRadius+zeros(1, num3);

    x = [x11, x12, x13];
    y = [y11, y12, y13];
    z = [z11, z12, z13];
end
XYZP22 = [x; y; z];

end

function XYZP23 = coordinateXYZP23(trackRadius, trackLength, trackWidth, movef, w, nt, t)
% 散射点P3
randnum = rand;
theta = (randnum<=0.5)*pi/2*rand + (randnum>0.5)*(pi/2*rand+3*pi/2);
tempmove = (randnum<=0.5)*(pi/2-theta)*trackRadius + (randnum>0.5)*(2*pi-theta+pi/2)*trackRadius;
if tempmove >= movef
    x11 = trackRadius*cos(w*t+theta);
    y11 = -trackWidth + zeros(1, nt);
    z11 = trackRadius+trackRadius*sin(w*t+theta);
    x = x11;
    y = y11;
    z = z11;
else
    num1 = floor(tempmove*nt/movef);
    x11 = trackRadius*cos(w*t(1:num1)+theta);
    y11 = -trackWidth + zeros(1, num1);
    z11 = trackRadius+trackRadius*sin(w*t(1:num1)+theta);

    num2 = nt-num1;
    x12 = trackLength:-(movef-tempmove)/num2:(trackLength-(movef-tempmove))+(movef-tempmove)/num2;
    y12 = -trackWidth + zeros(1, num2);
    z12 = 2*trackRadius+zeros(1, num2);
    x = [x11, x12];
    y = [y11, y12];
    z = [z11, z12];
end

XYZP23 = [x; y; z];

end

function XYZP24 = coordinateXYZP24(trackRadius, trackLength, trackWidth, movef, w, nt, t)
% 散射点P4
halftrackLength = trackLength/2;
if halftrackLength >= movef
    x11 = halftrackLength:-movef/nt:halftrackLength-movef+movef/nt;
    y11 = -trackWidth + zeros(1, nt);
    z11 = 2*trackRadius+zeros(1, nt);
    x = x11;
    y = y11;
    z = z11;
elseif (halftrackLength+pi*trackRadius) >= movef
    num1 = floor(halftrackLength*nt/movef);
    x11 = halftrackLength:-halftrackLength/num1:halftrackLength/num1;
    y11 = -trackWidth + zeros(1, num1);
    z11 = 2*trackRadius+zeros(1, num1);

    num2 = nt - num1;
    x12 = trackRadius*cos(w*t(1:num2)+pi/2);
    y12 = -trackWidth + zeros(1, num2);
    z12 = trackRadius+trackRadius*sin(w*t(1:num2)+pi/2);
    x = [x11, x12];
    y = [y11, y12];
    z = [z11, z12];
else
    num1 = floor(halftrackLength*nt/movef);
    x11 = halftrackLength:-halftrackLength/num1:halftrackLength/num1;
    y11 = -trackWidth + zeros(1, num1);
    z11 = 2*trackRadius+zeros(1, num1);

    num2 = floor(pi*trackRadius/movef);
    x12 = trackLength + trackRadius*cos(w*t(1:num2)+pi/2);
    y12 = -trackWidth + zeros(1, num2);
    z12 = trackRadius+trackRadius*sin(w*t(1:num2)+pi/2);

    num3 = nt-num1-num2;
    % 剩余的距离
    dist3 = movef-halftrackLength-pi*trackRadius;
    x13 = 0:dist3/num3:dist3-dist3/num3;
    y13 = -trackWidth + zeros(1, num3);
    z13 = zeros(1, num3);

    x = [x11, x12, x13];
    y = [y11, y12, y13];
    z = [z11, z12, z13];
end

XYZP24 = [x; y; z];

end










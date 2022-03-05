function tmpechoSignal = myMTI(T, echoSignal)
% 输入参数：
% T - 脉冲重复周期
% echoSignal - 时域回波信号

PRI = 1/T;                              % 脉冲重复频率
N = 1024;
Nmultiple = 2;                          % 这个值决定了频率轴的长短，因为频率响应是以2π为周期的，一般取1就可以
Frequency = (-Nmultiple*N/2:1:Nmultiple*N/2-1)/N*PRI;   % 频率轴采样

% =================二次相消器=================
% K这个值在-2~2之间取值
K = 1.5;
% 构造二次相消器传递函数H
H = 1-K*exp(-1i*2*pi*Frequency*T)+exp(-1i*2*pi*Frequency*T).^2;
% =================计算信号频谱=================
echoSignal_fft = fft(echoSignal);
% =================MTI滤波=================
echoSignal_filter = echoSignal_fft.*H;

tmpechoSignal = ifft(echoSignal_filter);

figure
plot(Frequency, abs(H))
grid on
xlabel('频率（Hz）')
ylabel('幅度响应(dB)')
title('MTI频率响应曲线（二次相消）')

% %------%%%%一次相消器%%%%------- 
% H=1-exp(-1i*2*pi*Frequency*T);
% figure
% plot(Frequency,abs(H))
% grid on
% xlabel('频率（Hz）')
% ylabel('幅度响应(dB)')
% title('MTI频率响应曲线(一次相消)')

end
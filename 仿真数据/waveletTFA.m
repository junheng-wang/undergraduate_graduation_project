function TFDiagram = waveletTFA(echoSignal, T, N)
%% 小波时频图绘制
close all;
t = 0:T/N:T-T/N;
% 选用带宽参数和中心频率均为4的复morlet小波
wavename = 'cmor4-4';
% 尺度序列的长度，即scal的长度
totalscal = 256;
% 该函数能求出以wavename命名的母小波的中心频率
fc = centfrq(wavename);

cparam = 2*fc*totalscal;
a = totalscal:-1:1;
scal = cparam./a;
% 该函数实现连续小波变换，s为输入信号，scal为尺度，wavename为小波名称
coefs = cwt(echoSignal, scal, wavename, 1/N);
% 该函数能将尺度转换为实际频率，其中scal为尺度，wavename为小波名称，1/fs为采样周期
f = scal2frq(scal, wavename, 1/(N*2))/2;
figure;
imagesc(t, f, abs(coefs));
colormap(jet); colorbar;
xlabel('时间 t/s'); ylabel('频率 f/Hz'); title('小波时频图');
% 以图象形式输出时频谱
TFDiagram = mat2gray(abs(coefs)*20);

end
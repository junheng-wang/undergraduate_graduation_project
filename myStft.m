function [carStft, carStftImg] = myStft(data, T, np, str)
% micro-Doppler signature 微多普勒信号
% average over range cells 平均超范围单元格
out = sum(data);
out = awgn(out, 12);
dT = T/np; F = 1/dT;
wd = 256; wdd2 = wd/2; wdd8 = wd/8;
ns = np/wd;

% 计算时频微多普勒特征
for k = 1:ns
    sig(1:wd, 1) = out(1, (k-1)*wd+1:(k-1)*wd+wd);
    TMP = stft(sig, 16);
    TF2(:, (k-1)*wdd8+1:(k-1)*wdd8+wdd8) = TMP(:, 1:8:wd);
end
TF = TF2;
disp('Calculating shifted segments of TF distribution ...') % 计算TF分布的移位段
TF1 = zeros(size(TF));
for k = 1:ns-1
    sig(1:wd, 1) = out(1,(k-1)*wd+1+wdd2:(k-1)*wd+wd+wdd2);
    TMP = stft(sig, 16);
    TF1(:, (k-1)*wdd8+1:(k-1)*wdd8+wdd8) = TMP(:,1:8:wd);
end
disp('Removing edge effects ...') % 去除边缘效果
for k = 1:ns-1
    TF(:, k*wdd8-8:k*wdd8+8) = TF1(:, (k-1)*wdd8+wdd8/2-8:(k-1)*wdd8+wdd8/2+8);
end

figure;
colormap(jet(256));
imagesc([0,T], [-F/2,F/2], 20*log10(fftshift(abs(TF), 1)+eps));
xlabel('Time (s)');
ylabel('Doppler (Hz)');
title('Micro-Doppler Signature of ', str);
% axis xy;
clim = get(gca,'CLim');
set(gca,'CLim',clim(2) + [-45 0]);
colorbar
drawnow

carStft = 20*log10(fftshift(abs(TF), 1)+eps);
% carStftImg = mat2gray(im2double(fftshift(abs(TF))));
carStftImg = fftshift(abs(TF));

end



function TF = stft(f, wd)  
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% 
% short-time Fourier transform with Guassian window 
% 带高斯窗的短时傅里叶变换
% Input: 
%      f - signal (real or complex) 
%      wd - std. deviation (sigma) of the Gaussian function 
%           高斯函数的标准偏差（σ）
% Output: 
%      TF - time-frequency distribution 
%           时频分布
% By V.C. Chen 
% 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% 
cntr = length(f)/2; 
sigma = wd; 
fsz = length(f); 
z = exp(-([1:fsz]-fsz/2).^2/(2*sigma^2))/(sqrt(2*pi)*sigma); 
for m=1:fsz 
    mm = m-cntr+fsz/2; 
    if (mm <= fsz && mm >= 1) 
        gwin(m) = z(mm); 
    else
        if (mm > fsz) 
            gwin(m) = z(rem(mm,fsz)); 
        else
            if(mm < 1)
                mm1 = mm+fsz;
                gwin(m) = z(mm1);
            end 
        end 
    end 
end 
winsz = length(gwin); 
x = zeros(1, fsz+winsz); % zero padding 
x(winsz/2+1:winsz/2+fsz) = f; 
for j = 1:fsz
    for k = 1:winsz
        X(k) = gwin(k)*x(j+k);   
    end
    TF(:,j) = fft(X).'; 
end
end
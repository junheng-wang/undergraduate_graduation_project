% % 预测标签，每一类的数目，类别数目
% function [confusion_matrix]=compute_confusion_matrix(predict_label,num_in_class,name_class)
% % predict_label为一维行向量
% % num_in_class代表每一类的个数
% % name_class代表类名
% num_class = length(num_in_class);
% num_in_class = [0 num_in_class];
% 
% confusion_matrix = size(num_class, num_class);
%  
% for ci = 1:num_class
%     for cj = 1:num_class
%         c_start = sum(num_in_class(1:ci))+1;
%         c_end = sum(num_in_class(1:ci+1));
%         % 统计对应标签个数,注意此处的predict_label可能是数值组成的向量，不是字符串组成的向量
%         summer = size(find(predict_label(c_start:c_end) == cj),2); 
%         % confusion_matrix(ci,cj)=summer/num_in_class(ci+1);%这个是显示正确率，
%         confusion_matrix(ci,cj) = summer;%这个是显示图片有多少张，
%     end
% end
%  
% draw_cm(confusion_matrix,name_class,num_class);
%  
% end

function compute_confusion_matrix(actual,detected)
% https://blog.csdn.net/zhaomengszu/article/details/
% 56283832?ops_request_misc=%257B%2522request%255Fid%2
% 522%253A%2522164592702616780271967215%2522%252C%2522
% scm%2522%253A%252220140713.130102334.pc%255Fall.%252
% 2%257D&request_id=164592702616780271967215&biz_id=
% 0&utm_medium=distribute.pc_search_result.none-task-blog
% -2~all~first_rank_ecpm_v1~rank_v31_ecpm-1-56283832.pc_
% search_result_cache&utm_term=matlab%E6%B7%B7%E6%B7%86%E
% 7%9F%A9%E9%98%B5&spm=1018.2226.3001.4187
[mat, ~] = confusionmat(actual,detected);
 
% mat = rand(10);           %# A 5-by-5 matrix of random values from 0 to 1
% mat(3,3) = 0;            %# To illustrate
% mat(5,2) = 0;            %# To illustrate
imagesc(mat);            %# Create a colored plot of the matrix values
colormap(flipud(gray));  %# Change the colormap to gray (so higher values are
                         %#   black and lower values are white)
 
textStrings = num2str(mat(:),'%0.02f');  %# Create strings from the matrix values
textStrings = strtrim(cellstr(textStrings));  %# Remove any space padding
 
[x,y] = meshgrid(1:2);   %# Create x and y coordinates for the strings
hStrings = text(x(:),y(:),textStrings(:),...      %# Plot the strings
                'HorizontalAlignment','center');
midValue = mean(get(gca,'CLim'));  %# Get the middle value of the color range
textColors = repmat(mat(:) > midValue,1,3);  %# Choose white or black for the
                                             %#   text color of the strings so
                                             %#   they can be easily seen over
                                             %#   the background color
set(hStrings,{'Color'},num2cell(textColors,2));  %# Change the text colors
 
set(gca,'XTick',1:2,...                         %# Change the axes tick marks
        'XTickLabel',{'轮式车','履带车'},...  %#   and tick labels
        'YTick',1:2,...
        'YTickLabel',{'轮式车','履带车'},...
        'TickLength',[0 0]);

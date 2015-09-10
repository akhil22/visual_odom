clear all;close all;
list = dir('/home/akhil/Desktop/visual_odom/seq_0/Left/data');
for i = 3:length(list)
    I = (imread(strcat('/home/akhil/Desktop/visual_odom/seq_0/Left/data/',list(i).name)));
%     imshow(I);
    xi = [499.5446;649.1831;761.4120;257.1301;474.1060;499.5446];
    yi = [235.1361;233.6398;371.3072;372.8036;233.6398;235.1361];
   

    bw = roipoly(I,xi,yi);
%      I(~bw) = 0;
    imwrite(I,strcat('/home/akhil/Desktop/visual_odom/seq_0/left_fast_match/',int2str(i),'.jpg'),'jpg');
end
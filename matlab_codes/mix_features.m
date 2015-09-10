clear all;close all;
% seq no
seq = 'seq_0';

for i=1:124;
feature_file1 = strcat('/home/akhil/Desktop/visual_odom/',seq,'/left_deep_match_features/',int2str(i),'.txt');
feature_file2 = strcat('/home/akhil/Desktop/visual_odom/',seq,'/left_rijvi_match_features/',int2str(i+2),'.txt');
feature_file3 = strcat('/home/akhil/Desktop/visual_odom/',seq,'/left_mix_match_features/',int2str(i),'.txt');

[Y1r X1r Y2r X2r]=textread(feature_file2, '%f %f %f %f', 'headerlines',1);
[X1 Y1 X2 Y2 score index]=textread(feature_file1, '%d %d %d %d %f %d', 'headerlines',1);

xi = [  485.7500
  335.7500
  704.7500
  629.7500
  485.7500]';
yi=[    259.2500
  353.7500
  349.2500
  242.7500
  259.2500]';

in = inpolygon(X1,Y1,xi,yi);
X1 = X1(in);
X2 = X2(in);

Y1 = Y1(in);
Y2 = Y2(in);

f = fopen(feature_file3,'w');

for i = 1:size(X1r,1)
    fprintf(f,'%f %f %f %f\n',X1r(i),Y1r(i),X2r(i),Y2r(i));
end
for i = 1:size(X1,1)
    fprintf(f,'%f %f %f %f\n',X1(i),Y1(i),X2(i),Y2(i));
end
fclose(f);
end
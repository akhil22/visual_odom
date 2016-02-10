seq = 'seq_husky';
for j=359:362
    matchfile = strcat('/home/akhil/visual_odom/',seq,'/left_rijvi_match_features/',int2str(j),'.txt');
     [Y1 X1 Y2 X2]  = textread(matchfile,'%f %f %f %f');
     I1_l = rgb2gray(imread(strcat('/home/akhil/visual_odom/',seq,'/left_rijvi_match/',int2str(j),'.jpg')));
     I2_l = rgb2gray(imread(strcat('/home/akhil/visual_odom/',seq,'/left_rijvi_match/',int2str(j+1),'.jpg')));
     I1 = im2single(I1_l);
     I2 = im2single(I2_l);
     showMatchedFeatures(I1,I2,[X1 Y1],[X2 Y2]);
     pause;
end;
     
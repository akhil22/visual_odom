
seq = 'seq_1';

%list all the files inside the keypoint directory

list = dir(strcat('/home/akhil/Desktop/visual_odom/',seq,'/left_rijvi_keys/'));
for i = 1:size(list)-3 
    j = i+2;
    if j==91
        continue;
    end
keyfile1=strcat('/home/akhil/Desktop/visual_odom/',seq,'/left_rijvi_keys/',int2str(j),'_',int2str(i),'.key')
keyfile2=strcat('/home/akhil/Desktop/visual_odom/',seq,'/left_rijvi_keys/',int2str(j+1),'_',int2str(i),'.key')
matchfile=strcat('/home/akhil/Desktop/visual_odom/',seq,'/left_rijvi_match_features/',int2str(j),'.txt')
[I1 I2]=textread(strcat('/home/akhil/Desktop/visual_odom/',seq,'/left_rijvi_match_keys/',int2str(j),'.txt'), '%d %d', 'headerlines',1);

f1= fopen(keyfile1);
f2= fopen(keyfile2);

g1 = textscan(f1,'%s','delimiter','\n');
g2 = textscan(f2,'%s','delimiter','\n');

f3= fopen(matchfile,'w');
for k = 1:size(I1,1)
    points1=sscanf(g1{1}{2+8*I1(k)},'%f');
    points2=sscanf(g2{1}{2+8*I2(k)},'%f');
    points1
    points2
    fprintf(f3,'%f %f %f %f\n',points1(1),points1(2),points2(1),points2(2));
end
fclose(f3);
fclose(f2);
fclose(f1);
end
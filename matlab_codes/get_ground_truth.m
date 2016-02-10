function [X Y Z] = get_ground_truth(seq)
X = 1;
Z = 1;
file_name = strcat('/home/akhil/visual_odom/',seq,'/ground_truth/00.txt');
fid = fopen(file_name);
tline= fgets(fid);
t = 1;
while ischar(tline)
    temp = sscanf(tline,'%f');
    Rt = [temp(1:4)';temp(5:8)';temp(9:12)'];
    tline = fgets(fid);
   
    X(t) = Rt(1,4);
    Y(t) = Rt(2,4);
    Z(t) = Rt(3,4);
     t = t+1;
end
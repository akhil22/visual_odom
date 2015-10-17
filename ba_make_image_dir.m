
seq = 'seq_0';

%list all the files inside the keypoint directory

list_left = dir(strcat('/home/akhil/visual_odom/',seq,'/left_rijvi_keys/'));
list_right = dir(strcat('/home/akhil/visual_odom/',seq,'/right_rijvi_keys/'));
num_keys = 5;
keyfile = cell(num_keys,1)
for i = 1:size(list_right)/2-3
    if i > size(list_right)/2 -3 - ceil(num_keys/2)
        break;
    end
    j = i+2;
    if j==20
        continue;
    end
for key_num =1:num_keys
    if(key_num == 1)
        keyfile{key_num}=strcat('/home/akhil/visual_odom/',seq,'/left_rijvi_keys/',int2str(j),'_',int2str(j-2),'.key');
        continue;
    end
    if(~mod(key_num,2))
   keyfile{key_num}=strcat('/home/akhil/visual_odom/',seq,'/left_rijvi_keys/',int2str(j+floor(key_num/2)),'_',int2str(j+floor(key_num/2)-2),'.key');
      continue;
    end
keyfile{key_num}=strcat('/home/akhil/visual_odom/',seq,'/right_rijvi_keys/',int2str(j+floor(key_num/2)),'_',int2str(j+floor(key_num/2) - 2),'.key');

end
keyfile
matchfile=strcat('/home/akhil/visual_odom/',seq,'/rijvi_list_keys/',int2str(i),'.txt')
f3= fopen(matchfile,'w');
for key_num = 1:num_keys
%     if key_num < num_keys
%  fprintf(f3,'%s ',keyfile{key_num});
%     else
        fprintf(f3,'%s\n',keyfile{key_num})
%     end

end
end


% [I1 I2]=textread(strcat('/home/akhil/visual_odom/',seq,'/left_rijvi_match_keys/',int2str(j),'.txt'), '%d %d', 'headerlines',1);
% 
% f1= fopen(keyfile1);
% f2= fopen(keyfile2);
% 
% g1 = textscan(f1,'%s','delimiter','\n');
% g2 = textscan(f2,'%s','delimiter','\n');


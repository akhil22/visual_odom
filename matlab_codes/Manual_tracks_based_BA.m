%camera parameters 
cx = 1242/2;
cy = 375/2;
fx = 718.0;
fy = fx;
base_length = 0.54;
K = [fx 0 cx;0 fy cy;0 0 1];
K			=	[718.856 0 607.1928; 0 718.856 185.2157; 0 0 1];
seq = 'seq_0';
num_frames = 6;
%read ground truth
[X Y Z R P Yaw]=textread(strcat('/home/akhil/Desktop/visual_odom/',seq,'/gtruth/exp.txt'), '%f %f %f %f %f %f', 'headerlines',1);
list = dir(strcat('/home/akhil/visual_odom/',seq,'/Left/data/'));
Transformations = zeros(4,4,num_frames);
f_match = zeros(num_frames-1);
matches = zeros(10000,3);
ba_input = zeros(6,10000);
point_index = 1;
match_index = 1;
for j=1:num_frames
    i = j+2;
    if j==1
        Transformations(:,:,j) = [eye(3) zeros(3,1);zeros(1,3),1];
    end
    
    if j == num_frames
        continue;
    end
    matchfile = strcat('/home/akhil/visual_odom/',seq,'/left_mix_match_features/',int2str(j),'.txt');
%      matchfile = strcat('/home/akhil/visual_odom/',seq,'/temp_features/',int2str(j),'.txt');

%     [feature_matches(j).X1 feature_matches(j).Y1 feature_matches(j).X2 feature_matches(j).Y2]  = textread(matchfile,'%f %f %f %f', 'headerlines',1);
    [feature_matches(j).X1 feature_matches(j).Y1 feature_matches(j).X2 feature_matches(j).Y2]  = textread(matchfile,'%f %f %f %f');
    temp = unique([feature_matches(j).X1 feature_matches(j).Y1 feature_matches(j).X2 feature_matches(j).Y2],'rows');
    xi =  [455.7500
  289.2500
  739.2500
  643.2500
  455.7500];
  yi = [257.7500
  362.7500
  361.2500
  250.2500
  257.7500];
  in = inpolygon(temp(:,1), temp(:,2),xi,yi);
  temp = temp(in,:);
    feature_matches(j).X1 = temp(:,1);
    feature_matches(j).Y1 = temp(:,2);
    feature_matches(j).X2 = temp(:,3);
    feature_matches(j).Y2 = temp(:,4);
    disp_image1 = strcat('/home/akhil/visual_odom/',seq,'/left_disparity/',int2str(i),'.png');
    disp_image2 = strcat('/home/akhil/visual_odom/',seq,'/left_disparity/',int2str(i+1),'.png');
     Transformations_temp = getTransformation(disp_image1,disp_image2,matchfile);
     Transformations(:,:,j+1) = Transformations(:,:,j)*Transformations_temp; 
end
point_index = 1;
match_index = 1;
for i=1:num_frames-1
    disparity_file1 = strcat('/home/akhil/visual_odom/',seq,'/left_disparity/',int2str(i+2),'.png');
    disparity_file2 = strcat('/home/akhil/visual_odom/',seq,'/left_disparity/',int2str(i+3),'.png');
     for j=1:size(feature_matches(i).X1,1)
%      for j=1:125

        point3d = get3DPoint3(disparity_file1,[feature_matches(i).X1(j);feature_matches(i).Y1(j)],...
              Transformations,1);
        ba_input(1,point_index) = feature_matches(i).X1(j);
        ba_input(2,point_index) = feature_matches(i).Y1(j);
        ba_input(3,point_index) = point3d(1);
        ba_input(4,point_index) = point3d(2);
        ba_input(5,point_index) = point3d(3);
        ba_input(6,point_index) = -0.1;
        point_index = point_index+1;
        point3d = get3DPoint3(disparity_file2,[feature_matches(i).X2(j);feature_matches(i).Y2(j)],...
              Transformations,1);
        ba_input(1,point_index) = feature_matches(i).X2(j);
        ba_input(2,point_index) = feature_matches(i).Y2(j);
        ba_input(3,point_index) = point3d(1);
        ba_input(4,point_index) = point3d(2);
        ba_input(5,point_index) = point3d(3);
        ba_input(6,point_index) = -0.1;
        point_index = point_index+1;
        matches(i,match_index) = point_index-2;
        matches(i+1,match_index) = point_index-1;
        X_temp = feature_matches(i).X2(j);
        Y_temp = feature_matches(i).Y2(j);
        for k=i+1:num_frames-1
            a = find((feature_matches(k).X1 == X_temp));
            b = find((feature_matches(k).Y1 == Y_temp));
            if(~isempty(intersect(a,b)))
%           if(~isempty(find((a==b)==1)))
                temp_index = intersect(a,b);
%                 temp_index
                disparity_file = strcat('/home/akhil/visual_odom/',seq,'/left_disparity/',int2str(k+2),'.png');
                point3d = get3DPoint3(disparity_file,[feature_matches(k).X2(temp_index(1));feature_matches(k).Y2(temp_index(1))],...
                Transformations,1);
                ba_input(1,point_index) = feature_matches(k).X2(temp_index(1));
                ba_input(2,point_index) = feature_matches(k).Y2(temp_index(1));
                ba_input(3,point_index) = point3d(1);
                ba_input(4,point_index) = point3d(2);
                ba_input(5,point_index) = point3d(3);
                ba_input(6,point_index) = -0.1;
                point_index = point_index+1;
                matches(k+1,match_index) = point_index-1;
                X_temp = feature_matches(k).X2(temp_index(1));
                Y_temp = feature_matches(k).Y2(temp_index(1));
                feature_matches(k).X1(temp_index) = [];
                feature_matches(k).Y1(temp_index) = [];
                feature_matches(k).X2(temp_index) = [];
                feature_matches(k).Y2(temp_index) = [];
            else
                break;
            end
        end
        match_index = match_index+1;
    end
        
end
T = Transformations(1:3,:,:);
P = T
for a=1:num_frames
    P(:,:,a) = [T(1:3,1:3,a)' -T(1:3,1:3,a)'*T(:,4,a)];
end
T = P
matches = matches(1:num_frames,1:match_index-1);
ba_input=ba_input(1:6,1:point_index-1);
point_cloud= ba_input(3:5,:);
save('/home/akhil/visual_odom/matlab_codes/matches2.mat','matches');
save('/home/akhil/visual_odom/matlab_codes/ba_input2.mat','ba_input');
save('/home/akhil/visual_odom/matlab_codes/T2.mat','T');
save('/home/akhil/visual_odom/matlab_codes/point_cloud2.mat','point_cloud');
[feature_matches(1).X1 feature_matches(1).Y1];
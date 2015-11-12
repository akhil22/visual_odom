%camera parameters 
cx = 1242/2;
cy = 375/2;
fx = 718.0;
fy = fx;
base_length = 0.54;
K = [fx 0 cx;0 fy cy;0 0 1];
seq = 'seq_0';
num_frames = 3;
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
    disp_image1 = strcat('/home/akhil/visual_odom/',seq,'/left_disparity/',int2str(i),'.png');
    disp_image2 = strcat('/home/akhil/visual_odom/',seq,'/left_disparity/',int2str(i+1),'.png');
    Transformations_temp = getTransformation(disp_image1,disp_image2,matchfile);
    Transformations(:,:,j+1) = Transformations(:,:,j)*Transformations_temp; 
end
for j=1:num_frames-1
    i = j+2;
    matchfile = strcat('/home/akhil/visual_odom/',seq,'/left_mix_match_features/',int2str(j),'.txt');
    [X1 Y1 X2 Y2]=textread(matchfile, '%f %f %f %f', 'headerlines',1);
    for k=1:100;
        points2d1 = [X1(k);Y1(k)];
        points2d2 = [X2(k);Y2(k)];
        points3d1 = get3Dpoint2(j,points2d1,Transformations);
        points3d2 = get3Dpoint2(j+1,points2d2,Transformations);
        ba_input(1,point_index) = points2d1(1);
        ba_input(2,point_index) = points2d1(2);
        ba_input(3,point_index) = points3d1(1);
        ba_input(4,point_index) = points3d1(2);
        ba_input(5,point_index) = points3d1(3);
        ba_input(6,point_index) = -0.1;
        point_index = point_index + 1;
        ba_input(1,point_index) = points2d2(1);
        ba_input(2,point_index) = points2d2(2);
        ba_input(3,point_index) = points3d2(1);
        ba_input(4,point_index) = points3d2(2);
        ba_input(5,point_index) = points3d2(3);
        ba_input(6,point_index) = -0.1;
        point_index = point_index + 1;
        matches(match_index,1) = point_index-2;
        matches(match_index,2) = point_index-1;
        matches(match_index,3) = j;
        matches(match_index,4) = j+1;
        match_index = match_index+1;
    end
        
end
ba_input=ba_input(:,1:point_index-1);
point_cloud=ba_input(3:5,:);
matches=matches(1:match_index-1,:);
T = Transformations(1:3,:,:);
P = T;
for a=1:num_frames
    P(:,:,a) = [T(1:3,1:3,a)' -T(1:3,1:3,a)'*T(:,4,a)];
end
T = P
save('/home/akhil/visual_odom/matlab_codes/ba_input3.mat','ba_input');
save('/home/akhil/visual_odom/matlab_codes/matches3.mat','matches');
save('/home/akhil/visual_odom/matlab_codes/T3.mat','T');
save('/home/akhil/visual_odom/matlab_codes/point_cloud3.mat','point_cloud');
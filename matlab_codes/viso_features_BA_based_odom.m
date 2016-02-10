%camera parameters 

%read ground truth
[X Y Z R P Yaw]=textread(strcat('/home/akhil/visual_odom/','seq_0','/gtruth/exp.txt'), '%f %f %f %f %f %f', 'headerlines',1);

%initialize segment
segment = 1;
T = eye(4);
% T(1,4) = X(segment);
% T(2,4) = 0;
% T(3,4) = Z(segment);
% th = P(segment);
% dcm = angle2dcm( Yaw(segment), P(segment), R(segment) )
% % T(1,1) = cos(th);
% % T(1,2) = 0;
% % T(1,3) = sin(th);
% % T(2,1) = 0;
% % T(2,2) = 1;
% % T(2,3) = 0;
% % T(3,1) = -sin(th);
% % T(3,2) = 0;
% % T(3,3) = cos(th);
% T(1:3,1:3) = dcm';

num_frames = 5;
len = num_frames;
frame_rate = 2;

%Construct Translation matrix T of initial Pose from ground truth 
cx = 647.0408325195312
cy = 360.7898254394531;
fx = 714.3764038085938;
fy = fx;
base_length = 0.54;
K = [fx 0 cx;0 fy cy;0 0 1];
K			=	[718.856 0 607.1928; 0 718.856 185.2157; 0 0 1];
seq = 'seq_0';
% num_frames = 6;
%read ground truth
[X Y Z R P Yaw]=textread(strcat('/home/akhil/visual_odom/',seq,'/gtruth/exp.txt'), '%f %f %f %f %f %f', 'headerlines',1);
seq = 'seq_00';

list = dir(strcat('/home/akhil/visual_odom/',seq,'/00/image_2/'));
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
     matchfile2 = strcat('/home/akhil/visual_odom/',seq,'/left_mix_match_features/',int2str(j),'.txt');
     matchfile = strcat('/home/akhil/visual_odom/',seq,'/left_rijvi_match_features/',int2str(j+2),'.txt');

%      matchfile = strcat('/home/akhil/visual_odom/',seq,'/temp_features/',int2str(j),'.txt');

%     [feature_matches(j).X1 feature_matches(j).Y1 feature_matches(j).X2 feature_matches(j).Y2]  = textread(matchfile,'%f %f %f %f', 'headerlines',1);
%       [feature_matches(j).X1 feature_matches(j).Y1 feature_matches(j).X2 feature_matches(j).Y2]  = textread(matchfile2,'%f %f %f %f');
      [feature_matches(j).Y1 feature_matches(j).X1 feature_matches(j).Y2 feature_matches(j).X2]  = textread(matchfile,'%f %f %f %f');
 
%     temp = unique([feature_matches(j).X1 feature_matches(j).Y1 feature_matches(j).X2 feature_matches(j).Y2],'rows');
     temp = [feature_matches(j).X1 feature_matches(j).Y1 feature_matches(j).X2 feature_matches(j).Y2];
    
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
%              temp = temp(in,:);
    temp = unique(temp,'rows');
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
disp('done');
for i=1:num_frames-1
    disparity_file1 = strcat('/home/akhil/visual_odom/',seq,'/left_disparity/',int2str(i+2),'.png');
    disparity_file2 = strcat('/home/akhil/visual_odom/',seq,'/left_disparity/',int2str(i+3),'.png');
    for j=1:size(feature_matches(i).X1,1)
%     for j=1:10

        point3d1 = get3DPoint3(disparity_file1,[feature_matches(i).X1(j);feature_matches(i).Y1(j)],...
              Transformations,1);
        point3d2 = get3DPoint3(disparity_file2,[feature_matches(i).X2(j);feature_matches(i).Y2(j)],...
              Transformations,1);
        if isnan(point3d1(1)) || isnan(point3d2(1))
            continue;
        end
        if norm(point3d1-point3d2)> 2 | norm(point3d1-point3d2) < 0.5 % | point3d1(3) > 15 | point3d2(3) > 15
                 continue;
        end
        ba_input(1,point_index) = feature_matches(i).X1(j);
        ba_input(2,point_index) = feature_matches(i).Y1(j);
        ba_input(3,point_index) = point3d1(1);
        ba_input(4,point_index) = point3d1(2);
        ba_input(5,point_index) = point3d1(3);
        ba_input(6,point_index) = -0.1;
        point_index = point_index+1;
        
        
        
        ba_input(1,point_index) = feature_matches(i).X2(j);
        ba_input(2,point_index) = feature_matches(i).Y2(j);
        ba_input(3,point_index) = point3d2(1);
        ba_input(4,point_index) = point3d2(2);
        ba_input(5,point_index) = point3d2(3);
        ba_input(6,point_index) = -0.1;
        point_index = point_index+1;
        matches(i,match_index) = point_index-2;
        matches(i+1,match_index) = point_index-1;
        X_temp = feature_matches(i).X2(j);
        Y_temp = feature_matches(i).Y2(j);
        temp3d = point3d2;
% %      for k = i+1:i+1+frame_rate
%             if k == num_frames-2
%                 continue;
%             end
        for k=i+1:num_frames-1
            a = find((feature_matches(k).X1 == X_temp));
            b = find((feature_matches(k).Y1 == Y_temp));
            if(~isempty(intersect(a,b)))
%           if(~isempty(find((a==b)==1)))
                temp_index = intersect(a,b);
%                 temp_index
                disparity_file = strcat('/home/akhil/visual_odom/',seq,'/left_disparity/',int2str(k+3),'.png');
                point3d = get3DPoint3(disparity_file,[feature_matches(k).X2(temp_index(1));feature_matches(k).Y2(temp_index(1))],...
                Transformations,1);
                if isnan(point3d(1))
                    continue;
                end
                if norm(temp3d-point3d) > 2 | norm(temp3d-point3d) < 0.5 % | point3d(3) > 15 | temp3d(3) > 15
                         continue
                end
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
                temp3d = point3d;
            else
                break;
            end
        end
        match_index = match_index+1;
    end
        
end
T2 = Transformations(1:3,:,:);
P = T2
[X Y Z] = get_ground_truth(seq);
X = X';
Y = Y';
Z = Z';
viso = demo_viso_stereo(1,num_frames);
temp_pose_viso = zeros(4,num_frames);
for i=1:num_frames;
    temp_pose_viso(:,i) = viso{i}*[0;0;0;1];
end
vehicle_positions3 = temp_pose_viso(1:3,:);
z3=vehicle_positions3-[X(segment:segment+len-1)';Y(segment:segment+len-1)';Z(segment:segment+len-1)'];
for a=1:num_frames
%     P(:,:,a) = [T2(1:3,1:3,a)' -T2(1:3,1:3,a)'*T2(:,4,a)];
     P(:,:,a) = [T2(1:3,1:3,a) T2(:,4,a)];

end
for a=1:num_frames
    visot = viso{a};
    P(:,:,a) = [visot(1:3,1:3) visot(1:3,4)];
end
temp_cos = T2;
T2 = P
matches = matches(1:num_frames,1:match_index-1);
ba_input=ba_input(1:6,1:point_index-1);
point_cloud= ba_input(3:5,:);
frame_string = num2str(num_frames);
save(strcat('/home/akhil/visual_odom/matlab_codes/',frame_string,'/matches2.mat'),'matches');
save(strcat('/home/akhil/visual_odom/matlab_codes/',frame_string,'/ba_input2.mat'),'ba_input');
save(strcat('/home/akhil/visual_odom/matlab_codes/',frame_string,'/T2.mat'),'T2');
save(strcat('/home/akhil/visual_odom/matlab_codes/',frame_string,'/point_cloud2.mat'),'point_cloud');
save('/home/akhil/visual_odom/matlab_codes/num_frames2.mat','num_frames');

% cameraRtCW = T2;
% point_obs_index = sparse(matches);
% point_obs_value = ba_input;
% W3D = 10;
% save('/home/akhil/visual_odom/matlab_codes/synthetic_data.mat', 'K', 'cameraRtC2W', 'point_cloud', ...
%             'point_obs_index', 'point_obs_value', 'w3D','N_new');
[feature_matches(1).X1 feature_matches(1).Y1];
cameraRtC2W1 = box_synthetic()
% cameraRtC2W1 = T2;
T3 = cameraRtC2W1;
for i=1:num_frames
    T3(4,:,i) = [0 0 0 1];
end
T2 = temp_cos
for i=1:num_frames
    T2(4,:,i) = [0 0 0 1];
end
for i=2:num_frames
    T3(:,:,i) = inv(T3(:,:,1))*T3(:,:,i);
end
T3(:,:,1) = eye(4);
for i=1:num_frames
    T3(:,:,i) = T*T3(:,:,i);
    T2(:,:,i) = T*T2(:,:,i);
end
T3;
temp_pose = zeros(4,num_frames);
temp_pose_init = zeros(4,num_frames);
% for i=1:num_frames
%     T2(1:3,1:3,i) = T3(1:3,1:3,i);
% end
for i=1:num_frames
temp_pose(:,i) = T2(:,:,i)*[0;0;0;1];
end
for i=1:num_frames
temp_pose_init(:,i) = T3(:,:,i)*[0;0;0;1];
end

vehicle_positions=temp_pose(1:3,:);
vehicle_positions2=temp_pose_init(1:3,:)

% x = temp(1,1,:);
% y = temp(2,1,:);
% z = temp(3,1,:);
% vehicle_positions = [
yaa = vehicle_positions(3,len);
ybb = vehicle_positions(3,1);
xaa = vehicle_positions(1,1);
xbb = vehicle_positions(1,len);
axisx_high = xbb+10;
axisx_low = xaa-10;
if xaa > xbb
    axisx_high = xbb+10;
    axisx_low  = xaa-10;
end
axisy_high = ybb+10;
axisy_low = yaa-10;
if yaa > ybb
    axisy_high = yaa+10;
    axisy_low  = ybb-10;
end
z=vehicle_positions-[X(segment:segment+len-1)';Y(segment:segment+len-1)';Z(segment:segment+len-1)'];
z2 = vehicle_positions2-[X(segment:segment+len-1)';Y(segment:segment+len-1)';Z(segment:segment+len-1)'];
m=max(sqrt(diag(z'*z)));
m2=max(sqrt(diag(z2'*z2)));
figure;
plot(vehicle_positions(1,1:len),vehicle_positions(3,1:len),'+b'); %axis([axisx_low axisx_high axisy_low axisy_high]);
xlabel('X') ;ylabel('Z');title(sprintf('Cosine based trajectory comparision max error = %f',m));
 legend('ground truth','computed trajecotry');
hold on;
yaa = Z(segment);
ybb = Z(segment+len-1,1);
xaa = X(segment);
xbb =  X(segment+len-1,1);;
axisx_high = xbb+10;
axisx_low = xaa-10;
if xaa > xbb
    axisx_high = xaa+10;
    axisx_low  = xbb-10;
end
axisy_high = ybb+10;
axisy_low = yaa-10;
if yaa > ybb
    axisy_high = yaa+10;
    axisy_low  = ybb-10;
end

plot(X(segment:segment+len-1),Z(segment:segment+len-1),'+r'); axis([axisx_low axisx_high axisy_low axisy_high]); xlabel('X') ;ylabel('Z');
% load('/home/akhil/visual_odom/matlab_codes/temp.mat');
% a = -temp(1,1,2:end)
% a = a(:);
% b = -temp(3,1,2:end)
% b = b(:);
%  plot(a,b,'+b'); axis([axisx_low axisx_high axisy_low axisy_high]); xlabel('X') ;ylabel('Z');title(sprintf('trajectory comparision max error = %f',m));
 legend('ground truth','computed trajecotry');
% keyboard;

figure;
plot(X(segment:segment+len-1),Z(segment:segment+len-1),'+r'); axis([axisx_low axisx_high axisy_low axisy_high]); xlabel('X') ;ylabel('Z');
hold on;
yaa = vehicle_positions2(3,len);
ybb = vehicle_positions2(3,1);
xaa = vehicle_positions2(1,1);
xbb = vehicle_positions2(1,len);
axisx_high = xbb+10;
axisx_low = xaa-10;
if xaa > xbb
    axisx_high = xaa+10;
    axisx_low  = xbb-10;
end
axisy_high = ybb+10;
axisy_low = yaa-10;
if yaa > ybb
    axisy_high = yaa+10;
    axisy_low  = ybb-10;
end
plot(vehicle_positions2(1,1:len),vehicle_positions2(3,1:len),'+b'); axis([axisx_low axisx_high axisy_low axisy_high]); xlabel('X') ;ylabel('Z');title(sprintf('BA based trajectory comparision max error = %f',m2));
 legend('ground truth','computed trajecotry');
% load('/home/akhil/visual_odom/matlab_codes/temp.mat');
% a = -temp(1,1,2:end)
% a = a(:);
% b = -temp(3,1,2:end)
% b = b(:);
%  plot(a,b,'+b'); axis([axisx_low axisx_high axisy_low axisy_high]); xlabel('X') ;ylabel('Z');title(sprintf('trajectory comparision max error = %f',m));
 legend('ground truth','computed trajecotry');

% z
% z2
[sqrt(diag(z'*z))]'
[sqrt(diag(z2'*z2))]'

T3

% z2 = vehicle_positions3-[X(segment:segment+len-1)';Y(segment:segment+len-1)';Z(segment:segment+len-1)'];
% m=max(sqrt(diag(z'*z)));
m3=max(sqrt(diag(z3'*z3)));
 figure;
 yaa = Z(segment);
ybb = Z(segment+len-1,1);
xaa = X(segment);
xbb =  X(segment+len-1,1);;
axisx_high = xbb+10;
axisx_low = xaa-10;
if xaa > xbb
    axisx_high = xaa+10;
    axisx_low  = xbb-10;
end
axisy_high = ybb+10;
axisy_low = yaa-10;
if yaa > ybb
    axisy_high = yaa+10;
    axisy_low  = ybb-10;
end
plot(X(segment:segment+len-1),Z(segment:segment+len-1),'+r'); axis([axisx_low axisx_high axisy_low axisy_high]); xlabel('X') ;ylabel('Z');
hold on;
yaa = vehicle_positions3(3,len);
ybb = vehicle_positions3(3,1);
xaa = vehicle_positions3(1,1);
xbb = vehicle_positions3(1,len);
axisx_high = xbb+10;
axisx_low = xaa-10;
if xaa > xbb
    axisx_high = xaa+10;
    axisx_low  = xbb-10;
end
axisy_high = ybb+10;
axisy_low = yaa-10;
if yaa > ybb
    axisy_high = yaa+10;
    axisy_low  = ybb-10;
end

plot(vehicle_positions3(1,1:len),vehicle_positions3(3,1:len),'+b'); axis([axisx_low axisx_high axisy_low axisy_high]); xlabel('X') ;ylabel('Z');title(sprintf('viso based trajectory comparision max error = %f',m3));
 legend('ground truth','computed trajecotry');
% load('/home/akhil/visual_odom/matlab_codes/temp.mat');
% a = -temp(1,1,2:end)
% a = a(:);
% b = -temp(3,1,2:end)
% b = b(:);
%  plot(a,b,'+b'); axis([axisx_low axisx_high axisy_low axisy_high]); xlabel('X') ;ylabel('Z');title(sprintf('trajectory comparision max error = %f',m));
 legend('ground truth','computed trajecotry');
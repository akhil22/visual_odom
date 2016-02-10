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

num_frames = 1800;
len = num_frames;
BA_frames = 10
frame_rate = 2;
% ba_input = zeros(6,30000);
% matches = zeros(BA_frames+1,10000);
%Construct Translation matrix T of initial Pose from ground truth

first_frame = 75;
cx = 647.0408325195312;
cy = 360.7898254394531;
fx = 714.3764038085938;
fy = fx;
base_length = 0.12;
K = [fx 0 cx;0 fy cy;0 0 1];

seq = 'seq_0';
% num_frames = 6;
%read ground truth
T_viso_temp = eye(4,4);
T_BA_temp = eye(4,4);
seq = 'seq_husky';
for i=1:10
    j = (i-1)*BA_frames;
    k = i*BA_frames;
    
    %calculate viso odometry
    viso = demo_viso_stereo(j+first_frame,k+first_frame);
    
    %compute tracks for BA
    tracks = demo_matching_tracking(j+first_frame,k+first_frame);
    point_index = 1;
    matching_index =1;
    ba_input = zeros(6,30000);
    matches = zeros(BA_frames+1,10000);
    
    disp_image = imread(strcat('/home/akhil/visual_odom/seq_husky/left_disparity/',int2str(j+first_frame),'.png'));
    size(disp_image);
    disp_image = zeros(720,1280,BA_frames+1);
    disp_image2 = zeros(1280,720,BA_frames+1);
    for p = 1:BA_frames+1
        disp_image(:,:,p) = imread(strcat('/home/akhil/visual_odom/seq_husky/left_disparity/',int2str(p+j+first_frame-1),'.png'));
        disp_image2(:,:,p) = disp_image(:,:,p)';
    end
    size(disp_image);
    for p = 1:size(tracks,2)
%         disp('new point');
        if size(tracks{p},2) == 0
            continue
        end
        consider = 1;
        prev_rot = [viso{BA_frames+1}(1:3,1:3) viso{BA_frames+1}(1:3,4)]; 
        temp_point = zeros(4,1);
        for t = 1:size(tracks{p},2)
%             disp('new match');
%              point3d = get3DPoint3(disp_image(:,(1280*(BA_frames+1-t)+1):(1280*(BA_frames+2-t))),tracks{p}(:,t));
             point3d = [1;1;1];
            point = tracks{p}(:,t);
            pix_col = point(1);
            pix_row = point(2);
            dispar = double(disp_image2(round(pix_col),round(pix_row),BA_frames+2-t)./256.0);
%               dispar = double(disp_image(round(pix_col),round(pix_row),BA_frames+2-t)./256.0);
            point3d(3) = fx*base_length./dispar;
            point3d(2) = (pix_row-cy)*point3d(3)/fx;
            point3d(1) = (pix_col-cx)*point3d(3)/fx;
            point3d(4) = 1;
           
            temp_point2 = zeros(4,1);
            if t == 1 
                temp_point = point3d;
                continue
            else
                current_rot = [viso{BA_frames+2-t}(1:3,1:3) viso{BA_frames+2-t}(1:3,4)];
                temp_point2 = prev_rot'*current_rot*point3d;
                prev_rot = current_rot;
                point2d = [K zeros(3,1)]*temp_point2;
                point2d = point2d/point2d(3);
            end
            error = norm(temp_point(1:3)-temp_point2(1:3))
            temp_point = point3d;
             temp_point2
             point3d
            if error > 1
                consider = 0;
            end
            
        end
         keyboard
        if consider == 0
            continue
        end
        
        for t = 1:size(tracks{p},2)
%             disp('new match');
%              point3d = get3DPoint3(disp_image(:,(1280*(BA_frames+1-t)+1):(1280*(BA_frames+2-t))),tracks{p}(:,t));
             point3d = [1;1;1];
            point = tracks{p}(:,t);
            pix_col = point(1);
            pix_row = point(2);
            dispar = double(disp_image2(round(pix_col),round(pix_row),BA_frames+2-t)./256.0);
%               dispar = double(disp_image(round(pix_col),round(pix_row),BA_frames+2-t)./256.0);
            point3d(3) = fx*base_length./dispar;
            point3d(2) = (pix_row-cy)*point3d(3)/fx;
            point3d(1) = (pix_col-cx)*point3d(3)/fx;
            point3d(4) = 1;
            ba_input(1,point_index) = tracks{p}(1,t);
            ba_input(2,point_index) = tracks{p}(2,t);
            ba_input(3,point_index) = point3d(1);
            ba_input(4,point_index) = point3d(2);
            ba_input(5,point_index) = point3d(3);
            ba_input(6,point_index) = -0.1;
            matches(BA_frames+2-t,matching_index) = point_index;
            point_index = point_index+1;
            
        end
        matching_index=matching_index+1;
    end
    P = zeros(3,4,BA_frames+1);
    for a=1:BA_frames+1
        visot = viso{a};
        P(:,:,a) = [visot(1:3,1:3) visot(1:3,4)];
    end
    T2 = P;
    matches = matches(1:BA_frames+1,1:matching_index-1);
    ba_input=ba_input(1:6,1:point_index-1);
    point_cloud= ba_input(3:5,1:point_index-1);
    frame_string = num2str(BA_frames+1);
    num_frames = BA_frames+1;
    save(strcat('/home/akhil/visual_odom/matlab_codes/',frame_string,'/matches2.mat'),'matches');
    save(strcat('/home/akhil/visual_odom/matlab_codes/',frame_string,'/ba_input2.mat'),'ba_input');
    save(strcat('/home/akhil/visual_odom/matlab_codes/',frame_string,'/T2.mat'),'T2');
    save(strcat('/home/akhil/visual_odom/matlab_codes/',frame_string,'/point_cloud2.mat'),'point_cloud');
    save('/home/akhil/visual_odom/matlab_codes/num_frames2.mat','num_frames');
     cameraRtC2W1 = box_synthetic()
    % cameraRtC2W1 = T2;
     T3 = cameraRtC2W1;
 
    for p=1:BA_frames+1
        T3(4,:,p) = [0 0 0 1];
    end
    for p=2:BA_frames+1
        T3(:,:,p) = inv(T3(:,:,1))*T3(:,:,p);
    end
    T3(:,:,1) = eye(4);
    for p = j:k
        T_BA(:,:,p+1) = T_BA_temp*T3(:,:,p-j+1);
    end
    T_BA_temp = T_BA(:,:,k+1);
    
    if j == 0
        indexing = 1;
    else
        indexing = j;
    end
    for p = j:k
        T_viso(:,:,p+1) = T_viso_temp*viso{p-j+1};
        p
    end
    T_viso_temp = T_viso(:,:,k+1);
%     keyboard;
    
end
% viso = demo_viso_stereo(num_frames);
% temp_pose_viso = zeros(4,num_frames);
for i=1:101;
    temp_pose_viso(:,i) = T_viso(:,:,i)*[0;0;0;1];
    temp_pose_BA(:,i) = T_BA(:,:,i)*[0;0;0;1];
end
% for a=1:num_frames
%     visot = viso{a};
%     P(:,:,a) = [visot(1:3,1:3) visot(1:3,4)];
% end
vehicle_positions3 = temp_pose_viso(1:3,:);
vehicle_positions4 = temp_pose_BA(1:3,:);

% z3=vehicle_positions3-[X(segment:segment+len-1)';Y(segment:segment+len-1)';Z(segment:segment+len-1)'];
len = 101;

% yaa = vehicle_positions3(3,len);
% ybb = vehicle_positions3(3,1);
% xaa = vehicle_positions3(1,1);
% xbb = vehicle_positions3(1,len);
% axisx_high = xbb+10;
% axisx_low = xaa-10;
% if xaa > xbb
%     axisx_high = xaa+10;
%     axisx_low  = xbb-10;
% end
% axisy_high = ybb+10;
% axisy_low = yaa-10;
% if yaa > ybb
%     axisy_high = yaa+10;
%     axisy_low  = ybb-10;
% end
 plot(vehicle_positions3(1,1:len),vehicle_positions3(3,1:len),'+b');  xlabel('X') ;ylabel('Z');
 hold on
 plot(vehicle_positions4(1,1:len),vehicle_positions4(3,1:len),'+r');  xlabel('X') ;ylabel('Z');

% load('/home/akhil/visual_odom/matlab_codes/temp.mat');
% a = -temp(1,1,2:end)
% a = a(:);
% b = -temp(3,1,2:end)
% b = b(:);
%  plot(a,b,'+b'); axis([axisx_low axisx_high axisy_low axisy_high]); xlabel('X') ;ylabel('Z');title(sprintf('trajectory comparision max error = %f',m));
 legend('viso','BA');
 T_viso
 T_BA
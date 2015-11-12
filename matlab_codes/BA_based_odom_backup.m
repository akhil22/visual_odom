%function implements local bundle Adjustment based odometry calculation over 'num_keys' frames
% sample sequence for num_keys = 5
% L1 L2 R2 L3 R3 (Li ,Ri represent left image frame of ith iterarion 
% and right image frame of ith iteration respectively 
base_length = 0.54;
seq = 'seq_0';
list_tracks = dir(strcat('/home/akhil/visual_odom/',seq,'/rijvi_tracks/'));
num_keys = 5;
keyfile = cell(num_keys,1);
imagefile = cell(num_keys,1);
Transformations = zeros(4,4,num_keys);
f = zeros(num_keys);
g = cell(num_keys,1);
for i=1:size(list_tracks)-2
    j = i+2;
    track_file = strcat('/home/akhil/visual_odom/',seq,'/rijvi_tracks/',int2str(j),'.txt');
    for key_num=1:num_keys
         if(key_num == 1)
             keyfile{key_num}=strcat('/home/akhil/visual_odom/',seq,'/left_rijvi_keys/',int2str(j),'_',int2str(j-2),'.key');
             imagefile{key_num} = strcat('/home/akhil/visual_odom/',seq,'/left_rijvi_match/',int2str(j),'.jpg');
             Transformations(:,:,1) = [eye(3) zeros(3,1);zeros(1,3),1];
             temp_keyfile = keyfile{key_num};
             f(key_num) = fopen(temp_keyfile);
             g(key_num) = textscan(f(key_num),'%s','delimiter','\n');
             continue;
         end
        if(~mod(key_num,2))
             keyfile{key_num}=strcat('/home/akhil/visual_odom/',seq,'/left_rijvi_keys/',int2str(j+floor(key_num/2)),'_',int2str(j+floor(key_num/2)-2),'.key');
             imagefile{key_num}=strcat('/home/akhil/visual_odom/',seq,'/left_rijvi_match/',int2str(j+floor(key_num/2)),'.jpg');
             temp_keyfile = keyfile{key_num};
             f(key_num) = fopen(temp_keyfile);
             g(key_num) = textscan(f(key_num),'%s','delimiter','\n');
             disp_image2 = strcat('/home/akhil/visual_odom/',seq,'/left_disparity/',int2str(j+floor(key_num/2)),'.png');
             disp_image1 = strcat('/home/akhil/visual_odom/',seq,'/left_disparity/',int2str(j+floor(key_num/2)-1),'.png');
             match_file = strcat('/home/akhil/visual_odom/',seq,'/left_mix_match_features/',int2str(j+floor(key_num/2)-3),'.txt');
             if (key_num == 2)
                Transformations(:,:,key_num) = getTransformation(disp_image1,disp_image2,match_file);
             else
                 temp_trans = getTransformation(disp_image1,disp_image2,match_file);
                 Transformations(:,:,key_num) = Transformations(:,:,key_num-2)*temp_trans;
             end
             continue;
        end
        keyfile{key_num}=strcat('/home/akhil/visual_odom/',seq,'/right_rijvi_keys/',int2str(j+floor(key_num/2)),'_',int2str(j+floor(key_num/2) - 2),'.key');
        imagefile{key_num}=strcat('/home/akhil/visual_odom/',seq,'/left_rijvi_match/',int2str(j+floor(key_num/2)),'.jpg');
        temp_keyfile = keyfile{key_num};
        f(key_num) = fopen(temp_keyfile);
        g(key_num) = textscan(f(key_num),'%s','delimiter','\n');
        temp_trans = [eye(3) zeros(3,1);zeros(1,3),1];
        temp_trans(1,4) = base_length;
        Transformations(:,:,key_num) = Transformations(:,:,key_num-1)*temp_trans;
    end
    fid = fopen(track_file);
    tline = fgets(fid);
    frame = sscanf(tline,'%d');
    k = 1;
    matches = zeros(10000,2);
    ba_input = zeros(6,10000);
    point_index = 1;
    match_index = 1;
    while ischar(tline)
%     disp(tline);
        for l = 1:k-2
            tline = fgets(fid);
            frame = sscanf(tline,'%d');
            tline = fgets(fid);
            num_points=sscanf(tline,'%d');
            for p=1:num_points
                tline = fgets(fid);
                id = sscanf(tline,'%d');
%                 temp_keyfile = keyfile{frame(1)+1};
%                 f1= fopen(temp_keyfile);
%                 temp_keyfile = keyfile{frame(2)+1};
%                 f2= fopen(temp_keyfile);
%                 g1 = textscan(f1,'%s','delimiter','\n');
%                 g2 = textscan(f2,'%s','delimiter','\n');
                points1=sscanf(g{frame(1)+1}{2+8*id(1)},'%f');
                points2=sscanf(g{frame(2)+1}{2+8*id(2)},'%f');
                if frame(1) == 0
                     point3d = get3DPoint(j,frame(1)+1,points1,Transformations);
                elseif ~mod((frame(1)+1),2)
                     point3d = get3DPoint(j,frame(1)+1,points1,Transformations);
                     
                elseif ~mod(frame(2)+1,2)
                     point3d = get3DPoint(j,frame(2)+1,points2,Transformations);
                     
                else
                    continue;
                end
                ba_input(1,point_index) = points1(1);
                ba_input(2,point_index) = points1(2);
                ba_input(3,point_index) = point3d(1);
                ba_input(4,point_index) = point3d(2);
                ba_input(5,point_index) = point3d(3);
                ba_input(6,point_index) = -0.1;
                point_index = point_index + 1;
                ba_input(1,point_index) = points2(1);
                ba_input(2,point_index) = points2(2);
                ba_input(3,point_index) = point3d(1);
                ba_input(4,point_index) = point3d(2);
                ba_input(5,point_index) = point3d(3);
                ba_input(6,point_index) = -0.1;
                point_index = point_index+1;
                matches(match_index,1) = point_index-3;
                matches(match_index,2) = point_index-2;
                match_index = match_index+1;                
            end
        end
        if(k ~= 1)
            tline = fgets(fid);
            frame = sscanf(tline,'%d');
        end
        tline = fgets(fid);
        num_points=sscanf(tline,'%d');
        for p=1:num_points
            tline = fgets(fid);
            id = sscanf(tline,'%d');
%             frame(1)+1;
%             temp_keyfile = keyfile{frame(1)+1};
%             f1= fopen(temp_keyfile);
%             frame(2)+1;
%             temp_keyfile = keyfile{frame(2)+1};
%             f2= fopen(temp_keyfile);
%             g1 = textscan(f1,'%s','delimiter','\n');
%             g2 = textscan(f2,'%s','delimiter','\n');
            points1=sscanf(g{frame(1)+1}{2+8*id(1)},'%f');
            points2=sscanf(g{frame(2)+1}{2+8*id(2)},'%f');
            if frame(1) == 0
                 point3d = get3DPoint(j,frame(1)+1,points1,Transformations);              
            elseif ~mod((frame(1)+1),2)
                 point3d = get3DPoint(j,frame(1)+1,points1,Transformations);                 
            elseif ~mod(frame(2)+1,2)
                 point3d = get3DPoint(j,frame(2)+1,points2,Transformations);                
            else
                 continue;
            end
            ba_input(1,point_index) = points1(1);
            ba_input(2,point_index) = points1(2);
            ba_input(3,point_index) = point3d(1);
            ba_input(4,point_index) = point3d(2);
            ba_input(5,point_index) = point3d(3);
            ba_input(6,point_index) = -0.1;
            point_index = point_index + 1;
            ba_input(1,point_index) = points2(1);
            ba_input(2,point_index) = points2(2);
            ba_input(3,point_index) = point3d(1);
            ba_input(4,point_index) = point3d(2);
            ba_input(5,point_index) = point3d(3);
            ba_input(6,point_index) = -0.1;
            point_index = point_index+1;
            matches(match_index,1) = point_index-3;
            matches(match_index,2) = point_index-2;
            match_index = match_index+1;
        end
      
        
        k=k+1;
        if k >= num_keys
            break
        end
    end
    ba_input=ba_input(:,1:point_index-1);
    matches=matches(1:match_index-1,:);
    T = Transformations(1:3,:,:);
    point_cloud = ba_input(3:5,:);
    save('ba_input.mat','ba_input')
    save('matches.mat','matches')
    save('T.mat','T')
    save('point_cloud.mat','point_cloud');
    break;
end
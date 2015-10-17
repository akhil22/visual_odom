seq = 'seq_0';

%list all the files inside the keypoint directory

list = dir(strcat('/home/akhil/visual_odom/',seq,'/rijvi_list_keys/'));
num_keys = 5;
for i = 1:size(list)-3
    keyfile = cell(num_keys,1);
    tracks = compute_tracks(strcat('/home/akhil/visual_odom/',seq,'/rijvi_tracks/',int2str(i+2),'.txt'))
    feature_tracks=strcat('/home/akhil/visual_odom/',seq,'/rijvi_tracks_features/',int2str(i+2),'.txt')
    f2= fopen(feature_tracks,'w')
    fid = fopen(strcat('/home/akhil/visual_odom/',seq,'/rijvi_list_keys/',list(i+2).name));
    for j=1:num_keys
%         fid = fopen(strcat('/home/akhil/visual_odom/',seq,'/rijvi_list_keys/',list(i+2).name));
        tline = fgets(fid);
        keyfile{j} = tline;
    end
    fclose(fid);
    for j=2:size(tracks,1)
        for k=1:num_keys-1
           
            f= fopen(strcat(keyfile{k,1}))
            g = textscan(f,'%s','delimiter','\n');
            points=sscanf(g{1}{2+8*tracks(j,k)},'%f');
            fprintf(f2,'%f %f ',points(1),points(2));
            fclose(f);
        end
        f= fopen(strcat(keyfile{k,1}));
        g = textscan(f,'%s','delimiter','\n');
        points=sscanf(g{1}{2+8*tracks(j,k)},'%f');
        fprintf(f2,'%f %f\n',points(1),points(2));
        fclose(f);
    end
end
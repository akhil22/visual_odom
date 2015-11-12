function point3d = get3Dpoint2(frame,point,Transformations)
%camera parameters 
cx = 1242/2;
cy = 375/2;
fx = 718.0;
fy = fx;
base_length = 0.54;
seq = 'seq_0';
point3d = zeros(4,1);
K = [fx 0 cx;0 fy cy;0 0 1];
    disp_image =  imread(strcat('/home/akhil/visual_odom/',seq,'/left_disparity/',int2str(frame+2),'.png'));
    disp_image = disp_image';
    pix_col = point(1);
    pix_row = point(2);
    dispar = double(disp_image(floor(pix_col),floor(pix_row))./256.0);
    point3d(3) = fx*base_length./dispar;
    point3d(2) = (pix_row-195)*point3d(3)/fx;
    point3d(1) = (pix_col-672)*point3d(3)/fx;
    point3d(4) = 1;
    point3d = Transformations(:,:,frame)*point3d;
end
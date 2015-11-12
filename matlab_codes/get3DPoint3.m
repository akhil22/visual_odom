function point3d = get3DPoint3(disparity_file,point,Transformations,frame)
%camera parameters 
cx = 1242/2;
cy = 375/2;
fx = 718.856;
fy = fx;
base_length = 0.54;
seq = 'seq_0';
point3d = zeros(4,1);
K = [fx 0 cx;0 fy cy;0 0 1];
K			=	[718.856 0 607.1928; 0 718.856 185.2157; 0 0 1];
    disp_image =  imread(disparity_file);
    disp_image = disp_image';
    pix_col = point(1);
    pix_row = point(2);
    dispar = double(disp_image(round(pix_col),round(pix_row))./256.0);
    point3d(3) = fx*base_length./dispar;
    point3d(2) = (pix_row-185.2157)*point3d(3)/fx;
    point3d(1) = (pix_col-607.1982)*point3d(3)/fx;
    point3d(4) = 1;
    point3d = Transformations(:,:,frame)*point3d;
end

clear all;close all;
% seq no
seq = 'seq_0';

%read ground truth
[X Y Z R P Yaw]=textread(strcat('/home/akhil/Desktop/visual_odom/',seq,'/gtruth/exp.txt'), '%f %f %f %f %f %f', 'headerlines',1);

%initialize segment
segment = 1;

while segment <= 300;

prev_theta = 0;
real_theta=0;

%Construct Translation matrix T of initial Pose from ground truth 
T = eye(4);
T(1,4) = X(segment);
T(2,4) = 0;
T(3,4) = Z(segment);
th = P(segment+1);
dcm = angle2dcm( Yaw(segment), P(segment), R(segment) )
% T(1,1) = cos(th);
% T(1,2) = 0;
% T(1,3) = sin(th);
% T(2,1) = 0;
% T(2,2) = 1;
% T(2,3) = 0;
% T(3,1) = -sin(th);
% T(3,2) = 0;
% T(3,3) = cos(th);
T(1:3,1:3) = dcm';
% T(3,2) = 0;
% T(3,3) = cos(th);
S = T;
% intrinsic camere parameters
base_length = 0.54;
fx=718;
fy=718;

cx = 1242/2;
cy = 375/2;

%internal camera calibration matrix
K = [fx 0 cx;0 fy cy;0 0 1];

% list all the files in left folder
list = dir(strcat('/home/akhil/Desktop/visual_odom/',seq,'/Left/data/'));

%length of trajectory to compute
len = 6;

%for ploting the trajectory 
vehicle_positions = zeros(3,147);

%iterate over images
for q=3:len-1

list(q+segment-1).name
list(q+segment).name

% I1_l =(imread(strcat('/home/akhil/Desktop/visual_odom/',seq,'/Left/data/',list(q+segment-1).name)));
% I2_l =(imread(strcat('/home/akhil/Desktop/visual_odom/',seq,'/Left/data/',list(q+segment).name)));
% I1_r =(imread(strcat('/home/akhil/Desktop/visual_odom/',seq,'/Right/data/',list(q+segment-1).name)));
% I2_r =(imread(strcat('/home/akhil/Desktop/visual_odom/',seq,'/Right/data/',list(q+segment).name)));
I1_l = rgb2gray(imread(strcat('/home/akhil/Desktop/visual_odom/',seq,'/Left/data/',list(q+segment-1).name)));
I2_l = rgb2gray(imread(strcat('/home/akhil/Desktop/visual_odom/',seq,'/Left/data/',list(q+segment).name)));
I1_r = rgb2gray(imread(strcat('/home/akhil/Desktop/visual_odom/',seq,'/Right/data/',list(q+segment-1).name)));
I2_r = rgb2gray(imread(strcat('/home/akhil/Desktop/visual_odom/',seq,'/Right/data/',list(q+segment).name)));


I1 = im2single(I1_l);
I2 = im2single(I2_l); 

% polygon cordinates of the image region we want to consider for correspondence matching  
xi = [  485.7500
  335.7500
  704.7500
  629.7500
  485.7500]';
yi=[    259.2500
  353.7500
  349.2500
  242.7500
  259.2500]';
% xi = [ 305.7500
%     8.7500
%   277.2500
%   413.7500
%   305.7500]';
% yi=[    266.7500
%   361.2500
%   359.7500
%   278.7500
%   266.7500]';


%read deep match feature matches 
feature_file = strcat('/home/akhil/Desktop/visual_odom/',seq,'/left_mix_match_features/',int2str(q+segment-1),'.txt')

% [Y1 X1 Y2 X2]=textread(feature_file, '%f %f %f %f', 'headerlines',1);
 [X1 Y1 X2 Y2]=textread(feature_file, '%f %f %f %f', 'headerlines',1);
% [Y1 X1 Y2 X2]=textread(feature_file, '%f %f %f %f', 'headerlines',1);
in = inpolygon(X1,Y1,xi,yi);
X1=X1(in);
Y1=Y1(in);
X2=X2(in);
Y2=Y2(in);

 figure;
 showMatchedFeatures(I1,I2,[X1 Y1],[X2 Y2]);
 
X1 = [X1';Y1'];
X2 = [X2';Y2'];
X1 = [X1;ones(1,size(X1,2))];
X2 = [X2;ones(1,size(X2,2))];


tX1 = K\X1;
tX2 = K\X2;
nump = size(X1,2);

theta = 2*atan2((1*tX2(2,:).*tX1(1,:)-tX2(1,:).*tX1(2,:)),(tX2(3,:).*tX1(2,:)+tX2(2,:).*tX1(3,:)));
% tX1(1,:) = tX1(1,:)/tX1(3,:);
% tX1(2,:) = tX1(2,:)/tX1(3,:);
% tX2(1,:) = tX2(1,:)/tX2(3,:);
% tX2(2,:) = tX2(2,:)/tX2(3,:);
%  F = estimateFundamentalMatrix(tX1(1:2,:)',tX2(1:2,:)')
 
% keyboard;
%%Ransac to select best angle
n_inliers = 0;
n_model = 0;
for i=1:nump
    temp = 0;
    for j=1:nump
        temp_th1 = theta(i);
        temp_th2 = theta(j);
        if theta(i)< 0 
            temp_th1 = theta(i) + 2*pi;
        end
        if theta(j) < 0 
            temp_th2 = theta(j) + 2*pi;
        end
        if abs(temp_th1-temp_th2)<0.05
            temp = temp+1;
        end
    end
    if temp > n_inliers
        n_inliers = temp;
        n_model = i;
    end
end

nump
n_inliers

calculated_theta(q-2) = theta(n_model);
calculated_theta2(q-2) = theta(n_model);
real_theta(q-2) = P(q-2+segment) - P(q-2+segment-1);
% continue;

% Rcomputed is rotation of 1st frame with respect to second frame
Rcomputed =[cos(theta(n_model)) 0 sin(theta(n_model)) ; 0 1 0 ; -sin(theta(n_model)) 0 cos(theta(n_model))]';

%tcap1 is the direction of translation vector computed in first frame
tcap1 = [sin((theta(n_model))/2) 0 cos((theta(n_model)/2))]'

%tcap is the direction of translation vector computed in second frame 
tcap = Rcomputed*tcap1;


 
%calculate disparity using libelas
param.disp_min    = 0;           % minimum disparity (positive integer)
param.disp_max    = 255;         % maximum disparity (positive integer)
param.subsampling = 0;
[D1_l,D1_r] = elasMex(I1_l',I1_r',param);
[D2_l,D2_r] = elasMex(I2_l',I2_r',param);


%find 3d points of the correspondences and select best 3d point correspondence to get translation magnitude from cosine rule 
ind = 0;
mmin = inf;

%x1main and x2main will contain best 3d correspondence 
x1main = zeros(3,1);
x2main = zeros(3,1);
mss = size(X1,2);
x1 = zeros(3,mss);
x2 = zeros(3,mss);
tarray = zeros(1,mss);
 for t = 1:mss;
% for t = 1:1;
pix_row = X1(2,t);
pix_col = X1(1,t);
x1(3,t) = fx*base_length./D1_l(floor(pix_col),floor(pix_row));
x1(2,t) = (pix_row-195)*x1(3,1)/fx;
x1(1,t) = (pix_col-672)*x1(3,1)/fx;
pix_row = X2(2,t);
pix_col = X2(1,t);
x2(3,t) = fx*base_length/D2_l(floor(pix_col),floor(pix_row));
x2(2,t) = (pix_row-195)*x2(3,1)/fx;
x2(1,t) = (pix_col-672)*x2(3,1)/fx;  
 x2=Rcomputed'*x2;
tarray(t) = sqrt(diag(x1(:,t)'*x1(:,t))+diag(x2(:,t)'*x2(:,t))-2*abs(diag(x1(:,t)'*x2(:,t))));

%best correspondence based on distance from ground plane
% if abs(x1(2,t)+x2(2,t)-3.3)< mmin
%     mmin = abs(x1(2,t)+x2(2,t)-3.3);
%     ind = t;
%     x1main = x1(:,t);
%     x2main = x2(:,t);
% end

if abs(x1(2,t)-1.65)<=0.3 & abs(x2(2,t)-1.65)<=0.3
    mmin = abs(x1(2,t)+x2(2,t)-3.3);
    ind = t;
    x1main = x1(:,t);
    x2main = x2(:,t);
end
end
  x1 = x1main;
  x2 = x2main;

%filter correspondences based on following rule
indd =abs(x1(2,:)-1.65) <= 0.3 & abs(x1(3,:))<= 20 & abs(x2(3,:))<= 20;
indd
x1=x1(:,indd)
x2=x2(:,indd)

%   x1 = x1main;
%   x2 = x2main;
tarray = tarray(tarray<10);
t = median(tarray);
 
Tresult=mean(t)*tcap1

% [x,val] = fmincon(@(x)myobjectiveWithouty(x,x1),[theta(n_model);-Rcomputed*Tresult;x2(:)],[],[],[0,0,1,0,zeros(1,size(x2(:),1))],[0],-0.002,0.002);
 [x,val] = fmincon(@(x)myobjectiveWithouty(x,theta(n_model),x1),[-Rcomputed*Tresult;x2(:)],[],[],[],[],-0.04,0.04,@(x)constraints(x,theta(n_model)));
% theta(n_model)
% theta(n_model) = x(1);
calculated_theta2(q-2) = theta(n_model);

% x(1)
% x1
% x2
 reshape(x(4:end),3,size(x1,2))
 sss = -Rcomputed'*x(1:3)
%  if sss < 1.2

     Rcomputed =[cos(theta(n_model)) 0 sin(theta(n_model)) ; 0 1 0 ; -sin(theta(n_model)) 0 cos(theta(n_model))]';
%            Tresult = -Rcomputed'*x(1:3)
%          Tresult = -Rcomputed*Tresult
% end 
Tcomputed = [Rcomputed' Tresult];
Tcomputed = [Tcomputed; 0 0 0 1];
T = T*Tcomputed;
temp_pose = T*[0;0;0;1];
vehicle_positions(:,q-2) = temp_pose(1:3);

end
close all;
yaa = vehicle_positions(3,len-3);
ybb = vehicle_positions(3,1);
xaa = vehicle_positions(1,1);
xbb = vehicle_positions(1,len-3);
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
figure;
plot(real_theta*180/pi,'+r');
hold on;
plot(calculated_theta*180/pi,'+b');
% hold on
% plot(calculated_theta2*180/pi,'-g');
% legend('ground truth difference in anlge','calculated difference in angle','optimized_difference in angle'); 
legend('ground truth difference in anlge','calculated difference in angle'); 


z=vehicle_positions(:,1:len-3)-[X(segment+1:segment+len-3)';zeros(1,len-3);Z(segment+1:segment+len-3)'];
m=max(sqrt(diag(z'*z)));
figure;
plot(X(segment+1:end),Z(segment+1:end),'+r'); axis([axisx_low axisx_high axisy_low axisy_high]); xlabel('X') ;ylabel('Z');
hold on;
% plot(vehicle_positions(1,1:len-3),vehicle_positions(3,1:len-3),'+b'); axis([axisx_low axisx_high axisy_low axisy_high]); xlabel('X') ;ylabel('Z');title(sprintf('trajectory comparision max error = %f',m));
% legend('ground truth','computed trajecotry');
load('/home/akhil/visual_odom/matlab_codes/temp.mat');
a = -temp(1,1,2:end)
a = a(:);
b = -temp(3,1,2:end)
b = b(:);
 plot(a,b,'+b'); axis([axisx_low axisx_high axisy_low axisy_high]); xlabel('X') ;ylabel('Z');title(sprintf('trajectory comparision max error = %f',m));
 legend('ground truth','computed trajecotry');

xlabel('iteration number')
ylabel('difference in angles between consecutive frames(in degrees)'); 

z=vehicle_positions(:,1:len-3)-[X(segment+1:segment+len-3)';zeros(1,len-3);Z(segment+1:segment+len-3)'];
segment = segment+30;

keyboard;
end

clear all;
 close all;
 clear all;close all;
%function motionEstimation()
%camera focal length
% [X Y Z R P Y]=textread('/home/akhil/exp.txt', '%f %f %f %f %f %f', 'headerlines',1);
seq = 'seq_0';
[X Y Z R P Yaw]=textread(strcat('/home/akhil/Desktop/visual_odom/',seq,'/gtruth/exp.txt'), '%f %f %f %f %f %f', 'headerlines',1);

% figure;
segment = 1
while segment <= 1000;
axisy_high = 50;
axisy_low = -1;
axisx_high = 20;
axisx_low = -20;
calculated_theta=0;
prev_theta = 0;
real_theta=0;
% plot(X(segment+1:end),Z(segment+1:end),'+r'); axis([axisx_low axisx_high axisy_low axisy_high]); xlabel('X') ;ylabel('Z');
% hold on;
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
T
% keyboard;
fx=718;
fy=718;

cx = 1242/2;
cy = 375/2;
%internal camera calibration matrix
K = [fx 0 cx;0 fy cy;0 0 1];

% %radius of curvature of camera path 
% r = 10.5;
% 
% %rotation about y axis
% theta = pi*az/180;
% 
% %Rotation Matrix
% R =[cos(theta) 0. sin(theta) ; 0. 1. 0. ; -sin(theta) 0 cos(theta)]';
% 
% %R =[cos(theta) -sin(theta) 0;sin(theta) cos(theta) 0;0 0 1]';
% % new camera center

% C = [r-r*cos(theta);0;r*sin(theta)];

% plane equation
n1 = 0;%+0.1*randn(1,1);
n2 = -1;%+0.1*randn(1,1);
n3 = 0;%+0.1*randn(1,1);
d = 1.65;%+0.1*randn(1,1);
n = [n1;n2;n3;d]';
base_length = 0.54;
%% generate n random points on the plane
% % generate points in [a b]
% a = 70;
% b = 40;
% nump = 100;
% X = a + (b-a)*rand(2,nump);
% temp = ones(1,nump);
% X = [X(1,:);-d*temp;X(2,:);temp];
% %X = [X;d*temp;temp];
% %%compute projection matrices P1 and P2
% fx_true = fx ;% + 2*randn(1,1)
% fy_true = fy ;%+ 2*randn(1,1)
% Ktrue = [fx_true 0 0;0 fy_true 0;0 0 1];
% P1 = [Ktrue zeros(3,1)];
% P2 = [Ktrue*R -Ktrue*R*C];
list = dir(strcat('/home/akhil/Desktop/visual_odom/',seq,'/Left/data/'));
% list = dir('/home/akhil/image_bkp/');
%len = size(list,1);
len = 14;
vehicle_positions = zeros(3,147);

for q=3:len-1
q
list(q+segment-1).name
list(q+segment).name
% I1_l = rgb2gray(imread(strcat('/home/akhil/Desktop/Left_png/1/',list(q+segment-1).name)));
% I2_l = rgb2gray(imread(strcat('/home/akhil/Desktop/Left_png/1/',list(q+segment).name)));
% I1_r = rgb2gray(imread(strcat('/home/akhil/Desktop/Right/1/',list(q+segment-1).name)));
% I2_r = rgb2gray(imread(strcat('/home/akhil/Desktop/Right/1/',list(q+segment).name)));
% I1_l =(imread(strcat('/home/akhil/Desktop/visual_odom/',seq,'/Left/data/',list(q+segment-1).name)));
% I2_l =(imread(strcat('/home/akhil/Desktop/visual_odom/',seq,'/Left/data/',list(q+segment).name)));
% I1_r =(imread(strcat('/home/akhil/Desktop/visual_odom/',seq,'/Right/data/',list(q+segment-1).name)));
% I2_r =(imread(strcat('/home/akhil/Desktop/visual_odom/',seq,'/Right/data/',list(q+segment).name)));
I1_l = rgb2gray(imread(strcat('/home/akhil/Desktop/visual_odom/',seq,'/Left/data/',list(q+segment-1).name)));
I2_l = rgb2gray(imread(strcat('/home/akhil/Desktop/visual_odom/',seq,'/Left/data/',list(q+segment).name)));
I1_r = rgb2gray(imread(strcat('/home/akhil/Desktop/visual_odom/',seq,'/Right/data/',list(q+segment-1).name)));
I2_r = rgb2gray(imread(strcat('/home/akhil/Desktop/visual_odom/',seq,'/Right/data/',list(q+segment).name)));
% I1_l = rgb2gray(imread(strcat('/home/akhil/image_bkp/',list(q+segment-1).name)));
% I2_l = rgb2gray(imread(strcat('/home/akhil/image_bkp/',list(q+segment).name)));
% I1_r = rgb2gray(imread(strcat('/home/akhil/image_right/',list(q+segment-1).name)));
% I2_r = rgb2gray(imread(strcat('/home/akhil/image_right/',list(q+segment).name)));


I1 = im2single(I1_l);
I2 = im2single(I2_l); 

%binSize =8;
%[f1,d1] = vl_dsift(I1,'size',binSize);
%[f2,d2] = vl_dsift(I2,'size',binSize);
% [f1,d1] = vl_sift(I1);
% [f2,d2] = vl_sift(I2);
% [f1, d1] = getFeatures(I1_l, 'peakThreshold', 0.01) ;
% [f2, d2] = getFeatures(I2_l, 'peakThreshold', 0.01) ;
% 
% [matches, scores] = vl_ubcmatch(d1,d2) ;
% xi = [ 311.7500
%    89.7500
%   317.7500
%   425.7500
%   311.7500]';
% yi=[271.2500
%   347.7500
%   361.2500
%   293.7500
%   271.2500]';
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
% points1 = detectSURFFeatures(I1);
% points2 = detectSURFFeatures(I2);
% points1 = corner(I1,'Harris');
% points2 = corner(I2,'Harris');

%indexPairs = matchFeatures(d1, d2);
% X1 = vpts1(indexPairs(:, 1),1:2);
% X2 = vpts2(indexPairs(:, 2),1:2);
% x = f1(1:2,matches(1,:))';
% y = f2(1:2,matches(2,:))';
% %figure; ax= axes;
% 
% % legend('Matched points 1','Matched points 2');
% % X1 = X1.Location;
% % X2 = X2.Location;
% % in = inpolygon(x(:,1),x(:,2),xi,yi);
%  X1=x(:,1);
%  Y1=x(:,2);
%  X2=y(:,1);
%  Y2=y(:,2);
%  figure;
%  showMatchedFeatures(I1,I2,[X1 Y1],[X2 Y2]);
 feature_file = strcat('/home/akhil/Desktop/visual_odom/',seq,'/left_deep_match_features/',int2str(q-2+segment-1),'.txt')
%   feature_file = strcat('/home/akhil/Desktop/rotation_deep_match_features/',int2str(q-2+segment-1),'.txt')
[X1 Y1 X2 Y2 score index]=textread(feature_file, '%d %d %d %d %f %d', 'headerlines',1);
in = inpolygon(X1,Y1,xi,yi);
X1=X1(in);
Y1=Y1(in);
X2=X2(in);
Y2=Y2(in);
%scores = scores(in);
% index = index(in);
% in = score > 0;
% X1=X1(in);
% Y1=Y1(in);
% X2=X2(in);
% Y2=Y2(in);
% score = score(in);
% index = index(in);
%plot(score);
% figure;
% showMatchedFeatures(I1,I2,[X1 Y1],[X2 Y2]);
X1 = [X1';Y1'];
X2 = [X2';Y2'];
% X1 = reshape(X1,[2,size(X1,1)]);
% X2 = reshape(X2,[2,size(X2,1)]);
X1 = [X1;ones(1,size(X1,2))];
X2 = [X2;ones(1,size(X2,2))];
 %showMatchedFeatures(I1,I2,x,y);
%X1 = X1';
%X2 = X2';
% X1 = reshape(X1,[2,size(X1,1)]);
% X2 = reshape(X2,[2,size(X2,1)]);
% X1 = [X1';Y1';ones(1,size(X1,1))];
% X2 = [X2';Y2';ones(1,size(X2,1))];
% X1 = [Y1';X1';ones(1,size(X1,1))];
% X2 = [Y2';X2';ones(1,size(X2,1))];



%%computing projection of these points on camera planes 
% X1 = P1*X;
% X2 = P2*X;
% 
% %%
% for i=1:nump
%     X1(:,i) = X1(:,i)/X1(3,i);
%     X2(:,i) = X2(:,i)/X2(3,i);
% end
% 
% X2 = X2 + [zeros(2,nump-20),1*randn(2,20);zeros(1,nump)];
% X1 = X1 + [zeros(2,nump-20),1*randn(2,20);zeros(1,nump)];
%[H corrPtIdx]=findHomography(X1(1:2,:),X2(1:2,:));  

%[H corrPtIdx] = homography(I1,I2);
tX1 = K\X1;
tX2 = K\X2;
nump = size(X1,2);
%computing theta based on essential matrix formulation 
%theta = -2*atan2(X2(2,p)*X1(3,p)-X2(3,p)*X1(2,p),X2(1,p)*X1(3,p)+X2(3,p)*X1(1,p))
theta = 2*atan2((1*tX2(2,:).*tX1(1,:)-tX2(1,:).*tX1(2,:)),(tX2(3,:).*tX1(2,:)+tX2(2,:).*tX1(3,:)));

% for i=1:nump
%     if theta(i) < 0
%         theta(i) = theta(i) + 2*pi;
%     end
% end

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
% s = [0:.05:6];
nump
n_inliers
n_nrm = sqrt( n1^2 + n2^2 + n3^2 ) ;
n1 = n1 / n_nrm ;
n2 = n2 / n_nrm ;
n3 = n3 / n_nrm ;
d= d/n_nrm;
calculated_theta(q-2) = theta(n_model);
%  if(theta(n_model) < 0)
%  calculated_theta(q-2) = theta(n_model)+2*pi;
%  end
%theta(n_model) = theta(n_model)-2*pi;
real_theta(q-2) = P(q-2+segment) - P(q-2+segment-1);
% theta(n_model) = P(q-2+segment) - P(q-2+segment-1);
%  calculated_theta(q-2) = real_theta(q-2);
% feature_file = strcat('/home/akhil/Desktop/left_deep_match_features/',int2str(q-2+segment-1),'.txt')
% [X1 Y1 X2 Y2 score index]=textread(feature_file, '%d %d %d %d %f %d', 'headerlines',1);
% xi = [499.5446;649.1831;761.4120;257.1301;474.1060;499.5446];
% yi = [235.1361;233.6398;371.3072;372.8036;233.6398;235.1361];
% xi =[376.2500;284.7500;746.7500;686.7500;376.2500];
% yi=  [311.7500;371.7500;371.7500;299.7500;311.7500];
%theta(n_model) = (theta(n_model);
% xi = [ 311.7500
%    89.7500
%   317.7500
%   425.7500
%   311.7500]';
% yi=[271.2500
%   347.7500
%   361.2500
%   293.7500
%   271.2500]';
% a = X1;
% b = X2;
% X1 = a(1,:)';
% Y1 = a(2,:)';
% X2 = b(1,:)';
% Y2 = b(2,:)';
% % figure;
% %  showMatchedFeatures(I1,I2,X(1:2,:)',Y(1:2,:)');
% in = inpolygon(X1,Y1,xi,yi);
% X1=X1(in);
% Y1=Y1(in);
% X2=X2(in);
% Y2=Y2(in);
% scores = scores(in);
% % index = index(in);
% % in = score > 0;
% % X1=X1(in);
% % Y1=Y1(in);
% % X2=X2(in);
% % Y2=Y2(in);
% % score = score(in);
% % index = index(in);
% %plot(score);
% figure;
% showMatchedFeatures(I1,I2,[X1 Y1],[X2 Y2]);
% X1 = [X1';Y1'];
% X2 = [X2';Y2'];
% % X1 = reshape(X1,[2,size(X1,1)]);
% % X2 = reshape(X2,[2,size(X2,1)]);
% X1 = [X1;ones(1,size(X1,2))];
% X2 = [X2;ones(1,size(X2,2))];
%[H corrPtIdx] =ransacfithomography(X1(1:2,:),X2(1:2,:),0.0001);
% [H corrPtIdx] =ransacfithomography([X1(1,:);X1(2,:)],[X2(2,:);X2(1,:)],0.0001);
% H = ones(3,3);
%[H corrPtIdx]=findHomography(X1(1:2,:),X2(1:2,:));
%% using svd to get translation
%computing the left hand side 
%%applyign normalization 

% X1c = sum(X1(1,:))/nump;
% Y1c = sum(X1(2,:))/nump;
% a = sqrt(2);
% X1 = [a/fx 0 -a*X1c/fx; 0 a/fy -a*Y1c/fy; 0 0 1]*X1;
% X2c = sum(X1(1,:))/nump;
% Y2c = sum(X1(2,:))/nump;
% X2 = [a/fx 0 -a*X2c/fx; 0 a/fy -a*Y2c/fy; 0 0 1]*X2;
% figure;
% plot(X1(1,:),X1(2,:),'+r');
% figure;
% plot(X2(1,:),X2(2,:),'+b');
% A = zeros(200,3);
% for i=1:nump
%     A(2*i-1,1) = 0;
%     A(2*i-1,2) = (-X2(3,i)*X1(1,i)*n1*(fy/fx)-X2(3,i)*X1(2,i)*n2-X2(3,i)*X1(3,i)*fy*n3)/d;
%     A(2*i-1,3) = (X2(2,i)*X1(1,i)*n1*(1/fx)+X2(2,i)*X1(2,i)*n2*(1/fy)+X2(2,i)*X1(3,i)*n3)/d;
%     A(2*i,1) = (X2(3,i)*X1(1,i)*n1+X2(3,i)*X1(2,i)*(fx/fy)*n2+X2(3,i)*X1(3,i)*fx*n3)/d;
%     A(2*i,2)=0;
%     A(2*i,3) = (-X2(1,i)*X1(1,i)*n1*(1/fx)-X2(1,i)*X1(2,i)*n2*(1/fy)-X2(1,i)*X1(3,i)*n3)/d;
% end

%%changing 
% theta(n_model) = real_theta(q-2);

%constraint that the final translation vector's slope is (theta/2)
Rcomputed =[cos(theta(n_model)) 0 sin(theta(n_model)) ; 0 1 0 ; -sin(theta(n_model)) 0 cos(theta(n_model))]';
A = K*Rcomputed*(K\eye(3)); 
% qw = (theta(n_model))/2;
% constr = (tan(qw)*cos(2*qw) - sin(2*qw))/(tan(qw)*sin(2*qw)+cos(2*qw));
% norm2 = sqrt(constr^2+1);
% constr = constr/norm2;
%tcap = [constr;0;1/norm2]
tcap1 = [sin((theta(n_model))/2) 0 cos((theta(n_model)/2))]'
tcap = Rcomputed*tcap1;
% B = (K*tcap*[n1,n2,n3]*(K\eye(3)));
% A
% B
% A = reshape(A,9,1)
% B = reshape(B,9,1)
% A = [A,B/d]
% 
% H= [-0.6399011 0.258648 -47.0547 ;
% -0.005955566 -0.5582205 -11.8107; 
% -1.1394e-05 0.0004318352 -0.7110852]
% 
% v = A\reshape(H,9,1)/100
% %v = lsqlin(A,y,[],[],[],[],[],[],[]);
% A*v
% reshape(H,9,1)/100
% A = zeros(200,1);
% for i=1:nump
%      A(2*i-1,1) = (1/constr)*(X2(2,i)*X1(1,i)*n1*(1/fx)+X2(2,i)*X1(2,i)*n2*(1/fy)+X2(2,i)*X1(3,i)*n3)/d;
%      A(2*i,1) = (X2(3,i)*X1(1,i)*n1+X2(3,i)*X1(2,i)*(fx/fy)*n2+X2(3,i)*X1(3,i)*fx*n3)/d+(1/constr)*(-X2(1,i)*X1(1,i)*n1*(1/fx)-X2(1,i)*X1(2,i)*n2*(1/fy)-X2(1,i)*X1(3,i)*n3)/d;
% end
 
% A = zeros(200,2);
% for i=1:nump
%     A(2*i-1,1) = 0;
%     A(2*i-1,2) = (X2(2,i)*X1(1,i)*n1*(1/fx)+X2(2,i)*X1(2,i)*n2*(1/fy)+X2(2,i)*X1(3,i)*n3)/d;
%     A(2*i,1) = (X2(3,i)*X1(1,i)*n1+X2(3,i)*X1(2,i)*(fx/fy)*n2+X2(3,i)*X1(3,i)*fx*n3)/d;
%     A(2*i,2) = (-X2(1,i)*X1(1,i)*n1*(1/fx)-X2(3,i)*X1(2,i)*n2*(1/fy)-X2(1,i)*X1(3,i)*n3)/d;
% end
%computing the right hand sideuntitled.jpg
% 
% B =     
% B2 = reshape(B,200,1);
% num_points = 200;
% %T = lsqlin(A(1:num_points,:),B2(1:num_points),A(1:num_points,:),B2(1:num_points))
% %T = lsqlin(A(1:num_points,:),B2(1:num_points),zeros(num_points,3),zeros(1,num_points),[1 0 -tan(theta(n_model)/2)],0);
% [U,S,V]=svd(A,0);
% %T = lsqlin(A(1:num_points,:),B2(1:num_points),A(1:num_points,:),B2,[1 0 tan(theta(n_model)/2)],0)
% 
% T= V*((U'*B2)./diag(S));
% %T = [T(1);0;T(2)];
% T = [T(1),0,T(1)/constr]';
% [R Rcomputed];
%  Tresult = -Rcomputed'*(v(2)/v(1))*tcap
%  Tcomputed = [Rcomputed' Tresult];
%  Tcomputed = [Tcomputed; 0 0 0 1];
%  T = T*Tcomputed;
%  temp_pose = T*[0;0;0;1];
%  vehicle_positions(:,q-2) = temp_pose(1:3);
% 
point_taken = 2;
% x1 = [377,317,1];
% x2 = [

 x1 = [X1(1,:);X1(2,:);ones(1,size(X1,2))];
 x2 = [X2(1,:);X2(2,:);ones(1,size(X1,2))];
 I1 = im2single(I1_l);
I2 = im2single(I2_l); 

%binSize =8;
%[f1,d1] = vl_dsift(I1,'size',binSize);
%[f2,d2] = vl_dsift(I2,'size',binSize);
[f1,d1] = vl_sift(I1);
[f2,d2] = vl_sift(I2);

[matches, scores] = vl_ubcmatch(d1,d2) ;
% xi = [ 311.7500
%    89.7500
%   317.7500
%   425.7500
%   311.7500]';
% yi=[271.2500
%   347.7500
%   361.2500
%   293.7500
%   271.2500]';

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
% points1 = detectSURFFeatures(I1);
% points2 = detectSURFFeatures(I2);
% points1 = corner(I1,'Harris');
% points2 = corner(I2,'Harris');

%indexPairs = matchFeatures(d1, d2);
% X1 = vpts1(indexPairs(:, 1),1:2);
% X2 = vpts2(indexPairs(:, 2),1:2);
x = f1(1:2,matches(1,:))';
y = f2(1:2,matches(2,:))';
%figure; ax= axes;

% legend('Matched points 1','Matched points 2');
% X1 = X1.Location;
% X2 = X2.Location;
% % % % % % xi=[575.7500
% % % % % %   649.2500
% % % % % %   662.7500
% % % % % %   554.7500
% % % % % %   575.7500];
% % % % % % yi=[263.7500
% % % % % %   260.7500
% % % % % %   313.2500
% % % % % %   314.7500
% % % % % %   263.7500];
in = inpolygon(x(:,1),x(:,2),xi,yi);
 X1=x(in,1);
 Y1=x(in,2);
 X2=y(in,1);
 Y2=y(in,2);
  %feature_file = strcat('/home/akhil/Desktop/rotation_deep_match_features/',int2str(q-2+segment-1),'.txt')
 feature_file = strcat('/home/akhil/Desktop/visual_odom/',seq,'/left_deep_match_features/',int2str(q-2+segment-1),'.txt')
[X1 Y1 X2 Y2 score index]=textread(feature_file, '%d %d %d %d %f %d', 'headerlines',1);
in = inpolygon(X1,Y1,xi,yi);
X1 = X1(in);
Y1 = Y1(in);
X2 = X2(in);
Y2 = Y2(in);
% figure;
% showMatchedFeatures(I1,I2,[X1 Y1],[X2 Y2]);
 X1 = [X1';Y1'];
X2 = [X2';Y2'];
% X1 = reshape(X1,[2,size(X1,1)]);
% X2 = reshape(X2,[2,size(X2,1)]);
X1 = [X1;ones(1,size(X1,2))];
X2 = [X2;ones(1,size(X2,2))];

%  figure;
%  x1 = [499;274;1];
%  x2 = [496;284;1];
 x1 = inv(K)*x1;
 x2 = inv(K)*x2;
 k1 = 1.65./x1(2,:);
 k2 = 1.65./x2(2,:);
 x1(1,:)=k1.*x1(1,:);
 x1(2,:)=k1.*x1(2,:);
 x1(3,:)=k1.*x1(3,:);
 x2(1,:)=k2.*x2(1,:);
 x2(2,:)=k2.*x2(2,:);
 x2(3,:)=k2.*x2(3,:);

param.disp_min    = 0;           % minimum disparity (positive integer)
param.disp_max    = 255;         % maximum disparity (positive integer)
param.subsampling = 0;
[D1_l,D1_r] = elasMex(I1_l',I1_r',param);
[D2_l,D2_r] = elasMex(I2_l',I2_r',param);



ind = 0;
mmin = inf;
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

if abs(x1(2,t)+x2(2,t)-3.3)< mmin
    mmin = abs(x1(2,t)+x2(2,t)-3.3)
    ind = t
    x1main = x1(:,t);
    x2main = x2(:,t);
end
end
% x1 = x1main;
% x2 = x2main;
% pix_row = X2(2,ind);
% pix_col = X2(1,ind);
% x2(3,1) = fx*base_length/D2_l(floor(pix_col),floor(pix_row));
% x2(2,1) = (pix_row-195)*x2(3,1)/fx;
% x2(1,1) = (pix_col-672)*x2(3,1)/fx;  
x1
x2
indd =abs(x1(2,:)-1.65) <= 0.2 & abs(x1(3,:))<= 15 & abs(x2(3,:))<= 15;
indd
x1=x1(:,indd)
x2=x2(:,indd)
%   x1(1,:)= 10.0535; 
%   x1(2,:)=1.6595;
%   x1(3,:)=13.8549;
%   x2(1,:)=10.6021 ;
%   x2(2,:)=1.65466;
%   x2(3,:)= 14.6673;

  x1 = x1main;
  x2 = x2main;
 tarray = tarray(tarray<10);
 t = median(tarray);
 %t = sqrt(diag(x1'*x1)+diag(x2'*x2)-2*abs(diag(x1'*x2)));
 %t = Z(segment+q-2) - Z(segment+q-3)
% Tresult=-Rcomputed'*t*tcap1
%t=v(2)/v(1);
  Tresult=mean(t)*tcap1
  if Tresult(3) > 2
%         break
  end
  
  [x,val] = fmincon(@(x)myobjective(x,x1),[theta(n_model);-Rcomputed*Tresult;x2(:)],[],[],[],[],-0.03,0.03);
   theta(n_model) = x(1);
   x(1)
    Rcomputed =[cos(theta(n_model)) 0 sin(theta(n_model)) ; 0 1 0 ; -sin(theta(n_model)) 0 cos(theta(n_model))]';
  Tresult = -Rcomputed'*x(2:4)
Tcomputed = [Rcomputed' Tresult];
 Tcomputed = [Tcomputed; 0 0 0 1];
 T = T*Tcomputed;
 temp_pose = T*[0;0;0;1];
 vehicle_positions(:,q-2) = temp_pose(1:3);
% Tresult(2) = 0;
%  [Tresult, C]
% acos([Tresult'/norm(Tresult) * C/norm(C)])*180/pi
% Tresult'/norm(Tresult) * C/norm(C);
% tempo2(q) =180*abs((pi*az/180) - theta(n_model))/pi
% %tempo2 = tempo2+ acos(Tresult'/norm(Tresult) * C/norm(C))*180/pi;
% tempo(q) = real(sqrt((Tresult(1)-C(1))^2+(Tresult(3)-C(3))^2))
% %fprintf(1,'At iteration %d: tem is: %f\n',q,tempo);
% %T = A\B2 ;
% %-Rcomputed'*T
% %T = lsqlin(A(1:num_points,:),B2(1:num_points));
% %Rcomputed'*T

% figure;
% plot(andg,'+b');
% title(strcat('R=',num2str(r),'m'));
% xlabel('yaw angle in degrees');
% ylabel('euclidean error in meters');
% figure;
% plot(and2g,'+r');
% title(strcat('R=',num2str(r),'m'));
% xlabel('yaw angle in degrees');
% ylabel('yaw angle error in degrees');
%end
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
 % % axisx_high = 20;
% axisx_low = -20 ;
% axisx_high = vehicle_positions(1,len-3)+1;
%  axisx_low = vehicle_positions(1,1)-1;


z=vehicle_positions(:,1:len-3)-[X(segment+1:segment+len-3)';zeros(1,len-3);Z(segment+1:segment+len-3)'];
m=max(sqrt(diag(z'*z)));
figure;
plot(X(segment+1:end),Z(segment+1:end),'+r'); axis([axisx_low axisx_high axisy_low axisy_high]); xlabel('X') ;ylabel('Z');
hold on;
plot(vehicle_positions(1,1:len-3),vehicle_positions(3,1:len-3),'+b'); axis([axisx_low axisx_high axisy_low axisy_high]); xlabel('X') ;ylabel('Z');title(sprintf('trajectory comparision max error = %f',m));
legend('ground truth','computed trajecotry');
% figure;
% plot(X(segment+1:end),Z(segment+1:end),'+r');  xlabel('X') ;ylabel('Z');
% hold on;
% plot(vehicle_positions(1,1:len-3),vehicle_positions(3,1:len-3),'+b'); xlabel('X') ;ylabel('Z');title(sprintf('trajectory comparision max error = %f',m));
% legend('ground truth','computed trajecotry');

figure;
plot(real_theta*180/pi,'+r');
hold on;
plot(calculated_theta*180/pi,'+b');
legend('ground truth difference in anlge','calculated difference in angle'); 
xlabel('iteration number')
ylabel('difference in angles between consecutive frames(in degrees)'); 
%vehicle_positions(:,1)
z=vehicle_positions(:,1:len-3)-[X(segment+1:segment+len-3)';zeros(1,len-3);Z(segment+1:segment+len-3)'];
segment = segment+30;
% figure;
keyboard;
end
% axis([0 len-3 -180 180]);

%size(vehicle_positions)
function T = getTransformation(disp_image1,disp_image2,feature_file)

%function gives the Transformation between two camera frames disp_image1 and disp_image2.
%Correspoding point in 
%the two frames is given by the feature_file. function uses
%scaramuzza's method to compute the planer rotaion between frames and
%cosine rule to get the translation

% internel Camera parameters
cx = 607.1928;
cy = 185.2157;
fx = 718.856;
fy = fx;
base_length = 0.54;
K = [fx 0 cx;0 fy cy;0 0 1];
K			=	[718.856 0 607.1928; 0 718.856 185.2157; 0 0 1];

%read the images
I1 = imread(disp_image1);
I2 = imread(disp_image2);

I1 = I1';
I2 = I2';
%read the correspondences 
[X1 Y1 X2 Y2]=textread(feature_file, '%f %f %f %f', 'headerlines',1);

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

in = inpolygon(X1,Y1,xi,yi);
X1=X1(in);
Y1=Y1(in);
X2=X2(in);
Y2=Y2(in);

%using scaramuzza's method to get the angle 
X1 = [X1';Y1'];
X2 = [X2';Y2'];

X1 = [X1;ones(1,size(X1,2))];
X2 = [X2;ones(1,size(X2,2))];

tX1 = K\X1;
tX2 = K\X2;
nump = size(X1,2);

theta = 2*atan2((1*tX2(2,:).*tX1(1,:)-tX2(1,:).*tX1(2,:)),(tX2(3,:).*tX1(2,:)+tX2(2,:).*tX1(3,:)));

for i = 1:nump;
    if(theta(i) >= pi)
        theta(i) = theta(i)-2*pi;
    end
    if(theta(i) <= -pi)
        theta(i) = theta(i) +2*pi;
    end
end
%%Ransac to select best angle
n_inliers = 0;
n_model = 0;
for i=1:nump
    temp = 0;
    for j=1:nump
        temp_th1 = theta(i);
        temp_th2 = theta(j);
        if theta(i) < 0 
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

nump;
n_inliers;
Rcomputed =[cos(theta(n_model)) 0 sin(theta(n_model)) ; 0 1 0 ; -sin(theta(n_model)) 0 cos(theta(n_model))]';
tcap1 = [sin((theta(n_model))/2) 0 cos((theta(n_model)/2))]';

%tcap is the direction of translation vector computed in second frame 
tcap = Rcomputed*tcap1;
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
dispar = double(I1(round(pix_col),round(pix_row))./256.0);
x1(3,t) = fx*base_length./dispar;
x1(2,t) = (pix_row-cy)*x1(3,1)/fx;
x1(1,t) = (pix_col-cx)*x1(3,1)/fx;
pix_row = X2(2,t);
pix_col = X2(1,t);
dispar = double(I2(round(pix_col),round(pix_row))./256.0);
x2(3,t) =  fx*base_length/dispar;
x2(2,t) = (pix_row-cy)*x2(3,1)/fx;
x2(1,t) = (pix_col-cx)*x2(3,1)/fx;  
x2=Rcomputed'*x2;
tarray(t) = sqrt(diag(x1(:,t)'*x1(:,t))+diag(x2(:,t)'*x2(:,t))-2*abs(diag(x1(:,t)'*x2(:,t))));

if abs(x1(2,t)-1.65)<=0.2 & abs(x2(2,t)-1.65)<=0.2
    mmin = abs(x1(2,t)+x2(2,t)-3.3);
    ind = t;
    x1main = x1(:,t);
    x2main = x2(:,t);
end
end
%    x1 = x1main;
%    x2 = x2main;
x1main;
x2main;
%filter correspondences based on following rule
indd =abs(x1(2,:)-1.65) <= 0.2 & abs(x1(3,:))<= 15 & abs(x2(3,:))<= 15;
indd;
x1=x1(:,indd);
x2=x2(:,indd);

   x1 = x1main;
   x2 = x2main;
tarray = tarray(tarray<10);
t = median(tarray);
 if t > 1.52
     t=1.5;
 end
Tresult=mean(t)*tcap1;

Tcomputed = [Rcomputed' Tresult];
T = [Tcomputed; 0 0 0 1];
end
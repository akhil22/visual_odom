%% main file 

clear all
close all
clc


%% Generate Synthetic 3d  
x=1:100;
y=ones(1,100);
ThreeD_points=zeros(1,1);
l=1;
for i=1:10
    
    for j=1:10
    ThreeD_points(l,1)=x(i);
    ThreeD_points(l,2)=y(j);
    ThreeD_points(l,3)=x(j);
    ThreeD_points(l,4)=l;
    l=l+1;
    end
end
[X,Y]=meshgrid(x,y);
Z=X-Y;

%% Intrensic matrix
fx=1;
fy=1;
cx=0;
cy=0;
K=[fx 0 cx;0 fy cy;0 0 1];


P=K*[eye(3) zeros(1,3)'];   % Initial 

for i=1:length(ThreeD_points)   
    wp=[ThreeD_points(i,1:3) 1];
    image_point =P*wp';     
    TwoD_points(i,1)=(image_point(1)/image_point(3));
    TwoD_points(i,2)=(image_point(2)/image_point(3));  
end
%%   Optimization 

r_gt=eye(3);
t_gt=[0 0 0];

%load Groundtruth Trajectory 

load Traj_09.mat



for sj=1:100    % length of trajectory

sj

t_gt1=Traj(sj,1:3);

T_GroundTruth2(sj,1:3)=t_gt1;

%if(sj)

t_gt=t_gt1;

%else 
    t_gt=Traj(sj,1:3)-Traj(sj-1,1:3);
   
%end 

%% Get New 3D Points and 2D points %%%%
if(sj)
for i=1:length(ThreeD_points)
new_ThreeD_points(i,1:3)=ThreeD_points(i,1:3)';%mtimes(r_gt,ThreeD_points(i,1:3)');
new_ThreeD_points(i,1:3)=t_gt+new_ThreeD_points(i,1:3);
end
end
%new_ThreeD_points(i,1:3)=t_gt+new_ThreeD_points(i,1:3);
%else
    
%end 
R1=normrnd(0,3,[100,3])
R1=(R1*.01);

%new_ThreeD_points=new_ThreeD_points+R1;




% Project 3D points into 2D again 

for i=1:length(new_ThreeD_points)
    wp=[new_ThreeD_points(i,1:3) 1];    
    image_point =P*wp';   
    new_TwoD_points(i,1)=(image_point(1)/image_point(3));
    new_TwoD_points(i,2)=(image_point(2)/image_point(3));        
end

%% Add Gaussian Noise with Zero mean and One Variance 
R=normrnd(0,3,[100,2])
R=(R*.001);

new_TwoD_points=new_TwoD_points+(R);


T_init=[t_gt1(1)-0.5 0 t_gt1(3)+0.5];    % Intialization Around Ground truth 

T_in1=T_init;

Translation_init(sj,1:3)=T_init;

T_opt_tmp=lm_optimization(K,T_init,ThreeD_points,new_TwoD_points);   % LM Optimization

Translation_LM_Traj(sj,1:3)=T_opt_tmp;



clear new_TwoD_points;
clear new_ThreeD_points;

end 


%% Translation Eucledian Error 
T_Error=0;

for i=1:length(Translation_LM_Traj)

T_Error=T_Error+sqrt(power((Translation_LM_Traj(i,1)-T_GroundTruth2(i,1)),2)+power((Translation_LM_Traj(i,3)-T_GroundTruth2(i,3)),2));


end 

%% Plot 

T_Error1=T_Error/length(Translation_LM_Traj);

figure(1),plot(T_GroundTruth2(:,1),T_GroundTruth2(:,3),'b--o',Translation_LM_Traj(:,1),Translation_LM_Traj(:,3),'g--*')




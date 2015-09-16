function [J] = jacobian_reproject(K,T_in,X_threeD)
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here


numPoints=length(X_threeD);

%J=zeros(numPoints,2);
numCameras=1;

J=zeros(2*numCameras*numPoints,numCameras*2); 

cur_idx=0;

colIdx = 1;
for i=1:numPoints 

    
    
    
    
    ele1= -K(1,1)/(X_threeD(i,3)+T_in(3));
    
    ele2= 0;
    
    
    
    
    
    
    ele3= (-((X_threeD(i,3)+T_in(3))*K(1,3))+(K(1,1)*X_threeD(i,1)+K(1,3)*X_threeD(i,3)+K(1,1)*T_in(1)+K(1,3)*T_in(3))) / ((X_threeD(i,3)+T_in(3))^2);
    
    ele4= (-((X_threeD(i,3)+T_in(3))*K(2,3))+(K(2,2)*X_threeD(i,2)+K(2,3)*X_threeD(i,3)+K(2,3)*T_in(3)))/ ((X_threeD(i,3)+T_in(3))^2);
    
    vector2=[ele2 ele4];
    
    vector1=[ele1 ele3];
    
    J(cur_idx+2, colIdx:(colIdx+1))=  vector2;
    
    J(cur_idx+1, colIdx:(colIdx+1))= vector1;
    
    cur_idx=cur_idx+2;


end 





end


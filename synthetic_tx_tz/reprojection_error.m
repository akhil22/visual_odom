function [errVec,tot_error] = reprojection_error(K,T_opt,X_threeD,X_twoD )
%UNTITLED2 Summary of this function goes here
%   Detailed explanation goes here
num_cameras=1;
numPoints=size(X_threeD);

errVec=zeros(2*num_cameras*numPoints,1);






   cur_idx=1;
   P1=K*[eye(3) T_opt'];
    for j=1:numPoints
        projectedPoint=P1*[X_threeD(j,1:3) 1]';

        errVec(cur_idx:cur_idx+1,1)=X_twoD(j,:)-nonHomogenize(projectedPoint)';
%         temp_vect=X_twoD(j,:)-nonHomogenize(projectedPoint)';
%         errVec(cur_idx,1)=temp_vect(1)^2;
%         errVec(cur_idx+1,1)=temp_vect(2)^2;
        
        cur_idx=cur_idx+2;
    end


tot_error=norm(errVec);








end


function [errVec,tot_error] = reprojectionErrorTrans(K,Rmat,T_opt,X0,Images)
%UNTITLED3 Summary of this function goes here
%   Detailed explanation goes here

%%    
% numPoints=size(Points,2);
% numCameras=size(Rmat,3);
% 
% reprojection_error=zeros(numCameras,numPoints);
% 
% for i=1:numCameras
%     for j=1:numPoints
%        im_estimate=K*[Rmat(:,:,i)'*Points(:,j)-Rmat(:,:,i)'*T(:,i)];
%        diff=(Images{1,i}(1,j)-nonHomogenize(im_estimate));
%        reprojection_error(i,j)=sqrt(sum(diff.*diff));
%     end
% end
% 
% tot_error=sum(sum(reprojection_error));
%%   

numPoints=size(X0,1)/3;
Points=reshape(X0,[3 numPoints]);
num_cameras=size(Rmat,3);
T = reshape(T_opt, [3 num_cameras]);

errVec=zeros(2*num_cameras*numPoints,1);

cur_idx=1;
for i=1:num_cameras
    for j=1:numPoints
    
        projectedPoint=K*[Rmat(:,:,i) T(:,i)]*[Points(:,j);1];
        %projectedPoint=K*[Rmat(:,:,i)'*Points(:,j)-Rmat(:,:,i)'*T(:,i)];
%         disp(size(Images{1,i}(:,j)));
%         disp(nonHomogenize(projectedPoint));
        errVec(cur_idx:cur_idx+1,1)=Images{1,i}(:,j)-nonHomogenize(projectedPoint);
        cur_idx=cur_idx+2;
    end
end

tot_error=norm(errVec);


end



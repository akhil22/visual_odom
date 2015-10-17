function [J]=computeJacTrans(K,Rmat,T_opt,X)
% J = jac_str(str_vec, args, dummy)
% return J i.e. df/dtranslation where f is the reprojection function,
% and str the structure parameters.

%	df/dstr

%	df/drot

numPoints=size(X,1)/3;
Points=reshape(X,[3 numPoints]);
numCameras=size(Rmat,3);
T = reshape(T_opt, [3 numCameras]);
homoPoints=[Points;ones(1,size(Points,2));];    %   
J=zeros(2*numCameras*numPoints,numCameras*3);   %   Initializing the size of the Jacobian
cur_idx=0;
for i=1:numCameras
    %P=K*[Rmat(:,:,i)' -Rmat(:,:,i)'*T(:,i);];
    
    P=K*[Rmat(:,:,i) T(:,i)];
    
    R = Rmat(:,:,i);
    Rt = R';

    colIdx = (i - 1) * 3 + 1;
    for j=1:numPoints
        reproj=P*[homoPoints(:,j);];
        elem1 = (-(K(1,:)*Rt(:,1))*reproj(3,:) - reproj(1,:)*(-(K(3,:)*Rt(:,1))))/(reproj(3,:)^2);
        elem2 = (-(K(1,:)*Rt(:,2))*reproj(3,:) - reproj(1,:)*(-(K(3,:)*Rt(:,2))))/(reproj(3,:)^2);
        elem3 = (-(K(1,:)*Rt(:,3))*reproj(3,:) - reproj(1,:)*(-(K(3,:)*Rt(:,3))))/(reproj(3,:)^2);
        vector1 = - [elem1 elem2 elem3];
        J(cur_idx+1, colIdx:(colIdx+2))= vector1;

        elem1 = (-(K(2,:)*Rt(:,1))*reproj(3,:) - reproj(2,:)*(-(K(3,:)*Rt(:,1))))/(reproj(3,:)^2);
        elem2 = (-(K(2,:)*Rt(:,2))*reproj(3,:) - reproj(2,:)*(-(K(3,:)*Rt(:,2))))/(reproj(3,:)^2);
        elem3 = (-(K(2,:)*Rt(:,3))*reproj(3,:) - reproj(2,:)*(-(K(3,:)*Rt(:,3))))/(reproj(3,:)^2);
        vector2 = - [elem1 elem2 elem3];
        J(cur_idx+2, colIdx:(colIdx+2))=  vector2;
        cur_idx=cur_idx+2;
    end 

end

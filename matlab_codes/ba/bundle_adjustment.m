function [T_opt] = bundle_adjustment(K,Rmat,T_opt,Images,X0)
%% Bundle adjustment over translation 
%   Detailed explanation goes here

J=computeJacTrans(K,Rmat,T_opt,X0);     %% Function 1      T_opt is initilization ,right ? Initialized exactly at groundtruth.

X_opt=X0;
%X_opt = Xold;

% X_opt = Xold+1*randn(size(Xold));

[~,err_old] =reprojectionErrorTrans(K,Rmat,T_opt,X_opt,Images);     % Function 2




maxIter=1000;
lambda=1e-1;
%lambda=1;
tol=10^3*eps;
ok_counter = 0;


fprintf(1,'Starting Reprojection Error is: %f\n', norm(err_old));

% return;


%% Below Part Goes for Levenberg marquardt Optimization 

% Optimization Iteration
for i = 1:maxIter
    
    if mod(i,20)==0
        fprintf(1,'Currently at iteration: %d\n', i);
    end
    
    J=computeJacTrans(K,Rmat,T_opt,X_opt); %

    JtJ = J'*J;                
    sizeJ = size(J,2);
    [errVec,tot_error]=reprojectionErrorTrans(K,Rmat,T_opt,X_opt,Images);
    A=JtJ+lambda*eye(size(JtJ));
    b=J'*errVec;
    d=linsolve(A,b);
    T_opt_tmp = T_opt - d;        %   UPDATE
    
    [~,err_tmp]=reprojectionErrorTrans(K,Rmat,T_opt_tmp,X_opt,Images);
    fprintf(1,'Current Error is: %f\n', err_tmp); 

  if err_tmp <= err_old,
      fprintf(1,'Error reduced\n');
    de = norm(err_old-err_tmp); %   If error has decreased less than the tolerance level, then,
    if de<tol
         break;
    end
    err_old = err_tmp;
    ok_counter = ok_counter+1;
    T_opt = T_opt_tmp;
%     X_opt = X_opt_tmp;
%     [i, err_tmp,log10(de), lambda]
    if ok_counter >20,
      lambda = lambda/1.4;
      ok_counter = 0;
    end
  else
    lambda = 2*lambda;
    ok_counter = 0;
  end

end

fprintf(1,'Now we are done.\n');
fprintf(1,'Final Reprojection Error is: %f\n', norm(err_old));








end


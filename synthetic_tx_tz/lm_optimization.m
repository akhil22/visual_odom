function [T_opt_tmp] = lm_optimization(K,T_init,ThreeD_points,new_TwoD_points)

[err_old, tota_old]=reprojection_error(K,T_init,ThreeD_points,new_TwoD_points);

err_old1=tota_old;

maxIter=300;

lambda=1e-3;  %
%lambda=100;


tol=10^3*eps;     

%tol=0;
ok_counter = 0;


fprintf(1,'Starting Reprojection Error is: %f\n', tota_old);

disp('First_lap');

%% LM Part For Optimization
% Optimization Iteration

for i = 1:maxIter
    
    if mod(i,20)==0
        fprintf(1,'Currently at iteration: %d\n', i);
    end
    
  
 J=jacobian_reproject(K,T_init,ThreeD_points);
 JtJ = J'*J;                
 sizeJ = size(J,2);
[errVec,tot_error]= reprojection_error(K,T_init,ThreeD_points,new_TwoD_points);       %  Function2
 
 
    A=JtJ+lambda*eye(size(JtJ));
    %b=J'*errVec;
    b=J'*errVec;
    %b=-b;
    d=linsolve(A,b);
    T_opt_tmp(1) = T_init(1) - d(1);        %   UPDATE
    T_opt_tmp(2) = 0;
    T_opt_tmp(3) = T_init(3) - d(2);
    %[~,err_tmp]=reprojectionErrorTrans(K,Rmat,T_opt_tmp,X_opt,Images);
    
    
    
    [errVecc,err_tmp]=reprojection_error(K,T_opt_tmp,ThreeD_points,new_TwoD_points);
   
    %fprintf(1,'Current Error is: %f\n', err_tmp); 

    if err_tmp <= err_old1,
      fprintf(1,'Error reduced\n');
    de = norm(err_old1-err_tmp); %   If error has decreased less than the tolerance level, then,
    if de<tol
         break;
    end
    err_old1 = err_tmp;
    ok_counter = ok_counter+1;
    T_init = T_opt_tmp;
%     X_opt = X_opt_tmp;
%     [i, err_tmp,log10(de), lambda]
    if ok_counter >20,
    
        lambda = lambda/1.4;
      
        ok_counter = 0;
    end
  else
    lambda = 1.2*lambda;
    ok_counter = 0;
  end

end







end


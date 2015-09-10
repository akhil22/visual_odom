function y = myobjective(x,x1)
ss = size(x1,2);
R = [cos(x(1)) 0 -sin(x(1));0 1 0;sin(x(1)) 0 cos(x(1))];
 J=R*x1+repmat(x(2:4),1,size(x1,2))-reshape(x(5:end),3,ss);
 y=sum(diag(J'*J));
end
function y = myobjectiveWithouty(x,theta,x1)
ss = size(x1,2);
R = [cos(theta) 0 -sin(theta);0 1 0;sin(theta) 0 cos(theta)];
 J=R*x1+repmat(x(1:3),1,size(x1,2))-reshape(x(4:end),3,ss);
 y=sum(diag(J'*J));
end
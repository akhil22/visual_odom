function [c ceq] = constraints(x,theta)
R = [cos(theta) 0 -sin(theta);0 1 0;sin(theta) 0 cos(theta)];
t = -R'*x(1:3);
ceq(1) = sin(theta/2)*t(3) - cos(theta/2)*t(1);
ceq(2) = t(2);
c=[];
end
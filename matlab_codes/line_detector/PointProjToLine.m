function projection=PointProjToLine(point,theta,dist)
% Project a 2-D point vector onto a line described by theta and dist
% Author: Allen Yang, August, 2003

[x,y]=size(point);
if (y==2) & (x==1)
    point =point';
elseif (x~=2) & (y~=1)
    projection = [];
    return;
end

theta = pi-theta;
sinTheta = sin(theta);
cosTheta = cos(theta);

xbar=[cosTheta -sinTheta]*point;
% Coordinate = [xbar; dist] in the rotated coordinated system.
projection = [cosTheta sinTheta; -sinTheta cosTheta]*[xbar; dist];
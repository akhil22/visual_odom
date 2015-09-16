function [nImPts]=nonHomogenize(ImPts)
%
%   Input: 
%       
%       
%   
%   Output: 
%       
%       

XPts=ImPts(1,:)./ImPts(3,:);
YPts=ImPts(2,:)./ImPts(3,:);
nImPts=[XPts;YPts;];
function [Ix,Iy]=sobel(im)
% Sobel operator, return the gradient magnitudes along both x and y axes.

[h w d]=size(im);
Ix=zeros(h,w); Iy = zeros(h,w);
if (d ~= 1)
    error('Sobel operator only compute gray level images');
    return;
end
%Sx = [-1 -2 -1; 0 0 0; 1 2 1];
%Sy = [-1 0 1; -2 0 2; -1 0 1];

for i=2:h-1
    for j=2:w-1
        Ix(i,j) = -im(i-1,j-1)-im(i-1,j)*2-im(i-1,j+1)+im(i+1,j-1)+im(i+1,j)*2+im(i+1,j+1);
        Iy(i,j) = -im(i-1,j-1)+im(i-1,j+1)-2*im(i,j-1)+2*im(i,j+1)-im(i+1,j-1)+im(i+1,j+1);        
    end
end

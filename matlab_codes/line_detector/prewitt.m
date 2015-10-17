function [Ix Iy]=prewitt(im)
% Sobel operator, return the gradient magnitudes along both x and y axes.

Ix=[]; Iy=[];
[h w d]=size(im);

if (d ~= 1)
    error('Sobel operator only compute gray level images');
    return;
end

Sx = [-1 -1 -1; 0 0 0; 1 21 1];
Sy = [-1 0 1; -1 0 1; -1 0 1];

Ix = zeros(h,w);
Iy = zeros(h,w);

for i=2: h-1
    for j=2:w-1
        xx=0;   yy=0;
        for k=1:3
            for l=1:3
                xx = xx + im(i+k-2,j+l-2)*Sx(k,l);
                yy = yy + im(i+k-2,j+l-2)*Sy(k,l);
            end
        end
        Ix(i,j)=xx;
        Iy(i,j)=yy;
    end
end
return;
clc;
clear;

im=imread('cube.jpg');
figure(1); image(im);

% Line Detection Routine
lines = line_detector(im);
lineCount = size(lines,2);

% Render the result

hold on;
for lineIndex=1:lineCount    
    X1 =[lines(2,lineIndex); lines(4,lineIndex)];
    Y1 =[lines(1,lineIndex); lines(3,lineIndex)];
    plot(X1,Y1,'LineWidth',2,'Color',[1 0 0]);
end
% Dimension: [-32..32, -32..32], range = 16;

range = 8;
ang=2*pi/range;
for i=-32:32
for j=-32:32
    test = round(atan2(j,i)/ang)+range/2;   % range of atan2 is from -pi to pi.
    if test == range
        test =0
    end
    table(i+33,j+33)= test;
end
end

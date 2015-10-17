function Line = LineFitting2(magnitude, direction)
% Line fitting using Connected Components Algorithm
% References: Jana Kosecka and Wei Zhang, Video Compass, IJCV 2002.
% limit is the minimal length of a line that will be extracted.

% Step 1, form line support regions
% Step 2, fit line parameters of the regions

[h w] = size(magnitude);
lowThreshold = floor(h*0.04);
highThreshold = floor(h*0.04);
 
element8 = [-1 -1; -1 0; -1 1; 0 -1; 0 1; 1 -1; 1 0; 1 1];
element24= [-2 -2; -2 -1; -2 0; -2 1; -2 2; -1 -2; -1 -1; -1 0; -1 1; -1 2; 0 -2; 0 -2; 0 1; 0 2; 1 -2; 1 -1; 1 0; 1 1; 1 2; 2 -2; 2 -1; 2 0; 2 1; 2 2];
N=0;    support=[]; region = [];

% Avoid boundary effect
head = 1;
for x=3:h-2
    for y=3:w-2
        if magnitude(x,y)>0     % new start point of a possible line support region
            % Init starting point
            tail = head;
            region(head,1:2)=[x y]; 
            mag = magnitude(x,y);
            dir = direction(x,y);
            magnitude(x,y)=0;
            start = head;
            while (tail>=head)
                % search new connected components
                for i=1:24   
                    xx = region(head,1) + element24(i,1);
                    yy = region(head,2) + element24(i,2);
                    if (xx>2)& (yy>2) & (xx<=h-2) & (yy<=w-2) & (magnitude(xx,yy)>0) & (abs(magnitude(xx,yy)-mag)<0.1) & (abs(direction(xx,yy)-dir)<=1)
                            tail = tail + 1;
                            region(tail,1:2)=[xx yy];
                            magnitude(xx,yy) = 0;
                    end
                end
                head = head +1;
            end
            if tail-start<=lowThreshold
                head = start;   tail = head -1;
            else
                % Step 2: register local line segmentation orientation.
                hold on;
                for i=start:tail
                    plot(region(i,2),region(i,1),'bo'); % The plot coordinate is (y,x) starting from the same upper-left
                end
                xybar=sum(region)/tail;
                xytilt=region - repmat(xybar,tail,1);
                D = zeros(2,2);
                for i=1:tail
                    xx = xytilt(i,1)*xytilt(i,1);
                    xy = xytilt(i,1)*xytilt(i,2);
                    yy = xytilt(i,2)*xytilt(i,2);
                    D=D + [xx xy; xy yy];
                end
                [Vector Value] = eig(D); %The eigenvector corresponding to the larger eigenvalue is Vector(:,2)
                theta = atan2(Vector(2,2), Vector(1,2));
                sinTheta = sin(theta);
                cosTheta = cos(theta);
                dist = xybar(1)*cosTheta+xybar(2)*sinTheta;
                N=N+1;
                support(N,1:4)=[start, tail, dir, dist];
            end
        end
    end
end

% Step 3: Search for line compounds
linenum=0;  Line = [];
for i=1:N
    if (support(i,1)~=0)
        start = support(i,1);   tail = support(i,2);    dir1=support(i,3);  dist=support(i,4);
        dir2 = mod(dir1+8,16);  % The opposite direction of dir1
        xytilt = [];
        total=0;
        for j=i:N
            if (support(j,1)~=0) & (abs(support(j,3)-dir1)<=1 | abs(support(j,3)-dir2)<=1) & (abs(support(j,4)-dist)<=30)
               for k=support(j,1):support(j,2)
                   total = total +1;
                   xytilt(total,1:2)=region(k,1:2);
               end
               support(j,1)=0;
            end
        end
        if total>highThreshold       
            xybar=sum(xytilt)/total;
            xytilt = xytilt - repmat(xybar,total,1);
            D = zeros(2,2);
            for j=1:total
                xx = xytilt(j,1)*xytilt(j,1);
                xy = xytilt(j,1)*xytilt(j,2);
                yy = xytilt(j,2)*xytilt(j,2);
                D=D + [xx xy; xy yy];
            end
            [Vector Value] = eig(D); %The eigenvector corresponding to the larger eigenvalue is Vector(:,2)
            theta = atan2(Vector(2,2), Vector(1,2));
            sinTheta = sin(theta);
            cosTheta = cos(theta);
            dist = xybar(1)*sinTheta-xybar(2)*cosTheta;       
            % Compute the two end points of the ideal line on
            % the image boundary.
            endpoints = zeros(2,2); head = 0;
            % case 1: y==1 axis
            xx = floor((dist-cosTheta) / sinTheta);
            if (xx>0) & (xx<=h)
                head = head +1;
                endpoints(:,head)=[xx; 1];
            end
            % case 2: x==1 axis
            yy = floor((sinTheta-dist)/cosTheta);
            if (yy>0) & (yy<=w)
                head = head +1;
                endpoints(:,head)=[1;yy];
            end
            % case 3: y==w axis
            xx = floor((dist+w*cosTheta)/sinTheta);
            if (xx>0) & (xx<=h)
                head = head +1;
                endpoints(:,head) = [xx;w];
            end
            % case 4: x==h axis
            yy = floor((h*sinTheta-dist)/cosTheta);
            if (yy>0) & (yy<=w)
                head = head +1;
                endpoints(:,head)=[h;yy];
            end
            
            % Step 4, finally register the line associated to the two
            % points. Note that the line is uncalibrated.
            linenum = linenum + 1;
            Line(1:2,linenum) = endpoints(1:2,1);
            linenum = linenum + 1;
            Line(1:2,linenum) = endpoints(1:2,2);
            
            % Visualization Step: Draw a line from endpoints(:,1) to
            % endpoints(:,2)
            X1 =[endpoints(2,1); endpoints(2,2)];
            Y1 =[endpoints(1,1); endpoints(1,2)];
            hold on;
            plot(X1,Y1,'LineWidth',2,'Color',[1 0 0]);
        end
    end
end
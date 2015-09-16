clear all
clc
close all
T=zeros(200,3);
for i=2:100
    
    
    T(i,1)=T(i,1)+rand*(0.01);
    T(i,2)=0;
    T(i,3)=T(i-1,3)+1*rand;
    T(1,1)=0.2; 
end 


for j=101:200

    T(j,3)=T(j-1,3)+rand*(0.01);
    T(j,2)=0;
    
    T(j,1)=T(j-1,1)+1*rand;


end 



figure(1),plot(T(:,1),T(:,3))
axis([-10 10 -100 200])
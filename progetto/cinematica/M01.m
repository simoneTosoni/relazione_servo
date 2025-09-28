function M01 = M01(Q,L)
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here

a=Q(1);
b=Q(2);
s=Q(3);
g=Q(4);

h =L(1);     
l1 =L(2);    
l2 =L(3);    
l3 =L(4);


M01=zeros(4,4);
M01(1,1)=cos(a);
M01(1,2)=-sin(a);
M01(1,3)=0;
M01(1,4)=l1*cos(a);

M01(2,1)=sin(a);
M01(2,2)=cos(a);
M01(2,3)=0;
M01(2,4)=l1*sin(a);

M01(3,1)=0;
M01(3,2)=0;
M01(3,3)=1;
M01(3,4)=h;

M01(4,1)=0;
M01(4,2)=0;
M01(4,3)=0;
M01(4,4)=1;

end
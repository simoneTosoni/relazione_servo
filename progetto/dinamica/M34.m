function M34 = M34(Q,L)
%UNTITLED4 Summary of this function goes here
%   Detailed explanation goes here

a=Q(1);
b=Q(2);
s=Q(3);
g=Q(4);

h =L(1);     
l1 =L(2);    
l2 =L(3);    
l3 =L(4);


M34=zeros(4,4);
M34(1,1)=cos(g);
M34(1,2)=-sin(g);
M34(1,3)=0;
M34(1,4)=0;

M34(2,1)=sin(g);
M34(2,2)=cos(g);
M34(2,3)=0;
M34(2,4)=0;

M34(3,1)=0;
M34(3,2)=0;
M34(3,3)=1;
M34(3,4)=0;

M34(4,1)=0;
M34(4,2)=0;
M34(4,3)=0;
M34(4,4)=1;

end
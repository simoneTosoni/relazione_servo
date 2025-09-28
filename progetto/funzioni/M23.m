function M23 = M23(Q,L)
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


M23=zeros(4,4);
M23(1,1)=1;
M23(1,2)=0;
M23(1,3)=0;
M23(1,4)=0;

M23(2,1)=0;
M23(2,2)=1;
M23(2,3)=0;
M23(2,4)=0;

M23(3,1)=0;
M23(3,2)=0;
M23(3,3)=1;
M23(3,4)=-s;

M23(4,1)=0;
M23(4,2)=0;
M23(4,3)=0;
M23(4,4)=1;
end
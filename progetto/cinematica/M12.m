function M12 = M12(Q,L)
%UNTITLED2 Summary of this function goes here
%   Detailed explanation goes here

a=Q(1);
b=Q(2);
s=Q(3);
g=Q(4);

h =L(1);     
l1 =L(2);    
l2 =L(3);    
l3 =L(4);


M12=zeros(4,4);
M12(1,1)=cos(b);
M12(1,2)=-sin(b);
M12(1,3)=0;
M12(1,4)=l2*cos(b);

M12(2,1)=sin(b);
M12(2,2)=cos(b);
M12(2,3)=0;
M12(2,4)=l2*sin(b);

M12(3,1)=0;
M12(3,2)=0;
M12(3,3)=1;
M12(3,4)=0;

M12(4,1)=0;
M12(4,2)=0;
M12(4,3)=0;
M12(4,4)=1;
end
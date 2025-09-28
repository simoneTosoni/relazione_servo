function Je = SCARAJacEsteso(Q,L,G)
% ## Je = SCARAJacEsteso(Q,L,G)
% ## Se=[x, y, z, c, xg3, yg3, zg3, xg2, yg2, zg2, alpha+beta, xg1, yg1, zg1, alpha];

a=Q(1);
b=Q(2);
s=Q(3);
g=Q(4);

h=L(1);
l1=L(2);
l2=L(3);
l3=L(4);

gh=G(1);
g1=G(2);
g2=G(3);
g3=G(4);

Je=zeros(15,4);
Je(1,1)=(-1)*l1*sin(a)-l2*sin(a+b); 
Je(1,2)=(-1)*l2*sin(a+b);

Je(2,1)=l1*cos(a)+l2*cos(a+b); 
Je(2,2)=l2*cos(a+b);

Je(3,3)=-1;

Je(4,1)=1;
Je(4,2)=1;
Je(4,4)=1;

Je(5,1)=(-1)*l1*sin(a)-l2*sin(a+b); 
Je(5,2)=(-1)*l2*sin(a+b);

Je(6,1)=l1*cos(a)+l2*cos(a+b); 
Je(6,2)=l2*cos(a+b);

Je(7,3)=-1;

Je(8,1)=(-1)*l1*sin(a)-g2*sin(a+b); 
Je(8,2)=(-1)*g2*sin(a+b);

Je(9,1)=l1*cos(a)+g2*cos(a+b); 
Je(9,2)=g2*cos(a+b);

Je(10,1)=0; 
Je(10,2)=0;

Je(11,1)=1; 
Je(11,2)=1;

Je(12,1)=(-1)*g1*sin(a); 
Je(12,2)=0;

Je(13,1)=g1*cos(a); 
Je(13,2)=0;

Je(15,1)=1; 
Je(15,2)=0;

end
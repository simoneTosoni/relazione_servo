function Jep = SCARAJacpEsteso(Q,Qp,L,G)
% ## Je = SCARAJacpEsteso(Q,Qp,L,G)
% ## Se=[x, y, z, c, xg3, yg3, zg3, xg2, yg2, zg2, alpha+beta, xg1, yg1, zg1, alpha];

a=Q(1);
b=Q(2);
s=Q(3);
g=Q(4);

ap=Qp(1);
bp=Qp(2);
sp=Qp(3);
gp=Qp(4);

h=L(1);
l1=L(2);
l2=L(3);
l3=L(4);

gh=G(1);
g1=G(2);
g2=G(3);
g3=G(4);

Jep=zeros(15,4);
Jep(1,1)=(-1)*l1*cos(a)*ap-l2*cos(a+b)*(ap+bp); 
Jep(1,2)=(-1)*l2*cos(a+b)*(ap+bp);

Jep(2,1)=(-1)*l1*sin(a)*ap-l2*sin(a+b)*(ap+bp); 
Jep(2,2)=(-1)*l2*sin(a+b)*(ap+bp);

Jep(5,1)=(-1)*l1*cos(a)*ap-l2*cos(a+b)*(ap+bp); 
Jep(5,2)=(-1)*l2*cos(a+b)*(ap+bp);

Jep(6,1)=(-1)*l1*sin(a)*ap-l2*sin(a+b)*(ap+bp); 
Jep(6,2)=(-1)*l2*sin(a+b)*(ap+bp);

Jep(8,1)=(-1)*l1*cos(a)*ap-g2*cos(a+b)*(ap+bp); 
Jep(8,2)=(-1)*g2*cos(a+b)*(ap+bp);

Jep(9,1)=(-1)*l1*sin(a)*ap-g2*sin(a+b)*(ap+bp); 
Jep(9,2)=(-1)*g2*sin(a+b)*(ap+bp);

Jep(12,1)=(-1)*g1*cos(a)*ap;


Jep(13,1)=(-1)*g1*sin(a)*ap;


end
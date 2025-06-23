function J = SCARAjac(Q,L)
%SCARAJACOBIANO jacobiano di un robot SCARA
% L = [h l1 l2 l3]';
% Q = [j1 j2 s g];

h=L(1);
l1=L(2);
l2=L(3);
l4=L(4);

j1=Q(1);
j2=Q(2);
s=Q(3);
g=Q(4);

J=zeros(4,4);
J(1,1)=-l1*sin(j1)-l2*sin(j1+j2);   J(1,2)=-l2*sin(j1+j2);    
J(1,3)=0;   J(1,4)=0;
J(2,1)= l1*cos(j1)+l2*cos(j1+j2);   J(2,2)= l2*cos(j1+j2);    
J(2,3)=0;   J(2,4)=0;
J(3,1)=0;                           J(3,2)=0;                 
J(3,3)=-1;  J(3,4)=0;
J(4,1)=1;                           J(4,2)=1;                 
J(4,3)=0;   J(4,4)=1;
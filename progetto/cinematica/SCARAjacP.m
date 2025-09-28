function Jp = SCARAjacP(Q,Qp,L)
%SCARAJACP derivata rispetto al tempo dello jacobiano esteso di un robot SCARA
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

Jp=zeros(4,4);
Jp(1,1)=-l1*cos(Q(1))*Qp(1)-l2*cos(Q(1)+Q(2))*(Qp(1)+Qp(2));   Jp(1,2)=-l2*cos(Q(1)+Q(2))*(Qp(1)+Qp(2));      Jp(1,3)=0; 
Jp(2,1)=-l1*sin(Q(1))*Qp(1)-l2*sin(Q(1)+Q(2))*(Qp(1)+Qp(2));   Jp(2,2)=-l2*sin(Q(1)+Q(2))*(Qp(1)+Qp(2));      Jp(1,3)=0;
Jp(3,1)=0;                                                     Jp(3,2)=0;                                     Jp(3,3)=0;
Jp(4,1)=0;                                                     Jp(4,2)=0;                                     Jp(4,3)=0;    Jp(4,4)=0;
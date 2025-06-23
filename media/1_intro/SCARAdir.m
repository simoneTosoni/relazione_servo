function S= SCARAdir(Q,L)
  % SCARA DIRETTO cinematica dir. robot SCARA
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
  
  S=zeros(4,1);
  S(1)=l1*cos(j1)+l2*cos(j1+j2);
  S(2)=l1*sin(j1)+l2*sin(j1+j2);
  S(3)=h-s;
  S(4)=j1+j2+g;

end
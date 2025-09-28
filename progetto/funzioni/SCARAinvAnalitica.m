function [Q] = SCARAinvAnalitica(S,L)
x=S(1);
y=S(2);
z=S(3);
phi=S(4);

h=L(1);
l1=L(2);
l2=L(3);
l3=L(4);


q2 = acos((x^2+y^2-l1^2-l2^2)/(2*l1*l2));             % Position joint 2
q1 = atan2(y,x)-atan2(l2*sin(q2),l1+l2*cos(q2));      % Position joint 1
q3 = h-z;
q4 = phi-q1-q2;

Q=[q1 q2 q3 q4]';
end
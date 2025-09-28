function Se = SCARAdirEstesa(Q,L,G)
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here

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

Se=[l1*cos(a)+l2*cos(a+b),
    l1*sin(a)+l2*sin(a+b),
    h-s,
    a+b+g,
    l1*cos(a)+l2*cos(a+b),
    l1*sin(a)+l2*sin(a+b),
    h-s+g3,
    l1*cos(a)+g2*cos(a+b),
    l1*sin(a)+g2*sin(a+b),
    h,
    a+b,
    g1*cos(a),
    g1*sin(a),
    h,
    a];

end
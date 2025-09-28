function R = Rot3(v_piano,alpha)
% 
x=v_piano(1);
y=v_piano(2);
z=v_piano(3);

a=alpha;

R=[x^2+(1-x^2)*cos(a) x*y*(1-cos(a))-z*sin(a) x*z*(1-cos(a))+y*sin(a) 0;
   x*y*(1-cos(a))+z*sin(a) y^2+(1-y^2)*cos(a) y*z*(1-cos(a))-x*sin(a) 0;
   x*z*(1-cos(a))-y*sin(a) y*z*(1-cos(a))+x*sin(a) z^2+(1-z^2)*cos(a) 0;
   0 0 0 1];

end
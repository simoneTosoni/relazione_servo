function [x,xp,xpp] = cicloidale(t,T,s0,ds)
%UNTITLED3 Summary of this function goes here
%   Detailed explanation goes here

xpp=ds/T^2*2*pi*sin(2*pi*t/T);
xp=ds/T*(1-cos(2*pi*t/T));
x=ds*((t/T)-1/(2*pi)*sin(2*pi*t/T))+s0;

end

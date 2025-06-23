function [x,xp,xpp] = cicloidale(t,T,s0,ds)
% Legge di moto cicloidale 
% t: istante di tempo in cui calcolare la posizione
% T: durata totale del moto
% s0: posizione iniziale del moto
% ds: differenza tra posizione finale e iniziale della traiettoria

xpp=ds/T^2*2*pi*sin(2*pi*t/T);
xp=ds/T*(1-cos(2*pi*t/T));
x=ds*((t/T)-1/(2*pi)*sin(2*pi*t/T))+s0;

end

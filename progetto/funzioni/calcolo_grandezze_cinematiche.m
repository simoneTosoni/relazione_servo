clc

% s ordinata -> ascissa
% t ascissa -> ordinata
Tcumulata=interp1(s, t, Scumulata);
Tcumulata(end)=t(end);

% figure
% plot(s,t,Scumulata,Tcumulata,'r*-')
% grid on
% title("Interpolazione")

% figure
% subplot(3,1,1)
% plot(Tcumulata,x,'-')
% grid on
% subplot(3,1,2)
% plot(Tcumulata,y,'-')
% grid on
% subplot(3,1,3)
% plot(Tcumulata,z,'-')
% grid on

pp_x=spline(Tcumulata,x);
pp_y=spline(Tcumulata,y);
pp_z=spline(Tcumulata,z);

pp_vx=fnder(pp_x);
pp_vy=fnder(pp_y);
pp_vz=fnder(pp_z);

vx=ppval(pp_vx,Tcumulata);
vy=ppval(pp_vy,Tcumulata);
vz=ppval(pp_vz,Tcumulata);

v=[vx;vy;vz];

% figure
% subplot(3,1,1)
% plot(Tcumulata,vx,'-')
% grid on
% subplot(3,1,2)
% plot(Tcumulata,vy,'-')
% grid on
% subplot(3,1,3)
% plot(Tcumulata,vz,'-')
% grid on

x_t_sp = nodi_trasformati(1,2) + cumtrapz(Tcumulata, vx);
y_t_sp = nodi_trasformati(2,2) + cumtrapz(Tcumulata, vy);
z_t_sp = nodi_trasformati(3,2) + cumtrapz(Tcumulata, vz);

figure(1)
plot3(x_t_sp,y_t_sp,z_t_sp,'LineWidth',1.5)
grid on

a=[gradient(vx,Tcumulata);gradient(vy,Tcumulata);gradient(vz,Tcumulata)];




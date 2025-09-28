close all
clear all
clc

geometria_SCARA
% CINEMATICA INVERSA
    
traiettoria_prova_fisso

Qi=SCARAinvAnalitica(Si,L);
Qf=SCARAinvAnalitica(Sf,L);


T=5;
n=100;
dt=T/(n-1);
tt=0:dt:T;

for i=1:n
    t=(i-1)*T/(n-1);
    tt(i)=t;
    [x(i) xd(i) xdd(i)]=cicloidale(t,T,Si(1),Sf(1)-Si(1));
    [y(i) yd(i) ydd(i)]=cicloidale(t,T,Si(2),Sf(2)-Si(2));
    [z(i) zd(i) zdd(i)]=cicloidale(t,T,Si(3),Sf(3)-Si(3));
    [g(i) gd(i) gdd(i)]=cicloidale(t,T,Si(4),Sf(4)-Si(4));
    
    S(:,i)=[x(i) y(i) z(i) g(i)]';
    Sd(:,i)=[xd(i) yd(i) zd(i) gd(i)]';
    Sdd(:,i)=[xdd(i) ydd(i) zdd(i) gdd(i)]';
    
    QQ(:,i)=SCARAinvAnalitica(S(:,i),L);
    J=SCARAjac(QQ(:,i),L);
    QQd(:,i)=pinv(J)*Sd(:,i);
    Jp=SCARAjacP(QQ(:,i),QQd(:,i),L);
    QQdd(:,i)=J^(-1)*(Sdd(:,i)-Jp*QQd(:,i));
    Q0=QQ(:,i);
    
end


figure(3)
plot(tt,S(1,:),tt,S(2,:),tt,S(3,:))
grid on
ylim([-1 1.2])
xlabel("tempo [s]")
ylabel("[m]")
legend('x','y','z','Location','best')
title('Coordinate posizione gripper')

figure(4)
plot(tt,Sd(1,:),tt,Sd(2,:),tt,Sd(3,:),tt(1:end-1),diff(S(1,:))./diff(tt(1:end)),tt(1:end-1),diff(S(2,:))./diff(tt(1:end)),tt(1:end-1),diff(S(3,:))./diff(tt(1:end)),[tt(1) tt(end)],[0 0],'k')
grid on
xlabel("tempo [s]")
ylabel("[m/s]")
legend('xd','yd','zd','xd#','yd#','zd#','Location','best')
title('Velocità gripper')

figure(5)
plot(tt,Sdd(1,:),tt,Sdd(2,:),tt,Sdd(3,:),tt(1:end-1),diff(Sd(1,:))./diff(tt(1:end)),tt(1:end-1),diff(Sd(2,:))./diff(tt(1:end)),tt(1:end-1),diff(Sd(3,:))./diff(tt(1:end)),[tt(1) tt(end)],[0 0],'k')
grid on
xlabel("tempo [s]")
ylabel("[m/s^2]")
legend("xdd",'ydd','zdd','xdd#','ydd#','zdd#',"Location",'best')
title('Accelerazioni gripper')

figure(6)
subplot(3,1,1,"align")
plot(tt,S(4,:),[tt(1) tt(end)],[0 0],'k')
grid on
xlabel("tempo [s]")
ylabel("[rad]")
legend('g',"Location",'best')
title('Orientamento gripper')
subplot(3,1,2,"align")
plot(tt,Sd(4,:),tt(1:end-1),diff(S(4,:))./diff(tt(1:end)),[tt(1) tt(end)],[0 0],'k')
grid on
xlabel("tempo [s]")
ylabel("[rad/s]")
legend('gd','gd# ')
title('Velocità orientamento gripper')
subplot(3,1,3,"align")
plot(tt,Sdd(4,:),tt(1:end-1),diff(Sd(4,:))./diff(tt(1:end)),[tt(1) tt(end)],[0 0],'k')
grid on
xlabel("tempo [s]")
ylabel("[rad/s^2]")
legend('gdd','gdd# ',"Location",'best')
title("Accelerazione orientamento gripper");

figure(7)
plot(tt,QQ(1,:),tt,QQ(2,:),tt,QQ(4,:))
grid on
xlabel("tempo [s]")
ylabel("[rad]")
legend('j1','j2','g','Location','best')
title('Coordinate giunti rotoidali')

figure(8)
plot(tt,QQd(1,:),tt,QQd(2,:),tt,QQd(4,:),tt(1:end-1),diff(QQ(1,:))./diff(tt(1:end)),tt(1:end-1),diff(QQ(2,:))./diff(tt(1:end)),tt(1:end-1),diff(QQ(4,:))./diff(tt(1:end)),[tt(1) tt(end)],[0 0],'k')
grid on
xlabel("tempo [s]")
ylabel("[rad/s]")
legend('j1d','j2d','gd]','j1d#','j2d#','gd#','Location','best')
title('Velocità giunti rotoidali')

figure(9)
plot(tt,QQdd(1,:),tt,QQdd(2,:),tt,QQdd(4,:),tt(1:end-1),diff(QQd(1,:))./diff(tt(1:end)),tt(1:end-1),diff(QQd(2,:))./diff(tt(1:end)),tt(1:end-1),diff(QQd(4,:))./diff(tt(1:end)),[tt(1) tt(end)],[0 0],'k')
grid on
xlabel("tempo [s]")
ylabel("[rad/s^2]")
legend("j1dd",'j2dd','gdd','j1dd#','j2dd#','gdd#',"Location",'best')
title('Accelerazioni giunti rotoidali')

figure(10)
subplot(3,1,1,"align")
plot(tt,QQ(3,:),[tt(1) tt(end)],[0 0],'k')
grid on
xlabel("tempo [s]")
ylabel("[m]")
legend('s',"Location",'best')
title('Posizione giunto prismatico')
subplot(3,1,2,"align")
plot(tt,QQd(3,:),tt(1:end-1),diff(QQ(3,:))./diff(tt(1:end)),[tt(1) tt(end)],[0 0],'k')
grid on
xlabel("tempo [s]")
ylabel("[m/s]")
legend('sd','sd# ')
title('Velocità giunto prismatico')
subplot(3,1,3,"align")
plot(tt,QQdd(3,:),tt(1:end-1),diff(QQd(3,:))./diff(tt(1:end)),[tt(1) tt(end)],[0 0],'k')
grid on
xlabel("tempo [s]")
ylabel("[m/s^2]")
legend('sdd','sdd# ',"Location",'best')
title("Accelerazione giunto prismatico");

figure(11)
plot_SCARA_trajectory(Qi,Qf,S(:,:),L,11,[26 23])

% exportgraphics(figure(3),"pos_fisso.eps",'ContentType', 'vector')
% exportgraphics(figure(4),"vel_fisso.eps",'ContentType', 'vector')
% exportgraphics(figure(5),"acc_fisso.eps",'ContentType', 'vector')
% exportgraphics(figure(7),"pos_giunti.eps",'ContentType', 'vector')
% exportgraphics(figure(8),"vel_giunti.eps",'ContentType', 'vector')
% exportgraphics(figure(9),"acc_giunti.eps",'ContentType', 'vector')
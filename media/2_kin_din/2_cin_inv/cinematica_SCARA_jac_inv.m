close all
clc

geometria_SCARA

% CINEMATICA INVERSA
    
traiettoria_prova_fisso

[Qi val]=SCARAinv(Si,L,Qi0,1e-6,55)
[Qf val]=SCARAinv(Sf,L,Qf0,1e-6,55)

T=5;
n=100;
dt=T/(n-1);
tt=0:dt:T;
Q0=Qi0;

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
    
    QQ(:,i)=SCARAinv(S(:,i),L,Q0,1e-6,55);
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
legend('x [m]','y [m]','z [m]','Location','best')
title('Coordinate posizione gripper')

figure(4)
plot(tt,Sd(1,:),tt,Sd(2,:),tt,Sd(3,:),tt(1:end-1),diff(S(1,:))./diff(tt(1:end)),tt(1:end-1),diff(S(2,:))./diff(tt(1:end)),tt(1:end-1),diff(S(3,:))./diff(tt(1:end)),[tt(1) tt(end)],[0 0],'k')
grid on
legend('xd [m/s]','yd [m/s]','zd [m/s]','xd#','yd#','zd#','Location','best')
title('Velocita' gripper')

figure(5)
plot(tt,Sdd(1,:),tt,Sdd(2,:),tt,Sdd(3,:),tt(1:end-1),diff(Sd(1,:))./diff(tt(1:end)),tt(1:end-1),diff(Sd(2,:))./diff(tt(1:end)),tt(1:end-1),diff(Sd(3,:))./diff(tt(1:end)),[tt(1) tt(end)],[0 0],'k')
grid on
legend("xdd [m/s^2]",'ydd [m/s^2]','zdd [m/s^2]','xdd#','ydd#','zdd#',"Location",'best')
title('Accelerazioni gripper')

figure(6)
subplot(3,1,1,"align")
plot(tt,SSdd(4,:),[tt(1) tt(end)],[0 0],'k')
grid on
legend('g [rad]',"Location",'best')
title('Orientamento gripper')
subplot(3,1,2,"align")
plot(tt,Sd(4,:),tt(1:end-1),diff(S(4,:))./diff(tt(1:end)),[tt(1) tt(end)],[0 0],'k')
grid on
legend('gd [rad/s]','gd# ')
title('Velocita' orientamento gripper')
subplot(3,1,3,"align")
plot(tt,Sdd(4,:),tt(1:end-1),diff(Sd(4,:))./diff(tt(1:end)),[tt(1) tt(end)],[0 0],'k')
grid on
legend('gdd [rad/s^2]','gdd# ',"Location",'best')
title("Accelerazione orientamento gripper");

figure(7)
plot(tt,QQ(1,:),tt,QQ(2,:),tt,QQ(4,:))
grid on
legend('j1 [rad]','j2 [rad]','g [rad]','Location','best')
title('Coordinate giunti rotoidali')

figure(8)
plot(tt,QQd(1,:),tt,QQd(2,:),tt,QQd(4,:),tt(1:end-1),diff(QQ(1,:))./diff(tt(1:end)),tt(1:end-1),diff(QQ(2,:))./diff(tt(1:end)),tt(1:end-1),diff(QQ(4,:))./diff(tt(1:end)),[tt(1) tt(end)],[0 0],'k')
grid on
legend('j1d [rad/s]','j2d [rad/s]','gd [rad/s]','j1d#','j2d#','gd#','Location','best')
title('Velocita' giunti rotoidali')

figure(9)
plot(tt,QQdd(1,:),tt,QQdd(2,:),tt,QQdd(4,:),tt(1:end-1),diff(QQd(1,:))./diff(tt(1:end)),tt(1:end-1),diff(QQd(2,:))./diff(tt(1:end)),tt(1:end-1),diff(QQd(4,:))./diff(tt(1:end)),[tt(1) tt(end)],[0 0],'k')
grid on
legend("j1dd [rad/s^2]",'j2dd [rad/s^2]','gdd [rad/s^2]','j1dd#','j2dd#','gdd#',"Location",'best')
title('Accelerazioni giunti rotoidali')

figure(10)
subplot(3,1,1,"align")
plot(tt,QQ(3,:),[tt(1) tt(end)],[0 0],'k')
grid on
legend('s [m]',"Location",'best')
title('Posizione giunto prismatico')
subplot(3,1,2,"align")
plot(tt,QQd(3,:),tt(1:end-1),diff(QQ(3,:))./diff(tt(1:end)),[tt(1) tt(end)],[0 0],'k')
grid on
legend('sd [m/s]','sd# ')
title('Velocita' giunto prismatico')
subplot(3,1,3,"align")
plot(tt,QQdd(3,:),tt(1:end-1),diff(QQd(3,:))./diff(tt(1:end)),[tt(1) tt(end)],[0 0],'k')
grid on
legend('sdd [m/s^2]','sdd# ',"Location",'best')
title("Accelerazione giunto prismatico");

figure(11)
plot_SCARA_trajectory(Qi,Qf,S(:,:),L,11,[26 23])

% Cinematica diretta e inversa robot SCARA per il progetto di Servo System and Robotics 
% Tosoni Simone, matricola 731405

clear 
close all
clc

geometria_SCARA

% CINEMATICA DIRETTA 
traiettoria_prova_giunti

% caratteristiche della legge di moto
T=5;
n=100;
dt=T/(n-1);
tt=0:dt:T;

for i=1:n
    t=(i-1)*T/(n-1);
    tt(i)=t;
    [a(i) ad(i) add(i)]=cicloidale(t,T,Qi(1),Qf(1)-Qi(1));
    [b(i) bd(i) bdd(i)]=cicloidale(t,T,Qi(2),Qf(2)-Qi(2));
    [s(i) sd(i) sdd(i)]=cicloidale(t,T,Qi(3),Qf(3)-Qi(3));
    [g(i) gd(i) gdd(i)]=cicloidale(t,T,Qi(4),Qf(4)-Qi(4));
    
    Q(:,i)=[a(i) b(i) s(i) g(i)]';
    S(:,i)=SCARAdir(Q(:,i),L);
    
    Qd(:,i)=[ad(i) bd(i) sd(i) gd(i)]';
    Sd(:,i)=SCARAjac(Q(:,i),L)*Qd(:,i);
    
    Qdd(:,i)=[add(i) bdd(i) sdd(i) gdd(i)]';
    Sdd(:,i)=SCARAjac(Q(:,i),L)*Qdd(:,i)+SCARAjacP(Q(:,i),Qd(:,i),L)*Qd(:,i);
end

figure(3)
plot(tt,S(1,:),tt,S(2,:),tt,S(3,:))
grid on
ylim([-0.8 1.4])
xlabel("t [s]")
ylabel("[m]")
legend('x','y','z','Location','best')
title('Coordinate posizione gripper')

figure(4)
plot(tt,Sd(1,:),tt,Sd(2,:),tt,Sd(3,:),tt(1:end-1),diff(S(1,:))./diff(tt(1:end)),tt(1:end-1),diff(S(2,:))./diff(tt(1:end)),tt(1:end-1),diff(S(3,:))./diff(tt(1:end)),[tt(1) tt(end)],[0 0],'k')
grid on
xlabel("t [s]")
ylabel("[m/s]")
legend('xd','yd','zd','xd#','yd#','zd#','Location','best')
title('Velocità gripper')

figure(5)
plot(tt,Sdd(1,:),tt,Sdd(2,:),tt,Sdd(3,:),tt(1:end-1),diff(Sd(1,:))./diff(tt(1:end)),tt(1:end-1),diff(Sd(2,:))./diff(tt(1:end)),tt(1:end-1),diff(Sd(3,:))./diff(tt(1:end)),[tt(1) tt(end)],[0 0],'k')
grid on
xlabel("t [s]")
ylabel("[m/s^2]")
legend("xdd",'ydd','zdd','xdd#','ydd#','zdd#',"Location",'best')
title('Accelerazioni gripper')

figure(6)
subplot(3,1,1,"align")
plot(tt,S(4,:),[tt(1) tt(end)],[0 0],'k')
xlabel("t [s]")
ylabel("[rad]")
grid on
legend('g ',"Location",'best')
title('Orientamento gripper')
subplot(3,1,2,"align")
plot(tt,Sd(4,:),tt(1:end-1),diff(S(4,:))./diff(tt(1:end)),[tt(1) tt(end)],[0 0],'k')
xlabel("t [s]")
ylabel("[rad/s]")
grid on
legend('gd ','gd# ')
title('Velocità orientamento gripper')
subplot(3,1,3,"align")
plot(tt,Sdd(4,:),tt(1:end-1),diff(Sd(4,:))./diff(tt(1:end)),[tt(1) tt(end)],[0 0],'k')
xlabel("t [s]")
ylabel("[rad/s^2]")
grid on
legend('gdd ','gdd# ',"Location",'best')
title("Accelerazione orientamento gripper");

figure(7)
plot_SCARA_trajectory(Qi,Qf,S,L,7,[33 319])






% Cinematica robot SCARA per il progetto di Servo System and Robotics 
% Tosoni Simone, matricola 731405

clear all
close all
clc

geometria_SCARA

Qi=[0 0 0 0]';  % Pos iniziale traiettoria
Qf=[pi/4 -pi/4 0.1 pi/2]'; % pos finale traiettoria

% caratteristiche della legge di moto
T=5;
n=100;
dt=T/(n-1);
tt=0:dt:T;

% definizione della traiettoria ai giunti
Q=zeros(4,n);
for i=1:n
    t=(i-1)*T/(n-1);
    tt(i)=t;
    [a(i) ad(i) add(i)]=cicloidale(t,T,Qi(1),Qf(1)-Qi(1));
    [b(i) bd(i) bdd(i)]=cicloidale(t,T,Qi(2),Qf(2)-Qi(2));
    [s(i) sd(i) sdd(i)]=cicloidale(t,T,Qi(3),Qf(3)-Qi(3));
    [g(i) gd(i) gdd(i)]=cicloidale(t,T,Qi(4),Qf(4)-Qi(4));

    % definizione delle matrici M del robot e calcolo delle posizioni della
    % traiettoria
    Q(:,i)=[a(i) b(i) s(i) g(i)]';
    m01=M01(Q(:,i),L);
    m12=M12(Q(:,i),L);
    m23=M23(Q(:,i),L);
    m34=M34(Q(:,i),L);
    M=m01*m12*m23*m34;
    S(:,i)=[M(1,4),M(2,4),M(3,4)]; % posizioni gripper x,y,z assolute
    Sr(i)=acos(M(1,1));

    % definizione delle matrici L riferite al loro polo i-1
    L010=[0 -1 0 0;
          1  0 0 0;
          0  0 0 0;
          0  0 0 0];
    L121=[0 -1 0 0;
          1  0 0 0;
          0  0 0 0;
          0  0 0 0];
    L232=[0 0 0 0;
          0 0 0 0;
          0 0 0 -1;
          0 0 0 0];
    L343=[0 -1 0 0;
          1 0 0 0;
          0 0 0 0;
          0 0 0 0];
          
    
    % calcolo delle matrici definite rispetto al polo 0 origine del sdr
    % mondo
    L120=(m01*L121*m01^-(1));
    m02=m01*m12;
    L230=(m02*L232*m02^(-1));
    m03=m01*m12*m23;
    L340=(m03*L343*pinv(m03));
    m04=m01*m12*m23*m34;

    % calcolo delle velocità x,y,z
    W010=L010*ad(i);
    W121=L121*bd(i);
    W232=L232*sd(i);
    W343=L343*gd(i);

    W120=m01*W121*m01^(-1);
    
    W231=m12*W232*pinv(m12);
    W230=m01*W231*pinv(m01);
    
    W342=m23*W343*pinv(m23);
    W341=m12*W342*m12^(-1);
    W340=m01*W341*m01^(-1);
    
    W02=W010+W120;
    W03=W010+W120+W230;
    W04=W010+W120+W230+W340;
    
    Sd(:,i)=W04*m04(:,4);
    Srd(i)=W04(2,1);

    % calcolo delle accelerazioni del gripper 
    H010=L010*add(i)+L010^(2)*ad(i)^2;
    H121=L121*bdd(i)+L121^(2)*bd(i)^2;
    H232=L232*sdd(i)+L232^(2)*sd(i)^2;
    H343=L343*gdd(i)+L343^(2)*sd(i)^2;
    
    H120=m01*H121*m01^(-1);
    
    H231=m12*H232*pinv(m12);
    H230=m01*H231*pinv(m01);
    
    H342=m23*H343*pinv(m23);
    H341=m12*H342*pinv(m12);
    H340=m01*H341*pinv(m01);
    
    H020=H010+H120+2*W010*W120;
    H030=H020+H230+2*W02*W230;
    H040=H030+H340+2*W03*W340;

    Sdd(:,i)=H040*m04(:,4);
    Srdd(i)=H040(2,1);

end

figure(1)
plot_SCARA_trajectory(Qi,Qf,S,L,1,[27 50])

figure(2)
subplot(2,2,1)
plot(tt,a,tt,ad,tt,add)
legend('alpha','ad','add')
grid on
subplot(2,2,2)
plot(tt,b,tt,bd,tt,bdd)
legend('beta','bd','bdd')
grid on
subplot(2,2,3)
plot(tt,s,tt,sd,tt,sdd)
legend('s','sd','sdd')
grid on
subplot(2,2,4)
plot(tt,g,tt,gd,tt,gdd)
legend('gamma','gd','gdd')
grid on


figure(3)
plot(tt,S(1,:),tt,Sd(1,:),tt(1:end-1),diff(S(1,:))./diff(tt(1:end)),tt,Sdd(1,:),tt(1:end-1),diff(Sd(1,:))./diff(tt(1:end)))
grid on
legend('x','xd','xd#','xdd','xdd#')
title('Posizioni x traiettoria')

figure(4)
plot(tt,S(2,:),tt,Sd(2,:),tt(1:end-1),diff(S(2,:))./diff(tt(1:end)),tt,Sdd(2,:),tt(1:end-1),diff(Sd(2,:))./diff(tt(1:end)))
grid on
% ylim([-0.2 1.4])
legend('y','yd','yd#','ydd','ydd#')
title('Posizioni y traiettoria')

figure(5)
plot(tt,S(3,:),tt,Sd(3,:),tt(1:end-1),diff(S(3,:))./diff(tt(1:end)),tt,Sdd(3,:),tt(1:end-1),diff(Sd(3,:))./diff(tt(1:end)))
grid on
legend('z','zd','zd#','zdd','zdd#')
title('Posizioni z traiettoria')

figure(6)
%plot(tt,Sr(:),tt,Srd(:),tt(1:end-1),diff(Sr(:))./diff(tt(1:end)),tt,Srdd(:),tt(1:end-1),diff(Srd(:))./diff(tt(1:end)))
plot(tt,Sr(:),tt,Srd(:),tt,Srdd(:))
grid on
%legend('g','gd','gd#','gdd','gdd#')
legend('g','gd','gdd')
title('Posizioni gamma orientamento gripper')




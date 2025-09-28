%% Calcolo tratto iniziale della traiettoria con tempo di attuazione minimo e legge di moto cicloidale
% calcolo cinematica inversa degli estremi

Qi_1 = SCARAinvAnalitica([nodi_trasformati(1:3,1);-pi],L);
Qf_1 = SCARAinvAnalitica([nodi_trasformati(1:3,2);pi],L);

% minimo tempo di attuazione
a_max_robot=[20 20 100 100]'; % accelerazioni massime joints del robot 
% T^2 minimo=deltaS*2*pi/amax
T1=sqrt(max([abs(Qf_1(1)-Qi_1(1)); abs(Qf_1(2)-Qi_1(2)); abs(Qf_1(3)-Qi_1(3)); 0]./a_max_robot*2*pi));

n1=50;
dt1=T1/(n1-1);
tt_1=0:dt1:T1;

for i=1:n1
    t=(i-1)*T1/(n1-1);
    %tt(i)=t;
    [q_j1_1(i), qd_j1_1(i), qdd_j1_1(i)]=cicloidale(t,T1,Qi_1(1),Qf_1(1)-Qi_1(1));
    [q_j2_1(i), qd_j2_1(i), qdd_j2_1(i)]=cicloidale(t,T1,Qi_1(2),Qf_1(2)-Qi_1(2));
    [q_j3_1(i), qd_j3_1(i), qdd_j3_1(i)]=cicloidale(t,T1,Qi_1(3),Qf_1(3)-Qi_1(3));
    [q_j4_1(i), qd_j4_1(i), qdd_j4_1(i)]=cicloidale(t,T1,Qi_1(4),Qf_1(4)-Qi_1(4));
    
    q_1(:,i)=[q_j1_1(i) q_j2_1(i) q_j3_1(i) q_j4_1(i)]';
    Siniziale(:,i)=SCARAdir(q_1(:,i),L);
    
    qd_1(:,i)=[qd_j1_1(i) qd_j2_1(i) qd_j3_1(i) qd_j4_1(i)]';
    Sd(:,i)=SCARAjac(q_1(:,i),L)*qd_1(:,i);
    
    qdd_1(:,i)=[qdd_j1_1(i) qdd_j2_1(i) qdd_j3_1(i) qdd_j4_1(i)]';
    Sdd(:,i)=SCARAjac(q_1(:,i),L)*qdd_1(:,i)+SCARAjacP(q_1(:,i),qd_1(:,i),L)*qd_1(:,i);

end

figure(1)
plot3(Siniziale(1,:),Siniziale(2,:),Siniziale(3,:),'x','Color',[0.9290 0.6940 0.1250])
grid on
hold on

figure
subplot(4,1,1)
plot(tt_1,Siniziale(1,:))
title("Traiettoria tratto iniziale da P1 asse x")
xlabel('Tempo (s)');
ylabel('m')
grid on
subplot(4,1,2)
plot(tt_1,Siniziale(2,:))
title("Traiettoria tratto iniziale da P1 asse y")
xlabel('Tempo (s)');
ylabel('m')
grid on
subplot(4,1,3)
plot(tt_1,Siniziale(3,:))
title("Traiettoria tratto iniziale da P1 asse z")
xlabel('Tempo (s)');
ylabel('m')
grid on
subplot(4,1,4)
plot(tt_1,Siniziale(4,:))
title("Traiettoria tratto iniziale da P1 phi")
xlabel('Tempo (s)');
ylabel('rad')
grid on

figure
subplot(4,1,1)
plot(tt_1,Sd(1,:),tt_1,gradient(Siniziale(1,:),tt_1))
title("Velocità tratto iniziale da P1 asse x")
legend("analitica","numerica",'Location','best')
xlabel('Tempo (s)');
ylabel('(m/s)');
grid on
subplot(4,1,2)
plot(tt_1,Sd(2,:),tt_1,gradient(Siniziale(2,:),tt_1))
title("Velocità tratto iniziale da P1 asse y")
legend("analitica","numerica",'Location','best')
xlabel('Tempo (s)');
ylabel('(m/s)');
grid on
subplot(4,1,3)
plot(tt_1,Sd(3,:),tt_1,gradient(Siniziale(3,:),tt_1))
title("Velocità tratto iniziale da P1 asse z")
legend("analitica","numerica",'Location','best')
xlabel('Tempo (s)');
ylabel('(m/s)');
grid on
subplot(4,1,4)
plot(tt_1,Sd(4,:),tt_1,gradient(Siniziale(4,:),tt_1))
title("Velocità tratto iniziale da P1 phi")
legend("analitica","numerica",'Location','best')
xlabel('Tempo (s)');
ylabel('(rad/s)');
grid on

figure
subplot(4,1,1)
plot(tt_1,Sdd(1,:),tt_1,gradient(Sd(1,:),tt_1))
title("Accelerazione tratto iniziale da P1 asse x")
legend("analitica","numerica",'Location','best')
xlabel('Tempo (s)');
ylabel('(m/s^2)');
grid on
subplot(4,1,2)
plot(tt_1,Sdd(2,:),tt_1,gradient(Sd(2,:),tt_1))
title("Accelerazione tratto iniziale da P1 asse y")
legend("analitica","numerica",'Location','best')
xlabel('Tempo (s)');
ylabel('(m/s^2)');
grid on
subplot(4,1,3)
plot(tt_1,Sdd(3,:),tt_1,gradient(Sd(3,:),tt_1))
title("Accelerazione tratto iniziale da P1 asse z")
legend("analitica","numerica",'Location','best')
xlabel('Tempo (s)');
ylabel('(m/s^2)');
grid on
subplot(4,1,4)
plot(tt_1,Sdd(4,:),tt_1,gradient(Sd(4,:),tt_1))
title("Accelerazione tratto iniziale da P1 phi")
legend("analitica","numerica",'Location','best')
xlabel('Tempo (s)');
ylabel('(rad/s^2)');
grid on

figure
subplot(4,1,1)
plot(tt_1,q_j1_1)
title("Traiettoria tratto iniziale da P1 j1")
ylabel('rad')
xlabel('Tempo (s)');
grid on
subplot(4,1,2)
plot(tt_1,q_j2_1)
title("Traiettoria tratto iniziale da P1 j2")
ylabel('rad')
xlabel('Tempo (s)');
grid on
subplot(4,1,3)
plot(tt_1,q_j3_1)
title("Traiettoria tratto iniziale da P1 j3")
ylabel('m')
xlabel('Tempo (s)');
grid on
subplot(4,1,4)
plot(tt_1,q_j4_1)
title("Traiettoria tratto iniziale da P1 j4")
ylabel('rad')
xlabel('Tempo (s)');
grid on

figure
subplot(4,1,1)
plot(tt_1,qd_j1_1,tt_1,gradient(q_1(1,:),tt_1))
title("Velocità tratto iniziale da P1 j1")
legend("analitica","numerica",'Location','best')
xlabel('Tempo (s)');
ylabel('rad/s')
grid on
subplot(4,1,2)
plot(tt_1,qd_j2_1,tt_1,gradient(q_1(2,:),tt_1))
title("Velocità tratto iniziale da P1 j2")
legend("analitica","numerica",'Location','best')
xlabel('Tempo (s)');
ylabel('rad/s')
grid on
subplot(4,1,3)
plot(tt_1,qd_j3_1,tt_1,gradient(q_1(3,:),tt_1))
title("Velocità tratto iniziale da P1 j3")
legend("analitica","numerica",'Location','best')
xlabel('Tempo (s)');
ylabel('m/s')
grid on
subplot(4,1,4)
plot(tt_1,qd_j4_1,tt_1,gradient(q_1(4,:),tt_1))
title("Velocità tratto iniziale da P1 j4")
legend("analitica","numerica",'Location','best')
xlabel('Tempo (s)');
ylabel('rad/s')
grid on

figure
subplot(4,1,1)
plot(tt_1,qdd_j1_1,tt_1,gradient(qd_1(1,:),tt_1))
title("Accelerazione tratto iniziale da P1 j1")
legend("analitica","numerica",'Location','best')
xlabel('Tempo (s)');
ylabel('rad/s^2')
grid on
subplot(4,1,2)
plot(tt_1,qdd_j2_1,tt_1,gradient(qd_1(2,:),tt_1))
title("Accelerazione tratto iniziale da P1 j2")
legend("analitica","numerica",'Location','best')
xlabel('Tempo (s)');
ylabel('rad/s^2')
grid on
subplot(4,1,3)
plot(tt_1,qdd_j3_1,tt_1,gradient(qd_1(3,:),tt_1))
title("Accelerazione tratto iniziale da P1 j3")
legend("analitica","numerica",'Location','best')
xlabel('Tempo (s)');
ylabel('m/s^2')
grid on
subplot(4,1,4)
plot(tt_1,qdd_j4_1,tt_1,gradient(qd_1(4,:),tt_1))
title("Accelerazione tratto iniziale da P1 j4")
legend("analitica","numerica",'Location','best')
xlabel('Tempo (s)');
ylabel('rad/s^2')
grid on

close(2:7)

% exportgraphics(figure(2),"traiettoria_w_iniziale.eps",'ContentType', 'vector')
% exportgraphics(figure(3),"velocita_w_iniziale.eps",'ContentType', 'vector')
% exportgraphics(figure(4),"accelerazione_w_iniziale.eps",'ContentType', 'vector')
% exportgraphics(figure(5),"traiettoria_j_iniziale.eps",'ContentType', 'vector')
% exportgraphics(figure(6),"velocita_j_iniziale.eps",'ContentType', 'vector')
% exportgraphics(figure(7),"accelerazione_j_iniziale.eps",'ContentType', 'vector')
%% Calcolo tratto finale della traiettoria con tempo di attuazione minimo e legge di moto cicloidale
% calcolo cinematica inversa degli estremi
Qi_2 = SCARAinvAnalitica([nodi_trasformati(1:3,end-1);phi(end)],L);
% Qi_2(4)=q_j4;
% Qi_2=SCARAinvAnalitica([x_t_sp(end); y_t_sp(end); z_t_sp(end); 1], L);
Qf_2 = SCARAinvAnalitica([nodi_trasformati(1:3,end);0],L);

% minimo tempo di attuazione
% T^2 minimo=deltaS*2*pi/amax
T2=sqrt(max([abs(Qf_2(1)-Qi_2(1)); abs(Qf_2(2)-Qi_2(2)); abs(Qf_2(3)-Qi_2(3)); 0]./a_max_robot*2*pi));


n2=50;
dt2=T2/(n2-1);
tt_2=0:dt2:T2;

for i=1:n2
    tempo=(i-1)*T2/(n2-1);
    %tt(i)=tempo;
    [q_j1_2(i), qd_j1_2(i), qdd_j1_2(i)]=cicloidale(tempo,T2,Qi_2(1),Qf_2(1)-Qi_2(1));
    [q_j2_2(i), qd_j2_2(i), qdd_j2_2(i)]=cicloidale(tempo,T2,Qi_2(2),Qf_2(2)-Qi_2(2));
    [q_j3_2(i), qd_j3_2(i), qdd_j3_2(i)]=cicloidale(tempo,T2,Qi_2(3),Qf_2(3)-Qi_2(3));
    [q_j4_2(i), qd_j4_2(i), qdd_j4_2(i)]=cicloidale(tempo,T2,Qi_2(4),Qf_2(4)-Qi_2(4));

    q_2(:,i)=[q_j1_2(i) q_j2_2(i) q_j3_2(i) q_j4_2(i)]';
    Sfinale(:,i)=SCARAdir(q_2(:,i),L);
    
    qd_2(:,i)=[qd_j1_2(i) qd_j2_2(i) qd_j3_2(i) qd_j4_2(i)]';
    Sd(:,i)=SCARAjac(q_2(:,i),L)*qd_2(:,i);
    
    qdd_2(:,i)=[qdd_j1_2(i) qdd_j2_2(i) qdd_j3_2(i) qdd_j4_2(i)]';
    Sdd(:,i)=SCARAjac(q_2(:,i),L)*qdd_2(:,i)+SCARAjacP(q_2(:,i),qd_2(:,i),L)*qd_2(:,i);
    
end

figure(1)
plot3(Sfinale(1,:),Sfinale(2,:),Sfinale(3,:),'x','Color',[0.4940 0.1840 0.5560])
legend("Tratto iniziale","tratto centrale","nodi","","","tratto finale",'Location','best')
hold on

figure
subplot(4,1,1)
plot(tt_2,Sfinale(1,:))
title("Traiettoria tratto finale a P1 asse x")
xlabel('Tempo (s)');
ylabel('m')
grid on
subplot(4,1,2)
plot(tt_2,Sfinale(2,:))
title("Traiettoria tratto finale a P1 asse y")
xlabel('Tempo (s)');
ylabel('m')
grid on
subplot(4,1,3)
plot(tt_2,Sfinale(3,:))
title("Traiettoria tratto finale a P1 asse z")
xlabel('Tempo (s)');
ylabel('m')
grid on
subplot(4,1,4)
plot(tt_2,Sfinale(4,:))
title("Traiettoria tratto finale a P1 phi")
xlabel('Tempo (s)');
ylabel('rad')
grid on

figure
subplot(4,1,1)
plot(tt_2,Sd(1,:),tt_2,gradient(Sfinale(1,:),tt_2))
title("Velocità tratto finale a P1 asse x")
legend("analitica","numerica",'Location','best')
xlabel('Tempo (s)');
ylabel('(m/s)');
grid on
subplot(4,1,2)
plot(tt_2,Sd(2,:),tt_2,gradient(Sfinale(2,:),tt_2))
title("Velocità tratto finale a P1 asse y")
legend("analitica","numerica",'Location','best')
xlabel('Tempo (s)');
ylabel('(m/s)');
grid on
subplot(4,1,3)
plot(tt_2,Sd(3,:),tt_2,gradient(Sfinale(3,:),tt_2))
title("Velocità tratto finale a P1 asse z")
legend("analitica","numerica",'Location','best')
xlabel('Tempo (s)');
ylabel('(m/s)');
grid on
subplot(4,1,4)
plot(tt_2,Sd(4,:),tt_2,gradient(Sfinale(4,:),tt_2))
title("Velocità tratto finale a P1 phi")
legend("analitica","numerica",'Location','best')
xlabel('Tempo (s)');
ylabel('(rad/s)');
grid on

figure
subplot(4,1,1)
plot(tt_2,Sdd(1,:),tt_2,gradient(Sd(1,:),tt_2))
title("Accelerazione tratto finale a P1 asse x")
legend("analitica","numerica",'Location','best')
xlabel('Tempo (s)');
ylabel('(m/s^2)');
grid on
subplot(4,1,2)
plot(tt_2,Sdd(2,:),tt_2,gradient(Sd(2,:),tt_2))
title("Accelerazione tratto finale a P1 asse y")
legend("analitica","numerica",'Location','best')
xlabel('Tempo (s)');
ylabel('(m/s^2)');
grid on
subplot(4,1,3)
plot(tt_2,Sdd(3,:),tt_2,gradient(Sd(3,:),tt_2))
title("Accelerazione tratto finale a P1 asse z")
legend("analitica","numerica",'Location','best')
xlabel('Tempo (s)');
ylabel('(m/s^2)');
grid on
subplot(4,1,4)
plot(tt_2,Sdd(4,:),tt_2,gradient(Sd(4,:),tt_2))
title("Accelerazione tratto finale a P1 phi")
legend("analitica","numerica",'Location','best')
xlabel('Tempo (s)');
ylabel('(rad/s^2)');
grid on

figure
subplot(4,1,1)
plot(tt_2,q_j1_2)
title("Traiettoria tratto finale a P1 j1")
ylabel('rad')
xlabel('Tempo (s)');
grid on
subplot(4,1,2)
plot(tt_2,q_j2_2)
title("Traiettoria tratto finale a P1 j2")
ylabel('rad')
xlabel('Tempo (s)');
grid on
subplot(4,1,3)
plot(tt_2,q_j3_2)
title("Traiettoria tratto finale a P1 j3")
ylabel('m')
xlabel('Tempo (s)');
grid on
subplot(4,1,4)
plot(tt_2,q_j4_2)
title("Traiettoria tratto finale a P1 j4")
ylabel('rad')
xlabel('Tempo (s)');
grid on

figure
subplot(4,1,1)
plot(tt_2,qd_j1_2,tt_2,gradient(q_2(1,:),tt_2))
title("Velocità tratto finale a P1 j1")
legend("analitica","numerica",'Location','best')
xlabel('Tempo (s)');
ylabel('rad/s')
grid on
subplot(4,1,2)
plot(tt_2,qd_j2_2,tt_2,gradient(q_2(2,:),tt_2))
title("Velocità tratto finale a P1 j2")
legend("analitica","numerica",'Location','best')
xlabel('Tempo (s)');
ylabel('rad/s')
grid on
subplot(4,1,3)
plot(tt_2,qd_j3_2,tt_2,gradient(q_2(3,:),tt_2))
title("Velocità tratto finale a P1 j3")
legend("analitica","numerica",'Location','best')
xlabel('Tempo (s)');
ylabel('m/s')
grid on
subplot(4,1,4)
plot(tt_2,qd_j4_2,tt_2,gradient(q_2(4,:),tt_2))
title("Velocità tratto finale a P1 j4")
legend("analitica","numerica",'Location','best')
xlabel('Tempo (s)');
ylabel('rad/s')
grid on

figure
subplot(4,1,1)
plot(tt_2,qdd_j1_2,tt_2,gradient(qd_2(1,:),tt_2))
title("Accelerazione tratto finale a P1 j1")
legend("analitica","numerica",'Location','best')
xlabel('Tempo (s)');
ylabel('rad/s^2')
grid on
subplot(4,1,2)
plot(tt_2,qdd_j2_2,tt_2,gradient(qd_2(2,:),tt_2))
title("Accelerazione tratto finale a P1 j2")
legend("analitica","numerica",'Location','best')
xlabel('Tempo (s)');
ylabel('rad/s^2')
grid on
subplot(4,1,3)
plot(tt_2,qdd_j3_2,tt_2,gradient(qd_2(3,:),tt_2))
title("Accelerazione tratto finale a P1 j3")
legend("analitica","numerica",'Location','best')
xlabel('Tempo (s)');
ylabel('m/s^2')
grid on
subplot(4,1,4)
plot(tt_2,qdd_j4_2,tt_2,gradient(qd_2(4,:),tt_2))
title("Accelerazione tratto finale a P1 j4")
legend("analitica","numerica",'Location','best')
xlabel('Tempo (s)');
ylabel('rad/s^2')
grid on


clear tempo
close(6:11)

% exportgraphics(figure(2),"traiettoria_w_finale.eps",'ContentType', 'vector')
% exportgraphics(figure(3),"velocita_w_finale.eps",'ContentType', 'vector')
% exportgraphics(figure(4),"accelerazione_w_finale.eps",'ContentType', 'vector')
% exportgraphics(figure(5),"traiettoria_j_finale.eps",'ContentType', 'vector')
% exportgraphics(figure(6),"velocita_j_finale.eps",'ContentType', 'vector')
% exportgraphics(figure(7),"accelerazione_j_finale.eps",'ContentType', 'vector')
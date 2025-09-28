clear 
close all
clc

geometria_SCARA

creazione_nodi

tratto_iniziale

%% Calcolo parte centrale della traiettoria  
% Calcolo traiettoria spline senza primo e ultimo punto
t=linspace(0,1,length(nodi_trasformati)-2); % tempo normalizzato tra 0 e 1
[x,y,z,t_trat]=interpolazione_spline(nodi_trasformati(:,1:end),t,1000);

% calcolo lunghezza dei singoli tratti 
for i=1:length(t_trat)-1
    LL(i)=distanza3([x(i),y(i),z(i)],[x(i+1),y(i+1),z(i+1)]);
end
Scumulata=cumsum([0,LL(1:end)]);
LL=[LL 0];


% Plot della curva interpolata e dei punti originali
figure(1)
plot3(x, y, z, 'x-','Color',[0 0.4470 0.7410], 'LineWidth', 2);
hold on;
plot3(nodi_trasformati(1,:), nodi_trasformati(2,:), nodi_trasformati(3,:), 'ro', 'MarkerSize', 8, 'MarkerFaceColor', '[0.8500 0.3250 0.0980]');
xlabel('X'); ylabel('Y'); zlabel('Z');
title('Traiettoria completa');
grid on;
axis equal;


%% Look ahead traiettoria

v_max=[1, 0.5, 0.7, 0.8, 0.5];
a_max=[4, 0.4, 1, 3.5, 2];
d_max=[3, 0.5, 1, 2, 2.5];
v_end=[0.2, 0.3, 0.3, 0.2, 0];
v_in=[0, v_end(1), v_end(2), v_end(3), v_end(4)];
s_finale=[200, 400, 600, 800, length(Scumulata)];

% calcolo delle grandezze cinematiche
[v_s1,v_t1,s1,t1,v_lim1,a_s1,a_t1]=look_ahead(Scumulata(1:s_finale(1)),v_max(1),a_max(1),d_max(1),v_in(1),v_end(1),0);
[v_s2,v_t2,s2,t2,v_lim2,a_s2,a_t2]=look_ahead(Scumulata(s_finale(1):s_finale(2)),v_max(2),a_max(2),d_max(2),v_in(2),v_end(2),Scumulata(s_finale(1))-s1(end));
[v_s3,v_t3,s3,t3,v_lim3,a_s3,a_t3]=look_ahead(Scumulata(s_finale(2):s_finale(3)),v_max(3),a_max(3),d_max(3),v_in(3),v_end(3),Scumulata(s_finale(2))-s2(end));
[v_s4,v_t4,s4,t4,v_lim4,a_s4,a_t4]=look_ahead(Scumulata(s_finale(3):s_finale(4)),v_max(4),a_max(4),d_max(4),v_in(4),v_end(4),Scumulata(s_finale(3))-s3(end));
[v_s5,v_t5,s5,t5,v_lim5,a_s5,a_t5]=look_ahead(Scumulata(s_finale(4):s_finale(5)),v_max(5),a_max(5),d_max(5),v_in(5),v_end(5),Scumulata(s_finale(4))-s4(end));
% concateno i tratti 
v_s=[v_s1(1:end-1) v_s2(1:end-1) v_s3(1:end-1) v_s4(1:end-1) v_s5(1:end)];
v_t=[v_t1(1:end-1) v_t2(1:end-1) v_t3(1:end-1) v_t4(1:end-1) v_t5(1:end)];
s=[s1(1:end-1) s2(1:end-1) s3(1:end-1) s4(1:end-1) s5(1:end)];
t=[t1(1:end-1) t1(end)+t2(1:end-1) t1(end)+t2(end)+t3(1:end-1) t1(end)+t2(end)+t3(end)+t4(1:end-1) t1(end)+t2(end)+t3(end)+t4(end)+t5(1:end)];
v_lim=[v_lim1(1:end-1) v_lim2(1:end-1) v_lim3(1:end-1) v_lim4(1:end-1) v_lim5(1:end)];
a_s=[a_s1(1:end-1) a_s2(1:end-1) a_s3(1:end-1) a_s4(1:end-1) a_s5(1:end)];
a_t=[a_t1(1:end-1) a_t2(1:end-1) a_t3(1:end-1) a_t4(1:end-1) a_t5(1:end)]; 

% Creazione dei grafici
figure;

% Velocità vs Spazio
subplot(2,2,1);
plot(Scumulata,v_lim,Scumulata, v_s,'b', 'LineWidth', 2); 
xlabel('Spazio percorso (m)');
ylabel('Velocità (m/s)');
title('Profilo di velocità in funzione dello spazio');
grid on;


v_tn=gradient(s,t);

% Velocità vs Tempo
subplot(2,2,2);
plot(t,v_lim,t,v_tn,t, v_t, 'r', 'LineWidth', 2);  
xlabel('Tempo (s)');
ylabel('Velocità (m/s)');
title('Profilo di velocità in funzione del tempo');
grid on;

a_tn=gradient(v_t,t);

% Accelerazione vs Tempo
subplot(2,2,4);
plot(t,a_tn,t, a_t, 'r', 'LineWidth', 2);
xlabel('Tempo (s)');
ylabel('Accelerazione (m/s^2)');
title('Profilo di accelerazione in funzione del tempo');
grid on;


% Spazio vs Tempo
subplot(2,2,3);
plot(t,s,'r', 'LineWidth', 2); 
xlabel('Tempo (s)');
ylabel('Spazio (m)');
title('Spostamento in funzione del tempo');
grid on;

%% Proietto la velocità del tratto centrale lungo gli assi 

calcolo_grandezze_cinematiche


figure
plot(Tcumulata,x_t_sp,Tcumulata,y_t_sp,Tcumulata,z_t_sp)
title("Posizione gripper working space")
legend("x","y","z",'Location','Best')
xlabel("t [s]")
ylabel("[m]")
grid on
figure(1)
plot3(x_t_sp, y_t_sp, z_t_sp, 'b-', 'LineWidth', 1);
xlabel('X'); ylabel('Y'); zlabel('Z');
grid on


figure
subplot(3,1,1)
plot(Tcumulata,v(1,:),'LineWidth',1.5)
hold on
plot(Tcumulata,gradient(x_t_sp,Tcumulata),'r',[Tcumulata(1) Tcumulata(end)],[0 0],'k')
legend("v proiettata","v numerica",'Location','best')
title("Proiezione velocità asse x")
xlabel("t [s]")
ylabel("v [m/s]")
grid on
subplot(3,1,2)
plot(Tcumulata,v(2,:),'LineWidth',1.5)
hold on
plot(Tcumulata,gradient(y_t_sp,Tcumulata),'r',[Tcumulata(1) Tcumulata(end)],[0 0],'k')
legend("v proiettata","v numerica",'Location','best')
title("Proiezione velocità asse y")
xlabel("t [s]")
ylabel("v [m/s]")
grid on
subplot(3,1,3)
plot(Tcumulata,v(3,:),'LineWidth',1.5)
hold on
plot(Tcumulata,gradient(z_t_sp,Tcumulata),'r',[Tcumulata(1) Tcumulata(end)],[0 0],'k')
legend("v proiettata","v numerica",'Location','best')
title("Proiezione velocità asse z")
xlabel("t [s]") 
ylabel("v [m/s]")
grid on

figure
subplot(3,1,1)
plot(Tcumulata,a(1,:),'b','LineWidth',1.5)
hold on
plot(Tcumulata,gradient(v(1,:),Tcumulata),'r',[Tcumulata(1) Tcumulata(end)],[0 0],'k')
legend("a tang proiettata","a numerica",'Location','best')
grid on
title("Proiezione accelerazione asse x")
xlabel("t [s]")
ylabel("a [m/s^2]")
subplot(3,1,2)
plot(Tcumulata,a(2,:),'b','LineWidth',1.5)
hold on
plot(Tcumulata,gradient(v(2,:),Tcumulata),'r',[Tcumulata(1) Tcumulata(end)],[0 0],'k')
legend("a tang proiettata","a numerica",'Location','best')
grid on
title("Proiezione accelerazione asse y")
xlabel("t [s]") 
ylabel("a [m/s^2]")
subplot(3,1,3)
plot(Tcumulata,a(3,:),'b','LineWidth',1.5)
hold on
plot(Tcumulata,gradient(v(3,:),Tcumulata),'r',[Tcumulata(1) Tcumulata(end)],[0 0],'k')
legend("a tang proiettata","a numerica",'Location','best')
grid on
title("Proiezione accelerazione asse z")
xlabel("t [s]")
ylabel("a [m/s^2]")

%% Calcolo grandezze cinematiche quarto giunto nel tratto centrale

j4_tratto_centrale

%% calcolo tratto di rientro

tratto_finale

%% Calcolo la cinematica inversa della traiettoria 
% cinematica inversa del tratto centrale
QQ=[]; % posizione dei giunti del robot
QQd=[]; % velocità ai giunti del robot
QQdd=[]; % accelerazione ai giunti del robot

for i=1:length(Scumulata)
    QQ(:,i)=SCARAinvAnalitica([x_t_sp(i);y_t_sp(i);z_t_sp(i);phi(i)],L);
    J=SCARAjac(QQ(:,i),L);
    QQd(:,i)=J^(-1)*[v(:,i);phid(i)];
    Jp=SCARAjacP(QQ(:,i),QQd(:,i),L);
    QQdd(:,i)=J^(-1)*([a(:,i);phidd(i)]-Jp*QQd(:,i));
end

% plot grandezze cinematiche joint space area centrale
figure
subplot(4,1,1)
plot(Tcumulata,QQ(1,:),[Tcumulata(1) Tcumulata(end)],[0 0],'k')
grid on
title("Posizione joint 1 tratto centrale")
xlabel("t [s]")
ylabel("[rad]")
subplot(4,1,2)
plot(Tcumulata,QQ(2,:),[Tcumulata(1) Tcumulata(end)],[0 0],'k')
grid on
title("Posizione joint 2 tratto centrale")
xlabel("t [s]")
ylabel("[rad]")
subplot(4,1,3)
plot(Tcumulata,QQ(3,:),[Tcumulata(1) Tcumulata(end)],[0 0],'k')
grid on
title("Posizione joint 3 tratto centrale")
xlabel("t [s]")
ylabel("[m]")
subplot(4,1,4)
plot(Tcumulata,QQ(4,:),[Tcumulata(1) Tcumulata(end)],[0 0],'k')
grid on
title("Posizione joint 4 tratto centrale")
xlabel("t [s]")
ylabel("[rad]")


QQdn(1,:)=gradient(QQ(1,:),Tcumulata);
QQdn(2,:)=gradient(QQ(2,:),Tcumulata);
QQdn(3,:)=gradient(QQ(3,:),Tcumulata);
QQdn(4,:)=gradient(QQ(4,:),Tcumulata);

figure
subplot(4,1,1)
plot(Tcumulata,QQd(1,:),Tcumulata,QQdn(1,:),[Tcumulata(1) Tcumulata(end)],[0 0],'k')
legend("v","vn",'Location','Best')
grid on
title("Velocità joint 1 tratto centrale")
xlabel("t [s]")
ylabel("v [rad/s]")
subplot(4,1,2)
plot(Tcumulata,QQd(2,:),Tcumulata,QQdn(2,:),[Tcumulata(1) Tcumulata(end)],[0 0],'k')
legend("v","vn",'Location','Best')
grid on
title("Velocità joint 2 tratto centrale")
xlabel("t [s]")
ylabel("v [rad/s]")
subplot(4,1,3)
plot(Tcumulata,QQd(3,:),Tcumulata,QQdn(3,:),[Tcumulata(1) Tcumulata(end)],[0 0],'k')
legend("v","vn",'Location','Best')
grid on
title("Velocità joint 3 tratto centrale")
xlabel("t [s]")
ylabel("v [m/s]")
subplot(4,1,4)
plot(Tcumulata,QQd(4,:),Tcumulata,QQdn(4,:),[Tcumulata(1) Tcumulata(end)],[0 0],'k')
legend("v","vn",'Location','Best')
grid on
title("Velocità joint 4 tratto centrale")
xlabel("t [s]")
ylabel("v [rad/s]")


QQddn(1,:)=gradient(QQdn(1,:),Tcumulata);
QQddn(2,:)=gradient(QQdn(2,:),Tcumulata);
QQddn(3,:)=gradient(QQdn(3,:),Tcumulata);
QQddn(4,:)=gradient(QQdn(4,:),Tcumulata);

figure
subplot(4,1,1)
plot(Tcumulata,QQdd(1,:),Tcumulata,QQddn(1,:),[Tcumulata(1) Tcumulata(end)],[0 0],'k')
legend("a","an",'Location','Best')
grid on
title("Accelerazioni joint 1 tratto centrale")
xlabel("t [s]")
ylabel("a [rad/s^2]")
subplot(4,1,2)
plot(Tcumulata,QQdd(2,:),Tcumulata,QQddn(2,:),[Tcumulata(1) Tcumulata(end)],[0 0],'k')
legend("a","an",'Location','Best')
grid on
title("Accelerazioni joint 2 tratto centrale")
xlabel("t [s]")
ylabel("a [rad/s^2]")
subplot(4,1,3)
plot(Tcumulata,QQdd(3,:),Tcumulata,QQddn(3,:),[Tcumulata(1) Tcumulata(end)],[0 0],'k')
legend("a","an",'Location','Best')
grid on
title("Accelerazioni joint 3 tratto centrale")
xlabel("t [s]")
ylabel("a [m/s^2]")
subplot(4,1,4)
plot(Tcumulata,QQdd(4,:),Tcumulata,QQddn(4,:),[Tcumulata(1) Tcumulata(end)],[0 0],'k')
legend("a","an",'Location','Best')
grid on
title("Accelerazioni joint 4 tratto centrale")
xlabel("t [s]")
ylabel("a [rad/s^2]")



%% plot di tutta la traiettoria completa nello spazio di lavoro

for i=1:n_j4
    S_j4(:,i)=SCARAdir([QQ(1,i); QQ(2,i); QQ(3,i); QQ(4,i)],L);
end

figure
subplot(4,1,1)
plot(tt_1,Siniziale(1,:),tt_1(end)+Tcumulata,x_t_sp,tt_1(end)+Tcumulata(end)+tt_2,Sfinale(1,:))
title("Traiettoria completa asse x")
legend("tratto iniziale","tratto centrale","tratto finale",'Location','best')
ylabel("[m]")
xlabel("t [s]")
grid on
subplot(4,1,2)
plot(tt_1,Siniziale(2,:),tt_1(end)+Tcumulata,y_t_sp,tt_1(end)+Tcumulata(end)+tt_2,Sfinale(2,:))
title("Traiettoria completa asse y")
legend("tratto iniziale","tratto centrale","tratto finale",'Location','best')
ylabel("[m]")
xlabel("t [s]")
grid on
subplot(4,1,3)
plot(tt_1,Siniziale(3,:),tt_1(end)+Tcumulata,z_t_sp,tt_1(end)+Tcumulata(end)+tt_2,Sfinale(3,:))
title("Traiettoria completa asse z")
legend("tratto iniziale","tratto centrale","tratto finale",'Location','best')
ylabel("[m]")
xlabel("t [s]")
grid on
subplot(4,1,4)
plot(tt_1,Siniziale(4,:),tt_1(end)+Tcumulata,S_j4(4,:),tt_1(end)+Tcumulata(end)+tt_2,Sfinale(4,:))
title("Traiettoria completa giunto 4")
legend("tratto iniziale","tratto centrale","tratto finale",'Location','best')
xlabel("t [s]")
ylabel("[rad]")
ylim([-4,4])
grid on


%% Coppie e forze ai giunti

Q_j=[q_j1_1 QQ(1,2:end-1) q_j1_2;
     q_j2_1 QQ(2,2:end-1) q_j2_2;
     q_j3_1 QQ(3,2:end-1) q_j3_2;
     q_j4_1 QQ(4,2:end-1) q_j4_2];

Qd_j=[qd_j1_1 QQd(1,2:end-1) qd_j1_2;
      qd_j2_1 QQd(2,2:end-1) qd_j2_2;
      qd_j3_1 QQd(3,2:end-1) qd_j3_2;
      qd_j4_1 QQd(4,2:end-1) qd_j4_2];

Qdd_j=[qdd_j1_1 QQdd(1,2:end-1) qdd_j1_2;
       qdd_j2_1 QQdd(2,2:end-1) qdd_j2_2;
       qdd_j3_1 QQdd(3,2:end-1) qdd_j3_2;
       qdd_j4_1 QQdd(4,2:end-1) qdd_j4_2];

simtime=[tt_1 tt_1(end)+Tcumulata(2:end-1) tt_1(end)+Tcumulata(end)+tt_2];

for i=1:length(Siniziale(1,:))+length(Scumulata)-2+length(Sfinale(1,:))
    Je=SCARAJacEsteso(Q_j(:,i),L,G);
    Sep(:,i)=Je*Qd_j(:,i);
    Jep=SCARAJacpEsteso(Q_j(:,i),Qd_j(:,i),L,G);
    FFq(:,i)=(Je'*M*Je)*Qdd_j(:,i)+(Je'*M*Jep)*Qd_j(:,i)-Je'*Fse+Je'*M*Ag;
end

figure
plot(simtime,FFq)
%plot(tt_1,FFq(:,1:length(tt_1)))
title("Coppie motori nel piano verticale")
legend("alpha","beta","s","gamma")
grid on

% debug delle coppie e forze ai giunti con energia e potenza

Se0=SCARAdirEstesa(Q_j(:,1),L,G);
Ep0=Se0'*M*Ag; % Energia potenziale all'inizio del moto per il calcolo dell'energia potenziale relativa

for i=1:length(Siniziale(1,:))+length(Scumulata)-2+length(Sfinale(1,:))
    Je=SCARAJacEsteso(Q_j(:,i),L,G);
    Ek(i)=0.5*Qd_j(:,i)'*Je'*M*Je*Qd_j(:,i); 

    Se=SCARAdirEstesa(Q_j(:,i),L,G);
    Ep(i)=Se'*M*Ag-Ep0;
end

Et=Ek+Ep;

figure
plot(simtime,Ek,simtime,Ep,simtime,Et,[simtime(1) simtime(end)],[0 0],'k')
%plot(tt_1,Ek(1:length(tt_1)),tt_1,Ep(1:length(tt_1)),tt_1,Ek(1:length(tt_1))+Ep(1:length(tt_1)),[tt_1(1) tt_1(end)],[0 0],'k')
ylim([-20 180])
title("Energia piano verticale")
legend("En cinetica","En potenziale","En totale","Location","best")
grid on

% calcolo della potenza W=FFq'QQp+Fse'Sep
for i=1:length(Siniziale(1,:))+length(Scumulata)-2+length(Sfinale(1,:))
    W(i)=FFq(:,i)'*Qd_j(:,i)+Fse'*Sep(:,i);
end

dEt=gradient(Et,simtime);
% dEt=[dEt(1:length(tt_1)-1)/dt1 dEt(length(tt_1):(length(tt_1)+length(t)-1))/(t(2)-t(1)) dEt(length(tt_1)+length(t):end)/dt2];

figure
plot(simtime,W,simtime,dEt)
legend("Potenza","der En tot")
title("Potenza piano verticale")
grid on


%% Esporto figure 

% exportgraphics(figure(1),"traiettoria_completa.eps",'ContentType', 'vector')


clear 
close all
clc

% definisco l'ascissa curvilinea come la somma di tutti i tratti della
% traiettoria definita per punti [x;y;z;0]

P=[-2,3,0,1;
    -1,3,0,1;
    0,3,0,1;
    0.5,3.2,0,1;
    0,3.8,0,1;
   -1,4,0,1;
   -1,2,0,1;
   -1,0,0,1;
   -0.4,-1,0,1
   1,0,0,1]';


figure
for i=1:length(P)
    plot(P(1,i),P(2,i),'xb')
    hold on
end
grid on
title("Traiettoria vista ortogonale")

% Trasformo i punti nello spazio

scala=[0.1 0.1 0.1 1]';

mtraj_1=[cos(pi/4) -sin(pi/4) 0 0;
         sin(pi/4) cos(pi/4) 0 0;
         0 0 1 0;
         0 0 0 1];
       
mtraj_2=[1 0 0 0;
         0 cos(pi/6) -sin(pi/6) 0;
         0 sin(pi/6) cos(pi/6) 0;
         0 0 0 1];

mtraj_3=[1 0 0 4;
         0 1 0 4;
         0 0 1 -1.5;
         0 0 0 1];
        
mtraj=mtraj_1*mtraj_2*mtraj_3;

for i=1:length(P)
    nodi_trasformati(:,i)=mtraj*P(:,i).*scala; 
end

% Aggiungo punto P1 inizio e fine della traiettoria 
P1=[0.1 0.3 0.15 1]';
nodi_trasformati=[P1 nodi_trasformati P1];

figure
for i=1:length(nodi_trasformati)
    plot3(nodi_trasformati(1,i),nodi_trasformati(2,i),nodi_trasformati(3,i),'xb')
    hold on
end
grid on
title("Traiettoria nello spazio")
close all




%% Calcolo tratto iniziale della traiettoria con tempo di attuazione minimo e legge di moto cicloidale
% calcolo cinematica inversa degli estremi

Qi_1 = SCARAinvAnalitica(nodi_trasformati(:,1),[0.3 0.55 0.55 0.35]');
Qf_1 = SCARAinvAnalitica(nodi_trasformati(:,2),[0.3 0.55 0.55 0.35]');

% minimo tempo di attuazione
a_max_robot=[20 20 100 100]'; % accelerazioni massime joints del robot 
% T^2 minimo=deltaS*2*pi/amax
T1=sqrt(max([abs(Qf_1(1)-Qi_1(1)); abs(Qf_1(2)-Qi_1(2)); abs(Qf_1(3)-Qi_1(3)); 0]./a_max_robot*2*pi));

n1=50;
dt1=T1/(n1-1);
tt_1=0:dt1:T1;

for i=1:n1
    t=(i-1)*T1/(n1-1);
    tt(i)=t;
    [q_j1_1(i), qd_j1_1(i), qdd_j1_1(i)]=cicloidale(t,T1,Qi_1(1),Qf_1(1)-Qi_1(1));
    [q_j2_1(i), qd_j2_1(i), qdd_j2_1(i)]=cicloidale(t,T1,Qi_1(2),Qf_1(2)-Qi_1(2));
    [q_j3_1(i), qd_j3_1(i), qdd_j3_1(i)]=cicloidale(t,T1,Qi_1(3),Qf_1(3)-Qi_1(3));
    [q_j4_1(i), qd_j4_1(i), qdd_j4_1(i)]=cicloidale(t,T1,Qi_1(4),Qf_1(4)-Qi_1(4));
    
end

% plotto la traiettoria nel working space
for i=1:n1
    Siniziale(:,i)=SCARAdir([q_j1_1(i); q_j2_1(i); q_j3_1(i); q_j4_1(i)],[0.3 0.55 0.55 0.35]');
end
figure
plot3(Siniziale(1,:),Siniziale(2,:),Siniziale(3,:),'xr')
hold on

figure(2)
subplot(3,1,1)
plot(tt_1,Siniziale(1,:))
title("Traiettoria di partenza da P1 asse x")
grid on
subplot(3,1,2)
plot(tt_1,Siniziale(2,:))
title("Traiettoria di partenza da P1 asse y")
grid on
subplot(3,1,3)
plot(tt_1,Siniziale(3,:))
title("Traiettoria di partenza da P1 asse z")
grid on

%% Calcolo tratto finale della traiettoria con tempo di attuazione minimo e legge di moto cicloidale

Qi_2 = SCARAinvAnalitica(nodi_trasformati(:,end-1),[0.3 0.55 0.55 0.35]');
Qf_2 = SCARAinvAnalitica(nodi_trasformati(:,end),[0.3 0.55 0.55 0.35]');

% minimo tempo di attuazione
% T^2 minimo=deltaS*2*pi/amax
T2=sqrt(max([abs(Qf_2(1)-Qi_2(1)); abs(Qf_2(2)-Qi_2(2)); abs(Qf_2(3)-Qi_2(3)); 0]./a_max_robot*2*pi));


n2=50;
dt2=T2/(n2-1);
tt_2=0:dt2:T2;

for i=1:n2
    t=(i-1)*T2/(n2-1);
    tt(i)=t;
    [q_j1_2(i), qd_j1_2(i), qdd_j1_2(i)]=cicloidale(t,T2,Qi_2(1),Qf_2(1)-Qi_2(1));
    [q_j2_2(i), qd_j2_2(i), qdd_j2_2(i)]=cicloidale(t,T2,Qi_2(2),Qf_2(2)-Qi_2(2));
    [q_j3_2(i), qd_j3_2(i), qdd_j3_2(i)]=cicloidale(t,T2,Qi_2(3),Qf_2(3)-Qi_2(3));
    [q_j4_2(i), qd_j4_2(i), qdd_j4_2(i)]=cicloidale(t,T2,Qi_2(4),Qf_2(4)-Qi_2(4));
    
end

% plotto la traiettoria nel working space
for i=1:n2
    Sfinale(:,i)=SCARAdir([q_j1_2(i); q_j2_2(i); q_j3_2(i); q_j4_2(i)],[0.3 0.55 0.55 0.35]');
end

figure(1)
plot3(Sfinale(1,:),Sfinale(2,:),Sfinale(3,:),'xr')
hold on

figure(3)
subplot(3,1,1)
plot(tt_2,Sfinale(1,:))
title("Traiettoria di rientro in P1 asse x")
grid on
subplot(3,1,2)
plot(tt_2,Sfinale(2,:))
title("Traiettoria di rientro in P1 asse y")
grid on
subplot(3,1,3)
plot(tt_2,Sfinale(3,:))
title("Traiettoria di rientro in P1 asse z")
grid on

%% Calcolo parte centrale della traiettoria  
% Calcolo traiettoria spline senza primo e ultimo punto
t=linspace(0,1,length(nodi_trasformati)-2); % tempo normalizzato tra 0 e 1
[x,y,z,t_trat]=interpolazione_spline(nodi_trasformati(:,1:end),t,1000);

% calcolo lunghezza dei singoli tratti 
for i=1:length(t_trat)-1
    L(i)=distanza3([x(i),y(i),z(i)],[x(i+1),y(i+1),z(i+1)]);
end
Scumulata=cumsum([0,L(1:end)]);


% Plot della curva interpolata e dei punti originali
figure(1)
plot3(x, y, z, 'bx-', 'LineWidth', 2);
hold on;
plot3(nodi_trasformati(1,:), nodi_trasformati(2,:), nodi_trasformati(3,:), 'ro', 'MarkerSize', 8, 'MarkerFaceColor', 'r');
xlabel('X'); ylabel('Y'); zlabel('Z');
title('Curva spline interpolata tra i punti dati');
grid on;
axis equal;
hold off;

% figure    PROBABILMENTE È RIBA INUTILE CHE ANDRÀ TOLTA PERCHÈ POCO
% SIGNIFICATIVA 
% subplot(3,1,1)
% plot(t_trat,Scumulata,'-b')
% title("Ascissa curvilinea")
% grid on
% xlabel("tempo [s]")
% ylabel("ascissa curvilinea [m]")
% 
% subplot(3,1,2)
% plot(t_trat(1:end-1),gradient(Scumulata,t_trat),'-b') % dividi per il tempo
% grid on
% xlabel("tempo [s]")
% ylabel("velocità [m/s]")
% 
% subplot(3,1,3)
% plot(t_trat(1:end-2),gradient(gradient(Scumulata,t_trat),t_trat),'-b') % dividi per il tempo ^2
% grid on
% xlabel("tempo [s]")
% ylabel("accelerazione [m/s^2]")


%% Look ahead traiettoria

% Parametri primo tratto
v1_max=0.3;
a1_max=0.6;
d1_max=0.5;
v1_in=0;
v1_end=0.2;
v_lim1=[];
s1_finale=333;

% Parametri secondo tratto
v2_max=0.5;
a2_max=2;
d2_max=1;
v2_in=v1_end; 
v2_end=0.1;
v_lim2=[];
s2_finale=500;

% Parametri terzo tratto
v3_max=0.4;
a3_max=0.4;
d3_max=0.5;
v3_in=v2_end; 
v3_end=0;
v_lim3=[];
s3_finale=length(Scumulata);
% calcolo delle grandezze cinematiche
[v_s1,v_t1,s1,t1,v_lim1,a_s1,a_t1]=look_ahead(Scumulata(1:s1_finale),v1_max,a1_max,d1_max,v1_in,v1_end,0);
[v_s2,v_t2,s2,t2,v_lim2,a_s2,a_t2]=look_ahead(Scumulata(s1_finale:s2_finale),v2_max,a2_max,d2_max,v2_in,v2_end,Scumulata(s1_finale)-s1(end));
[v_s3,v_t3,s3,t3,v_lim3,a_s3,a_t3]=look_ahead(Scumulata(s2_finale:s3_finale),v3_max,a3_max,d3_max,v3_in,v3_end,Scumulata(s2_finale)-s2(end));
% concateno i tratti 
v_s=[v_s1(1:end-1) v_s2(1:end-1) v_s3(1:end)];
v_t=[v_t1(1:end-1) v_t2(1:end-1) v_t3(1:end)];
s=[s1(1:end-1) s2(1:end-1) s3(1:end)];
t=[t1(1:end-1) t1(end)+t2(1:end-1) t1(end)+t2(end)+t3(1:end)];
v_lim=[v_lim1(1:end-1) v_lim2(1:end-1) v_lim3(1:end)];
a_s=[a_s1(1:end-1) a_s2(1:end-1) a_s3(1:end)];
a_t=[a_t1(1:end-1) a_t2(1:end-1) a_t3(1:end)]; 

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
plot(t,v_lim,t,v_tn,t, v_t, 'r', 'LineWidth', 2); % unità di misura???
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
axis("equal")

% Spazio vs Tempo
subplot(2,2,3);
plot(t,s,'r', 'LineWidth', 2); 
xlabel('Tempo (s)');
ylabel('Spazio (m)');
title('Spostamento in funzione del tempo');
grid on;



%% Proietto la velocità del tratto centrale lungo gli assi 

% for i=1:length(Scumulata)-1
%     ds=gradient(s);
%     v(:,i)=v_t(i+1) * [x(i+1)-x(i);y(i+1)-y(i);z(i+1)-z(i)]/ds(i);
% %     v=[zeros(3,1) v];
%     a(:,i)=a_t(i+1) * [x(i+1)-x(i);y(i+1)-y(i);z(i+1)-z(i)]/ds(i);
% %     a=[zeros(3,1) a];
% end

dsx=gradient(x,s);
dsy=gradient(y,s);
dsz=gradient(z,s);
ds=sqrt(dsx.^2+dsy.^2+dsz.^2);
tang=[dsx; dsy; dsz]./ds;

v=tang.*v_t;
a=tang.*a_t;

figure
subplot(3,1,1)
plot(t,v(1,:),'LineWidth',1.5)
hold on
plot(t,gradient(x,t),'r',[t(1) t(end)],[0 0],'k')
legend("v proiettata","v numerica",'Location','best')
title("Proiezione velocità asse x")
xlabel("t [s]")
ylabel("v [m/s]")
grid on
subplot(3,1,2)
plot(t,v(2,:),'LineWidth',1.5)
hold on
plot(t,gradient(y,t),'r',[t(1) t(end)],[0 0],'k')
legend("v proiettata","v numerica",'Location','best')
title("Proiezione velocità asse y")
xlabel("t [s]")
ylabel("v [m/s]")
grid on
subplot(3,1,3)
plot(t,v(3,:),'LineWidth',1.5)
hold on
plot(t,gradient(z,t),'r',[t(1) t(end)],[0 0],'k')
legend("v proiettata","v numerica",'Location','best')
title("Proiezione velocità asse z")
xlabel("t [s]") 
ylabel("v [m/s]")
grid on

figure
subplot(3,1,1)
plot(t,a(1,:),'b','LineWidth',1.5)
hold on
plot(t,gradient(v(1,:),t),'r',[t(1) t(end)],[0 0],'k')
legend("a tang proiettata","a numerica",'Location','best')
grid on
title("Proiezione accelerazione asse x")
xlabel("t [s]")
ylabel("a [m/s^2]")
subplot(3,1,2)
plot(t,a(2,:),'b','LineWidth',1.5)
hold on
plot(t,gradient(v(2,:),t),'r',[t(1) t(end)],[0 0],'k')
legend("a tang proiettata","a numerica",'Location','best')
grid on
title("Proiezione accelerazione asse y")
xlabel("t [s]") 
ylabel("a [m/s^2]")
subplot(3,1,3)
plot(t,a(3,:),'b','LineWidth',1.5)
hold on
plot(t,gradient(v(3,:),t),'r',[t(1) t(end)],[0 0],'k')
legend("a tang proiettata","a numerica",'Location','best')
grid on
title("Proiezione accelerazione asse z")
xlabel("t [s]")
ylabel("a [m/s^2]")



%% plot di tutta la traiettoria completa in x,y,z 

figure 
subplot(3,1,1)
plot(tt_1,Siniziale(1,:),tt_1(end)+t,x,tt_1(end)+t(end)+tt_2,Sfinale(1,:))
title("Traiettoria completa asse x")
grid on
subplot(3,1,2)
plot(tt_1,Siniziale(2,:),tt_1(end)+t,y,tt_1(end)+t(end)+tt_2,Sfinale(2,:))
title("Traiettoria completa asse y")
grid on
subplot(3,1,3)
plot(tt_1,Siniziale(3,:),tt_1(end)+t,z,tt_1(end)+t(end)+tt_2,Sfinale(3,:))
title("Traiettoria completa asse z")
grid on

%% simscape

sampleTime = 0.1;
numSteps = length(v_s);
time = sampleTime*(1:numSteps-1);
time = time';
siminX = timeseries([Siniziale(1,:) x Sfinale(1,:) ],[tt_1 tt_1(end)+t tt_1(end)+t(end)+tt_2]);
siminY = timeseries([Siniziale(2,:) y Sfinale(2,:) ],[tt_1 tt_1(end)+t tt_1(end)+t(end)+tt_2]);
siminZ = timeseries([Siniziale(3,:) z Sfinale(3,:) ],[tt_1 tt_1(end)+t tt_1(end)+t(end)+tt_2]);

siminVX = timeseries(v(1,:),time);
siminVY = timeseries(v(2,:),time);
siminVZ = timeseries(v(3,:),time);

% Calcolare cinematica inversa iniziale
geometria_SCARA
Q=[];
[Q,ok]=SCARAinv([Siniziale(1:3,1)' 0]',L,[2*pi/3 pi/2 0.2 0]',1e-6,50);

plot_SCARA_trajectory(Q,[0.5515 1.8684 0.2299 -2.419]',[0 0 0]',L,1,[37 37])

%% Calcolo la cinematica inversa della traiettoria 
% cinematica inversa del tratto centrale
QQ=[]; % posizione dei giunti del robot
QQd=[]; % velocità ai giunti del robot
QQdd=[]; % accelerazione ai giunti del robot
Q0=Q;

% v=[zeros(3,1) v] ;
v=[gradient(x,t); gradient(y,t); gradient(z,t)];

% a=[zeros(3,1) a];
a=[gradient(v(1,:),t);gradient(v(2,:),t);gradient(v(3,:),t)];


for i=1:length(Scumulata)
    % QQ(:,i)=SCARAinv([x(i);y(i);z(i);0],L,Q0,1e-6,55);
    QQ(:,i)=SCARAinvAnalitica([x(i);y(i);z(i);0],L);
    J=SCARAjac(QQ(:,i),L);
    QQd(:,i)=J^(-1)*[v(:,i);0];
    Jp=SCARAjacP(QQ(:,i),QQd(:,i),L);
    QQdd(:,i)=J^(-1)*([a(:,i);0]-Jp*QQd(:,i));
    Q0=QQ(:,i);
end

% plot grandezze cinematiche joint space area centrale
figure
subplot(3,1,1)
plot(t,QQ(1,:),[t(1) t(end)],[0 0],'k')
grid on
title("Posizione joint 1 tratto centrale")
xlabel("t [s]")
ylabel("q [rad]")
subplot(3,1,2)
plot(t,QQ(2,:),[t(1) t(end)],[0 0],'k')
grid on
title("Posizione joint 2 tratto centrale")
xlabel("t [s]")
ylabel("q [rad]")
subplot(3,1,3)
plot(t,QQ(3,:),[t(1) t(end)],[0 0],'k')
grid on
title("Posizione joint 3 tratto centrale")
xlabel("t [s]")
ylabel("s [m]")


QQdn(1,:)=gradient(QQ(1,:),t);
QQdn(2,:)=gradient(QQ(2,:),t);
QQdn(3,:)=gradient(QQ(3,:),t);

figure
subplot(3,1,1)
plot(t,QQd(1,:),t,QQdn(1,:),[t(1) t(end)],[0 0],'k')
legend("v","vn",'Location','Best')
grid on
title("Velocità joint 1 tratto centrale")
xlabel("t [s]")
ylabel("v [rad/s]")
subplot(3,1,2)
plot(t,QQd(2,:),t,QQdn(2,:),[t(1) t(end)],[0 0],'k')
legend("v","vn",'Location','Best')
grid on
title("Velocità joint 2 tratto centrale")
xlabel("t [s]")
ylabel("v [rad/s]")
subplot(3,1,3)
plot(t,QQd(3,:),t,QQdn(3,:),[t(1) t(end)],[0 0],'k')
legend("v","vn",'Location','Best')
grid on
title("Velocità joint 3 tratto centrale")
xlabel("t [s]")
ylabel("v [m/s]")

% calolo della accelerazione normale da sommare a quella tangenziale
% calcolata numericamente
% k=curvatura3D(x,y,z);
% a_n = QQdn.^2 .* k;
QQddn(1,:)=gradient(QQdn(1,:),t);%+a_n(1,:);
QQddn(2,:)=gradient(QQdn(2,:),t);%+a_n(2,:);
QQddn(3,:)=gradient(QQdn(3,:),t);%+a_n(3,:);

figure
subplot(3,1,1)
plot(t,QQdd(1,:),t,QQddn(1,:),[t(1) t(end)],[0 0],'k')
legend("a","an",'Location','Best')
grid on
title("Accelerazioni joint 1 tratto centrale")
xlabel("t [s]")
ylabel("a [rad/s^2]")
subplot(3,1,2)
plot(t,QQdd(2,:),t,QQddn(2,:),[t(1) t(end)],[0 0],'k')
legend("a","an",'Location','Best')
grid on
title("Accelerazioni joint 2 tratto centrale")
xlabel("t [s]")
ylabel("a [rad/s^2]")
subplot(3,1,3)
plot(t,QQdd(3,:),t,QQddn(3,:),[t(1) t(end)],[0 0],'k')
legend("a","an",'Location','Best')
grid on
title("Accelerazioni joint 3 tratto centrale")
xlabel("t [s]")
ylabel("a [m/s^2]")

% debug velocità e accelerazioni: provo a ricalcolare la cinematica diretta
for i=1:length(Scumulata)
    SS(:,i)=SCARAdir(QQ(:,i),L);
    J=SCARAjac(QQ(:,i),L);
    debug_v(:,i)=J*[QQd(:,i)];
    debug_v_square(i)=sqrt((debug_v(1,i)^2+debug_v(2,i)^2+debug_v(3,i)^2));
    debug_a(:,i)=SCARAjac(QQ(:,i),L)*QQdd(:,i)+SCARAjacP(QQ(:,i),QQd(:,i),L)*QQd(:,i);
    debug_a_square(i)=sqrt((debug_a(1,i)^2+debug_a(2,i)^2+debug_a(3,i)^2));
end

figure
subplot(3,1,1)
plot(t,debug_a(1,:),'b',t,a(1,:),'r')
title("Debug accelerazione: ricalcolo cinematica diretta")
grid on
subplot(3,1,2)
plot(t,debug_a(2,:),'b',t,a(2,:),'r')
grid on
subplot(3,1,3)
plot(t,debug_a(3,:),'b',t,a(3,:),'r')
grid on

figure
subplot(3,1,1)
plot(t,debug_v(1,:),'b',t,v(1,:),'r')
title("Debug velocità: ricalcolo cinematica diretta")
grid on
subplot(3,1,2)
plot(t,debug_v(2,:),'b',t,v(2,:),'r')
grid on
subplot(3,1,3)
plot(t,debug_v(3,:),'b',t,v(3,:),'r')
grid on

figure
subplot(3,1,1)
plot(t,SS(1,:),'b',t,x(:),'r')
title("Debug posizioni: ricalcolo cinematica diretta")
grid on
subplot(3,1,2)
plot(t,SS(2,:),'b',t,y(:),'r')
grid on
subplot(3,1,3)
plot(t,SS(3,:),'b',t,z(:),'r')
grid on

figure 
plot(t,debug_v_square,'b')
title("Debug vel totale e acc totale")
hold on
plot(t,debug_a_square,'r')


% figure
plot_SCARA_trajectory(QQ(:,20),QQ(:,80),[0 0 0]',L,1,[37 37])

%% Plot grandezze cinematiche traiettoria completa

figure
subplot(3,1,1)
plot(tt_1,q_j1_1,'-b',tt_1(end)+t,QQ(1,:),tt_1(end)+t(end)+tt_2,q_j1_2,[0 tt_1(end)+t(end)+tt_2(end)],[0 0],'k')
grid on
title("Posizione joint 1")
xlabel("t [s]")
ylabel("q [rad]")
subplot(3,1,2)
plot(tt_1,q_j2_1,'-b',tt_1(end)+t,QQ(2,:),tt_1(end)+t(end)+tt_2,q_j2_2,[0 tt_1(end)+t(end)+tt_2(end)],[0 0],'k')
grid on
title("Posizione joint 2")
xlabel("t [s]")
ylabel("q [rad]")
subplot(3,1,3)
plot(tt_1,q_j3_1,'-b',tt_1(end)+t,QQ(3,:),tt_1(end)+t(end)+tt_2,q_j3_2,[0 tt_1(end)+t(end)+tt_2(end)],[0 0],'k')
grid on
title("Posizione joint 3")
xlabel("t [s]")
ylabel("s [m]")

figure
subplot(3,1,1)
plot(tt_1,qd_j1_1,'-b',tt_1(end)+t,QQd(1,:),tt_1(end)+t(end)+tt_2,qd_j1_2,[0 tt_1(end)+t(end)+tt_2(end)],[0 0],'k')
grid on
title("Velocità joint 1")
xlabel("t [s]")
ylabel("v [rad/s]")
subplot(3,1,2)
plot(tt_1,qd_j2_1,'-b',tt_1(end)+t,QQd(2,:),tt_1(end)+t(end)+tt_2,qd_j2_2,[0 tt_1(end)+t(end)+tt_2(end)],[0 0],'k')
grid on
title("Velocità joint 2")
xlabel("t [s]")
ylabel("v [rad/s]")
subplot(3,1,3)
plot(tt_1,qd_j3_1,'-b',tt_1(end)+t,QQd(3,:),tt_1(end)+t(end)+tt_2,qd_j3_2,[0 tt_1(end)+t(end)+tt_2(end)],[0 0],'k')
grid on
title("Velocità joint 3")
xlabel("t [s]")
ylabel("v [m/s]")

figure
subplot(3,1,1)
plot(tt_1,qdd_j1_1,'-b',tt_1(end)+t,QQdd(1,:),tt_1(end)+t(end)+tt_2,qdd_j1_2,[0 tt_1(end)+t(end)+tt_2(end)],[0 0],'k')
grid on
title("Accelerazioni joint 1")
xlabel("t [s]")
ylabel("a [rad/s^2]")
subplot(3,1,2)
plot(tt_1,qdd_j2_1,'-b',tt_1(end)+t,QQdd(2,:),tt_1(end)+t(end)+tt_2,qdd_j2_2,[0 tt_1(end)+t(end)+tt_2(end)],[0 0],'k')
grid on
title("Accelerazioni joint 2")
xlabel("t [s]")
ylabel("a [rad/s^2]")
subplot(3,1,3)
plot(tt_1,qdd_j3_1,'-b',tt_1(end)+t,QQdd(3,:),tt_1(end)+t(end)+tt_2,qdd_j3_2,[0 tt_1(end)+t(end)+tt_2(end)],[0 0],'k')
grid on
title("Accelerazioni joint 3")
xlabel("t [s]")
ylabel("a [m/s^2]")

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

simtime=[tt_1 tt_1(end)+t(2:end-1) tt_1(end)+t(end)+tt_2];

figure
subplot(3,1,1)
plot(simtime,Q_j(1,:),'*-b',simtime,Q_j(2,:),'*-r',simtime,Q_j(3,:),'*-g',simtime,Q_j(4,:),'*-k')
grid on
subplot(3,1,2)
plot(simtime,Qd_j(1,:),'*-b',simtime,Qd_j(2,:),'*-r',simtime,Qd_j(3,:),'*-g',simtime,Qd_j(4,:),'*-k')
grid on
subplot(3,1,3)
plot(simtime,Qdd_j(1,:),'*-b',simtime,Qdd_j(2,:),'*-r',simtime,Qdd_j(3,:),'*-g',simtime,Qdd_j(4,:),'*-k')
grid on

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
    Ek(i)=0.5*Qd_j(:,i)'*Je'*M*Je*Qd_j(:,i); % energia cinetica con gravità

    % Ep=Se'*M*Ag;
    Se=SCARAdirEstesa(Q_j(:,i),L,G);
    Ep(i)=Se'*M*Ag-Ep0;
end

Et=Ek+Ep;

figure
plot(simtime,Ek,simtime,Ep,simtime,Et,[simtime(1) simtime(end)],[0 0],'k')
%plot(tt_1,Ek(1:length(tt_1)),tt_1,Ep(1:length(tt_1)),tt_1,Ek(1:length(tt_1))+Ep(1:length(tt_1)),[tt_1(1) tt_1(end)],[0 0],'k')
ylim([-20 100])
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




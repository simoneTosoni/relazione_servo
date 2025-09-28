clear
close all

% definizione caratteristiche robot
geometria_SCARA

% ##Fse=[1 1 0 0 0 0 0 0]'; % vettore forse esterne applicate Fse=[x y 0 0 c 0 0 0]'
% 
% ##Q=[pi/4 pi/4 0.1 0]';
% ##
% ##% stampo posizione iniziale 
% ##figure(10)
% ##S=SCARAdir(Q,L)
% ##PlotSCARA(Q,L,'b',10)
% ##hold on
% ##grid on
% ##my_PlotAreaSCARA(L,10)
% ##title("Parte 1 esercizio")
% ##
% ##% stampo configurazioni robot inizio del moto e fine del moto
% ##my_PlotSCARA(Q,L,'b',10)
% ##hold off
% ##
% ##Qp=[-2.66 -2.2]';
% ##Qpp=[26.63 -12.360]';

% ##Se=[x, y, z, c, xg3, yg3, zg3, xg2, yg2, zg2, alpha+beta, xg1, yg1, zg1, alpha];

% M=diag([0.5 0.5 0.5 0.002 5 5 5 7 7 7 0.045 9 9 9 0.1]);  % matrice diagolane delle masse dei link del robot e del gripper
% 
% Ag=[0 0 -9.81 0 0 0 -9.81 0 0 -9.81 0 0 0 -9.81 0]';
% Fse=[1 1 0 0 0 0 0 0 0 0 0 0 0 0 0]';

% ##Je=my_SCARAJacEsteso(Q,L,G);
% ##Jep=my_SCARAJacpEsteso(Q,Qp,L,G);
% ##Fq=(Je'*M*Je)*Qpp+(Je'*M*Jep)*Qp-Je'*Fse;

% parte 2

% ##Si=[1.2 0.5 0 0]';
% ##Sf=[0.7 0.8 0.1 pi]';

cinematica_SCARA_jac_inv

% M=diag([0.5 0.5 0.5 10 5 5 5 7 7 7 0.45 9 9 9 1]);  % matrice diagolane delle masse dei link del robot e del gripper

Ag=[0 0 9.81 0 0 0 9.81 0 0 9.81 0 0 0 9.81 0]';
Fse=[0 0 0 0 0 0 0 0 0 0 0 0 0 0 0]';

% calcolo le coppie dei motori nel punto 2.a PIANO ORIZZONTALE

% ##for i=1:n
% ##    Je=SCARAJacEsteso(QQ(:,i),L,G);
% ##    Sep(:,i)=Je*QQd(:,i);
% ##    Jep=SCARAJacpEsteso(QQ(:,i),QQd(:,i),L,G);
% ##    FFq(:,i)=(Je'*M*Je)*QQdd(:,i)+(Je'*M*Jep)*QQd(:,i)-Je'*Fse;
% ##end
% ##
% ##figure(8)
% ##plot(tt,FFq)
% ##title("Coppie motori nel piano orizzontale")
% ##legend("alpha","beta","s","gamma")
% ##grid on
% ##
% ##Se0=SCARAdirEstesa(QQ(:,1),L,G);
% ##Ep0=0;%Se0'*M*Ag; % Energia potenziale all'iziniio del moto per il calcolo dell'energia potenziale relati
% ##
% ##for i=1:n
% ##    Je=SCARAJacEsteso(QQ(:,i),L,G);
% ##    Ek(i)=0.5*QQd(:,i)'*Je'*M*Je*QQd(:,i); % energia cinetica 
% ##
% ##    % Ep=Se'*M*Ag;
% ##    Se=SCARAdirEstesa(QQ(:,i),L,G);
% ##    Ep(i)=0;%Se'*M*Ag-Ep0;
% ##end
% ##
% ##figure(9)
% ##plot(tt,Ek,tt,Ep,tt,Ek+Ep)
% ##title("Energia piano orizzontale")
% ##legend("En cinetica","En potenziale","En totale")
% ##grid on
% ##
% ##% calcolo della potenza W=FFq'QQp+Fse'Sep
% ##for i=1:n
% ##    W(i)=FFq(:,i)'*QQd(:,i)+Fse'*Sep(:,i);
% ##end
% ##
% ##dEt=diff(Ek)/dt;
% ##
% ##figure(10)
% ##plot(tt,W,tt(1:end-1),dEt)
% ##legend("Potenza","der En tot")
% ##title("Potenza piano orizzontale")
% ##grid on


% calcolo delle coppie ai motori con azione gravitazionale 2b

for i=1:n
    Je=SCARAJacEsteso(QQ(:,i),L,G);
    Sep(:,i)=Je*QQd(:,i);
    Jep=SCARAJacpEsteso(QQ(:,i),QQd(:,i),L,G);
    FFq(:,i)=(Je'*M*Je)*QQdd(:,i)+(Je'*M*Jep)*QQd(:,i)-Je'*Fse+Je'*M*Ag;
end

figure(12)
plot(tt,FFq)
title("Coppie motori nel piano verticale")
legend("alpha","beta","s","gamma")
grid on

% debug con energia potenziale e cinetica (conservazione dell'energia)

Se0=SCARAdirEstesa(QQ(:,1),L,G);
Ep0=Se0'*M*Ag; % Energia potenziale all'iziniio del moto per il calcolo dell'energia potenziale relativa

for i=1:n
    Je=SCARAJacEsteso(QQ(:,i),L,G);
    Ek(i)=0.5*QQd(:,i)'*Je'*M*Je*QQd(:,i); % energia cinetica con gravità

    % Ep=Se'*M*Ag;
    Se=SCARAdirEstesa(QQ(:,i),L,G);
    Ep(i)=Se'*M*Ag-Ep0;
end

figure(13)
plot(tt,Ek,tt,Ep,tt,Ek+Ep,[tt(1) tt(end)],[0 0],'k')
ylim([-5 30])
title("Energia piano verticale")
legend("En cinetica","En potenziale","En totale","Location","best")
grid on

% calcolo della potenza W=FFq'QQp+Fse'Sep
for i=1:n
    W(i)=FFq(:,i)'*QQd(:,i)+Fse'*Sep(:,i);
end

dEt=diff(Ek+Ep)/dt;

figure(14)
plot(tt,W,tt(1:end-1),dEt)
legend("Potenza","der En tot")
title("Potenza piano verticale")
grid on

clear
close all

addpath /home/simone/Uni/magistrale/servo_systems_and_robotics/progetto/progetto_legnani/progetto/funzioni/


% definizione caratteristiche robot
geometria_SCARA

cinematica_SCARA_jac_inv

% calcolo delle coppie ai motori con azione gravitazionale 

for i=1:n
    Je=SCARAJacEsteso(QQ(:,i),L,G);
    Sep(:,i)=Je*QQd(:,i);
    Jep=SCARAJacpEsteso(QQ(:,i),QQd(:,i),L,G);
    FFq(:,i)=(Je'*M*Je)*QQdd(:,i)+(Je'*M*Jep)*QQd(:,i)-Je'*Fse+Je'*M*Ag;
end

figure(12)
subplot(3,1,1)
plot(tt,FFq(1:2,:))
title("Coppie e forze motori nel piano verticale")
legend("alpha","beta",'Location','best')
ylabel("[N/m]")
grid on
subplot(3,1,2)
plot(tt,FFq(3,:),'Color',[0.9290 0.6940 0.1250])
legend("s",'Location','best')
ylabel("[N]")
grid on
subplot(3,1,3)
plot(tt,FFq(4,:),'Color',[0.4940 0.1840 0.5560])
legend("gamma",'Location','best')
ylabel("[N/m]")
xlabel("tempo [s]")
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
xlabel("tempo [s]")
ylabel("[J]")
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
xlabel("tempo [s]")
ylabel("[W]")
grid on

% exportgraphics(figure(12),"coppie.eps",'ContentType', 'vector')
% exportgraphics(figure(13),"energia.eps",'ContentType', 'vector')
% exportgraphics(figure(14),"potenza.eps",'ContentType', 'vector')

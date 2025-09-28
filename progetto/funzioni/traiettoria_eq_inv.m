% Calcola la traiettoria nello spazione dei giunti in maniera numerica

close all

traiettoria_eq

Q0=[1.2014 1.7384 0.0799 -2.2398]';

for i=1:length(trajectory)
%     t=(i-1)*T/(n-1);
%     tt(i)=t;
%     [x(i) xd(i) xdd(i)]=cicloidale(t,T,Si(1),Sf(1)-Si(1));
%     [y(i) yd(i) ydd(i)]=cicloidale(t,T,Si(2),Sf(2)-Si(2));
%     [z(i) zd(i) zdd(i)]=cicloidale(t,T,Si(3),Sf(3)-Si(3));
%     [g(i) gd(i) gdd(i)]=cicloidale(t,T,Si(4),Sf(4)-Si(4));
    
%     S(:,i)=[x(i) y(i) z(i) g(i)]';
%     Sd(:,i)=[xd(i) yd(i) zd(i) gd(i)]';
%     Sdd(:,i)=[xdd(i) ydd(i) zdd(i) gdd(i)]';
    
    QQ(:,i)=SCARAinv([trajectory(1:3,i);0],L,Q0,1e-6,55);
%     J=SCARAjac(QQ(:,i),L);
%     QQd(:,i)=pinv(J)*Sd(:,i);
%     Jp=SCARAjacP(QQ(:,i),QQd(:,i),L);
%     QQdd(:,i)=J^(-1)*(Sdd(:,i)-Jp*QQd(:,i));
    Q0=QQ(:,i);
    
end

sampleTime = 0.1;
numSteps = length(trajectory);
time = sampleTime*(0:numSteps-1);
time = time';
simin1 = timeseries(QQ(1,:),time);
simin2 = timeseries(QQ(2,:),time);
simin3 = timeseries(QQ(3,:),time);
simin4 = timeseries(QQ(4,:),time);

% controllo che la cinematica inversa della traiettoria sia corretta
for i=1:length(trajectory)
    pippo(:,i)=SCARAdir(QQ(:,i),L);
end

figure(1)
hold on
plot3(pippo(1,:),pippo(2,:),pippo(3,:),'o','DisplayName',"check trajectory")
title("traiettoria")
xlabel("x")
ylabel("y")
zlabel("z")
axis("equal")
grid on

% differenza nell'ordine di grandezza di 10^-7
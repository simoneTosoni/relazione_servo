clear 
close all

% definisco l'ascissa curvilinea come la somma di tutti i tratti della
% traiettoria definita per punti [x;y;z;0]

P1=[-2,3,0,1]';
% primo tratto t1
P2=[0.5,3,0,1]';
% secondo tratto t2
P3=[-1,4,0,1]';
% terzo tratto t3
P4=[-1,0,0,1]';
% tratto semicircolare t4
Pc=[0,0,0,1]'; % centro traiettoria semicircolare

% Trasformo i punti nello spazio

scala=0.1;

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

A=mtraj*P1;
B=mtraj*P2;
C=mtraj*P3;
D=mtraj*P4;
E=mtraj*Pc;

% calolo lunghezza dei singoli tratti
L1=distanza3(A,B);
L2=distanza3(B,C);
L3=distanza3(C,D);
L4=2*pi*(distanza3(D,E))/2; % Lunghezza semicirconferenza

Stot=L1+L2+L3+L4

% definisco l'asse temporale unico 
Tin=0;
Tfin=10;
dt=0.05;
t=Tin:dt:Tfin;

% Proietto la traiettoria sugli assi cartesiani
% Tratto L1

T1=2;
S1=[];
for i=1:length(t)
    if t(i)<T1
%         [s1,s1p,s1pp]=cicloidale(t(i),T1,0,1);
        [s1,s1p,s1pp]=tretratti(t(i),T1,0,1,1/3,1/3);
        S1=[S1 A+(B-A)*s1];
    end
end




% Tratto L2

T2=4;
S2=[];
for i=1:length(t)
    if t(i)<T2 && t(i)>T1
%         [s2,s2p,s2pp]=cicloidale(t(i)-T1,T2-T1,0,1);
        [s2,s2p,s2pp]=tretratti(t(i)-T1,T2-T1,0,1,1/3,1/3);
        S2=[S2 B+(C-B)*s2];
    end
end


% Tratto L3

T3=6;
% dt3=0.1;
% t3=0:dt3:T3;
S3=[];
for i=1:length(t)
    if t(i)<T3 && t(i)>T2
%         [s3,s3p,s3pp]=cicloidale(t(i)-T2,T3-T2,0,1);
        [s3,s3p,s3pp]=tretratti(t(i)-T2,T3-T2,0,1,1/3,1/3);
        S3=[S3 C+(D-C)*s3];
    end
end



% Tratto L4
cd=C-D;
de=D-E;
vettore_piano=cross(cd(1:3),de(1:3));
versore_piano=vettore_piano/sqrt(vettore_piano(1)^2+vettore_piano(2)^2+vettore_piano(3)^2)

T4=Tfin;
% dt4=0.1;
% t4=0:dt4:T4;
S4=[];
for i=1:length(t)
    if t(i)<T4 && t(i)>T3
%         [s4,s4p,s4pp]=cicloidale(t(i)-T3,T4-T3,0,pi);
        [s4,s4p,s4pp]=tretratti(t(i)-T3,T4-T3,0,pi,1/3,1/3);
        for alpha=s4
            S4=[S4 E+Rot3(versore_piano,alpha)*(D-E)];
        end
    end
end



% Spostamento Totale 
S=[A S1 B S2 C S3 D S4]


% stampo per debug
figure
plot3(A(1),A(2),A(3),'xb')
hold on
grid on
xlabel("x")
ylabel("y")
zlabel("z")
plot3(B(1),B(2),B(3),'xb')
plot3(C(1),C(2),C(3),'xb')
plot3(D(1),D(2),D(3),'xb')
plot3(E(1),E(2),E(3),'xb')

plot3(S(1,:),S(2,:),S(3,:),'o-r')

figure 
plot(t,ascissa_curva(S),Tfin,Stot,'xr')
title("Ascissa curvilinea")
grid on
xlabel("tempo [s]")
ylabel("ascissa curvilinea [m]")

figure 
plot(t(1:end-1),diff(ascissa_curva(S)))
grid on
xlabel("tempo [s]")
ylabel("velocità [m]")

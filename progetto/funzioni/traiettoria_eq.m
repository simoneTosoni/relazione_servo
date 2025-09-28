% clear 
close all

T_primo_tratto=2;
T_secondo_tratto=2;
T_terzo_tratto=2;
T_quarto_tratto=2;
Tfin=T_primo_tratto+T_secondo_tratto+T_terzo_tratto+T_quarto_tratto; % durata 
dt=0.1; % intervallo step

t=0:dt:Tfin;

t1=0:dt:T_primo_tratto;
P2=[-2,3,0,0]';
A=[0.5,3,0,pi/2]';
P=[];
for i=t1
    P=[P P2+(A-P2)/T_primo_tratto*i];
end
P(:,end)=[];

t2=0:dt:T_secondo_tratto;
B=[-1,4,0,pi]';
R=[]; 
for i=t2
    R=[R A+(B-A)/T_secondo_tratto*i];
end
R(:,end)=[];

t3=0:dt:T_terzo_tratto;
C=[-1,0,0,3*pi/2]';
Q=[];
for i=t3
    Q=[Q B+(C-B)/T_terzo_tratto*i];
end
Q(:,end)=[];

t4=0:dt:T_quarto_tratto;
% P3=[1,0,0,2*pi]';
Pc=[0,0,0,2*pi]';
W=[];
alpha=0;
raggio=1;
Rot=[cos(alpha) -sin(alpha) 0 0;sin(alpha) cos(alpha) 0 0; 0 0 1 0; 0 0 0 1];

for i=t4
    alpha = 0 + pi/T_quarto_tratto*i;
    Rot=[cos(alpha) -sin(alpha) 0 0;sin(alpha) cos(alpha) 0 0; 0 0 1 0; 0 0 0 1];
    W=[W Pc+Rot*(C-Pc)];
end

trajectory(:,:)=[P(:,:) R(:,:) Q(:,:) W(:,:)];

G=trajectory(4,:);
Gp=P(4,:);
Gr=R(4,:);
Gq=Q(4,:);
Gw=W(4,:);


%% trasformazione della traiettoria
 
% per routare il piano xy su cui giace la traiettoria uso le matrici di rototraslazione
trajectory=[trajectory(1:3,:); ones(1,length(trajectory))];
P=[P(1:3,:); ones(1,length(P))];
R=[R(1:3,:); ones(1,length(R))];
Q=[Q(1:3,:); ones(1,length(Q))];
W=[W(1:3,:); ones(1,length(W))];

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
    
for i=1:length(trajectory)   
  trajectory(:,i)=mtraj*trajectory(:,i)*scala;
end

for i=1:length(P)
    P(:,i)=mtraj*P(:,i)*scala;
end

for i=1:length(R)
    R(:,i)=mtraj*R(:,i)*scala;
end

for i=1:length(Q)
    Q(:,i)=mtraj*Q(:,i)*scala;
end

for i=1:length(W)
    W(:,i)=mtraj*W(:,i)*scala;
end

trajectory=[trajectory(1:3,:);G(1,:)];
P=[P(1:3,:);Gp(1,:)];
R=[R(1:3,:);Gr(1,:)];
Q=[Q(1:3,:);Gq(1,:)];
W=[W(1:3,:);Gw(1,:)];

%% Stampo risultati

figure(1)
plot3(trajectory(1,:),trajectory(2,:),trajectory(3,:))
title("traiettoria")
xlabel("x")
ylabel("y")
zlabel("z")
axis("equal")
grid on

geometria_SCARA
plot_SCARA_trajectory([0 pi/2 0 0]',[0.5515 1.8684 0.2299 -2.419]',[0 0 0]' ,L,1,[37 37])

figure(2)
plot(t,trajectory(1,:))
grid on
title("Direzione X")

figure(3)
plot(t,trajectory(2,:))
grid on
title("Direzione Y")

figure(4)
plot(t,trajectory(3,:))
grid on
title("Direzione Z")

figure(5)
plot(t,trajectory(4,:))
grid on
title("Angolo polso")





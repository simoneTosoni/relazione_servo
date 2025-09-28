% Definizione della traiettoria a forma di t stilizzata del robot SCARA per il progetto di Servo System and Robotics 
% Tosoni Simone, matricola 731405

clear 
close all
clc

scaling_factor=0.05;

figure(1)
% disegno il segmento verticale
vert=[-1,4,0;
      -1,0,0]'*scaling_factor;
plot3(vert(1,:),vert(2,:),vert(3,:),'o-',"linewidth",2)

hold on
grid on
axis("equal")
view([0 90])
xlabel('x')
ylabel('y')
zlabel('z')
xlim([-3 2])
ylim([-2 5])
zlim([-1 1])
plot3([-3 2],[0 0],[0 0],'k')
title("Traiettoria assegnata")


% disegno il semicerchio
n=10;
i=0:1:n;
x=-cos(i*pi/n);
y=-sqrt(1-x.^2);
z=zeros(1,n+1);
scfr=[x;y;z]*scaling_factor;
plot3(scfr(1,:),scfr(2,:),scfr(3,:),'o-',"linewidth",2)



% disegno il segmento orizontale
oriz=[-2,3,0;
      0.5,3,0]'*scaling_factor;
plot3(oriz(1,:),oriz(2,:),oriz(3,:),'o-',"linewidth",2)
hold off

% racchiudo tutti i punti in un unico vettore
% il primo tratto è costituito dal tratto verticale e dalla semicfr
% il secondo tratto è costituito dal tratto orizontale

temp=[vert scfr];
tratto_1_i=[temp; ones(1,columns(temp))];

tratto_2_i=[oriz; ones(1,columns(oriz))];

% per routare il piano xy su cui giace la traiettoria uso le matrici di rototraslazione

mtraj_1=[cos(pi/2) -sin(pi/2) 0 1;
         sin(pi/2) 0 cos(pi/2) 0.4;
         0 0 1 0;
         0 0 0 1];
       
mtraj_2=[1 0 0 0;
         0 cos(pi/6) -sin(pi/6) 0;
         0 sin(pi/6) cos(pi/6) 0;
         0 0 0 1];
         
mtraj=mtraj_1*mtraj_2;         
    
for i=1:columns(tratto_1_i)   
  tratto_1_f(:,i)=mtraj*tratto_1_i(:,i);
end

for j=1:columns(tratto_2_i)   
  tratto_2_f(:,j)=mtraj*tratto_2_i(:,j);
end

h = 0.2;    % cambiare la geometria del robot
l1 = 0.8;   % 
l2 = 0.7;   % 
l3 = 0.4;   % 

L = [h l1 l2 l3]';

Qi=[0 0 0 0]';  % Pos iniziale traiettoria
Qf=[pi/4 -pi/4 0.1 pi]'; % pos finale traiettoria

% caratteristiche della legge di moto
T=5;
n=100;
dt=T/(n-1);
tt=0:dt:T;
h1=figure(2); % 3D plot of manipulator
set(h1,'name','3D representation xyz')
hold on
grid on
tit = title('manipulators poses and trajectory');
xlabel('x')
ylabel('y')
zlabel('z')
axis("equal")
%plot3(x,y,z,'g') % if x(t) y(t) z(t) are available plot trajectory 
                  % x,y,z, are n*i arrays
PlotSCARA(Qi,L,'r',2)     % Robot in pos. iniz.
PlotSCARA(Qf,L,'b',2)   % Robot in pos. fin.
legend('Robot in Si',' ',' ','Robot in Sf',' ',' ');

view(27,26) %point of view azimut, elevation

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
    S(:,i)=[M(1,4),M(2,4),M(3,4)];

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

end

figure(2)
hold on
plot3(S(1,:),S(2,:),S(3,:))

% plot traiettoria
##plot3(tratto_1_i(1,:),tratto_1_i(2,:),tratto_1_i(3,:),'o-',"linewidth",2)
##plot3(tratto_2_i(1,:),tratto_2_i(2,:),tratto_2_i(3,:),'o-',"linewidth",2)

plot3(tratto_1_f(1,:),tratto_1_f(2,:),tratto_1_f(3,:),'o-',"linewidth",2)
plot3(tratto_2_f(1,:),tratto_2_f(2,:),tratto_2_f(3,:),'o-',"linewidth",2)
hold off
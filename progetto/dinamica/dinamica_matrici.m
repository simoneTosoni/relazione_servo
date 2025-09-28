clear all
close all
clc

geometria_SCARA

% Posizioni inizio e fine della traiettoria
Qi=[-0.0302 2.5584 0.1500 1.0000]';  % Pos iniziale traiettoria
Qf=[1.2014 1.7384 0.0799 1.0000]'; % pos finale traiettoria


h1=figure(1); % 3D plot of manipulator
set(h1,'name','3D representation xyz')
hold on
grid on
tit = title('manipulators poses and trajectory');
xlabel('x')
ylabel('y')
zlabel('z')
%plot3(x,y,z,'g') % if x(t) y(t) z(t) are available plot trajectory 
                  % x,y,z, are n*i arrays
PlotSCARA(Qi,L,'r',1)     % Robot in pos. iniz.
PlotSCARA(Qf,L,'b',1)   % Robot in pos. fin.
legend('Robot in Si',' ',' ','Robot in Sf',' ',' ');


view(27,26) %point of view azimut, elevation


% inizio cinematica traiettoria giocattolo    
T=0.622;
n=501;
dt=T/(n-1);
tt=0:dt:T;

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
    m01(:,:,i)=M01(Q(:,i),L);
    m12(:,:,i)=M12(Q(:,i),L);
    m23(:,:,i)=M23(Q(:,i),L);
    m34(:,:,i)=M34(Q(:,i),L);
    MM(:,:,i)=m01(:,:,i)*m12(:,:,i)*m23(:,:,i)*m34(:,:,i);
    S(:,i)=[MM(1,4,i),MM(2,4,i),MM(3,4,i)]; % posizioni gripper x,y,z assolute
    Sr(i)=acos(MM(1,1,i));

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
    L120(:,:,i)=(m01(:,:,i)*L121*m01(:,:,i)^-(1));
    m02(:,:,i)=m01(:,:,i)*m12(:,:,i);
    L230(:,:,i)=(m02(:,:,i)*L232*m02(:,:,i)^(-1));
    m03(:,:,i)=m01(:,:,i)*m12(:,:,i)*m23(:,:,i);
    L340(:,:,i)=(m03(:,:,i)*L343*pinv(m03(:,:,i)));
    m04(:,:,i)=m01(:,:,i)*m12(:,:,i)*m23(:,:,i)*m34(:,:,i);

    % calcolo delle velocità x,y,z
    W010(:,:,i)=L010*ad(i);
    W121(:,:,i)=L121*bd(i);
    W232(:,:,i)=L232*sd(i);
    W343(:,:,i)=L343*gd(i);

    W120(:,:,i)=m01(:,:,i)*W121(:,:,i)*m01(:,:,i)^(-1);
    
    W231(:,:,i)=m12(:,:,i)*W232(:,:,i)*pinv(m12(:,:,i));
    W230(:,:,i)=m01(:,:,i)*W231(:,:,i)*pinv(m01(:,:,i));
    
    W342(:,:,i)=m23(:,:,i)*W343(:,:,i)*pinv(m23(:,:,i));
    W341(:,:,i)=m12(:,:,i)*W342(:,:,i)*m12(:,:,i)^(-1);
    W340(:,:,i)=m01(:,:,i)*W341(:,:,i)*m01(:,:,i)^(-1);
    
    W02(:,:,i)=W010(:,:,i)+W120(:,:,i);
    W03(:,:,i)=W010(:,:,i)+W120(:,:,i)+W230(:,:,i);
    W04(:,:,i)=W010(:,:,i)+W120(:,:,i)+W230(:,:,i)+W340(:,:,i);
    
    Sd(:,i)=W04(:,:,i)*m04(:,4,i);
    Srd(i)=W04(2,1); % potrebbe essere sbagliata

    % calcolo delle accelerazioni del gripper 
    H010(:,:,i)=L010*add(i)+L010^(2)*ad(i)^2;
    H121(:,:,i)=L121*bdd(i)+L121^(2)*bd(i)^2;
    H232(:,:,i)=L232*sdd(i)+L232^(2)*sd(i)^2;
    H343(:,:,i)=L343*gdd(i)+L343^(2)*sd(i)^2;
    
    H120(:,:,i)=m01(:,:,i)*H121(:,:,i)*m01(:,:,i)^(-1);
    
    H231(:,:,i)=m12(:,:,i)*H232(:,:,i)*pinv(m12(:,:,i));
    H230(:,:,i)=m01(:,:,i)*H231(:,:,i)*pinv(m01(:,:,i));
    
    H342(:,:,i)=m23(:,:,i)*H343(:,:,i)*pinv(m23(:,:,i));
    H341(:,:,i)=m12(:,:,i)*H342(:,:,i)*pinv(m12(:,:,i));
    H340(:,:,i)=m01(:,:,i)*H341(:,:,i)*pinv(m01(:,:,i));
    
    H020(:,:,i)=H010(:,:,i)+H120(:,:,i)+2*W010(:,:,i)*W120(:,:,i);
    H030(:,:,i)=H020(:,:,i)+H230(:,:,i)+2*W02(:,:,i)*W230(:,:,i);
    H040(:,:,i)=H030(:,:,i)+H340(:,:,i)+2*W03(:,:,i)*W340(:,:,i);

    Sdd(:,i)=H040(:,:,i)*m04(:,4,i);
    Srdd(i)=H040(2,1);

    
  m0g1(:,:,i)=[cos(a(i)) -sin(a(i)) 0 g1*cos(a(i));
      sin(a(i)) cos(a(i)) 0 g1*sin(a(i));
      0 0 1 h;
      0 0 0 1]; %OK
      
  m10(:,:,i)=M01(Q(:,1),L)^-1;

  % disp("m01 * m10 risulta = "), disp(m10*M01(Q(:,1),L))

  m1g1=m10(:,:,i)*m0g1(:,:,i);
  % m1g1^-1
  % disp("m1g1 * mg11 risulta = "), disp(m1g1*m1g1^-1)

  J1=m1g1*Jg1*m1g1'; 

  m2g2=[1 0 0 g2-l2;
        0 1 0 0;
        0 0 1 0;
        0 0 0 1]; % potrebbe essere sbagliata
        
  J2=m2g2*Jg2*m2g2';

  m3g3=[1 0 0 g3-l3;
        0 1 0 0;
        0 0 1 0;
        0 0 0 1];
        
  J3=m3g3*Jg3*m3g3'; 

  Hg=[0 0 0 0;0 0 0 0;0 0 0 -9.81;0 0 0 0];

  % CASO CON FORZE ESTERNE SUL GRIPPER NULLE ------ CASO 1
  Fx=0;
  Fy=0;
  Fz=0;
  F=[Fx Fy Fz 0]'; % vedo se usarlo

  phiA=[0 0 0 Fx;
        0 0 0 Fy;
        0 0 0 Fz;
        -Fx -Fy -Fz 0]; % A è un sdr posto nel gripper con origine coincidente al sdr 3 ma parallelo a quello di base
    
    
  J10(:,:,i)=m01(:,:,i)*J1*m01(:,:,i)';
  J20(:,:,i)=m02(:,:,i)*J2*m02(:,:,i)';
  J30(:,:,i)=m03(:,:,i)*J3*m03(:,:,i)';
  
  m0A(:,:,i)=[1 0 0 m03(1,4);0 1 0 m03(2,4);0 0 1 m03(3,4);0 0 0 1];  
  
  phi3(:,:,i)=m0A(:,:,i)*phiA*m0A(:,:,i)'+Hg*J30(:,:,i)-J30(:,:,i)*Hg'-H030(:,:,i)*J30(:,:,i)+J30(:,:,i)*H030(:,:,i)'; % questo sembra giusto ma non so perchè   
  
  phi2(:,:,i)=-H020(:,:,i)*J20(:,:,i)+J20(:,:,i)*H020(:,:,i)'+Hg*J20(:,:,i)-J20(:,:,i)*Hg'+phi3(:,:,i);
  
  phi1(:,:,i)=-H010(:,:,i)*J10(:,:,i)+J10(:,:,i)*H010(:,:,i)'+Hg*J10(:,:,i)-J10(:,:,i)*Hg'+phi2(:,:,i);
  
  cc3(i)=-PseDot(phi3(:,:,i),L230);%torque
  cc2(i)=-PseDot(phi2(:,:,i),L120);
  cc1(i)=-PseDot(phi1(:,:,i),L010);
  
  ww3(i)=-PseDot(phi3(:,:,i),W230(:,:,i));%power
  ww2(i)=-PseDot(phi2(:,:,i),W120(:,:,i));
  ww1(i)=-PseDot(phi1(:,:,i),W010(:,:,i));
  ww4(i)=PseDot(m0A(:,:,i)*phiA*m0A(:,:,i)',W03(:,:,i));
  ww(i)= ww1(i)+ww2(i)+ww3(i)+ww4(i);
  
  % visto che non va un cazzo cerco di calcolare Ep ed Ek
  Ek(i)=trace(0.5*W010(:,:,i)*J10(:,:,i)*W010(:,:,i)')+trace(0.5*W02(:,:,i)*J20(:,:,i)*W02(:,:,i)')+trace(0.5*W03(:,:,i)*J30(:,:,i)*W03(:,:,i)');
  
  Ep(i)=-trace(Hg*J10(:,:,i))-trace(Hg*J20(:,:,i))-trace(Hg*J30(:,:,i));
  
end

figure(1)
hold on
plot3(S(1,:),S(2,:),S(3,:))
hold off

figure(2)
subplot(4,1,1)
plot(tt,a,tt,ad,tt,add)
legend('alpha','ad','add')
grid on
subplot(4,1,2)
plot(tt,b,tt,bd,tt,bdd)
legend('beta','bd','bdd')
grid on
subplot(4,1,3)
plot(tt,s,tt,sd,tt,sdd)
legend('s','sd','sdd')
grid on
subplot(4,1,4)
plot(tt,g,tt,gd,tt,gdd)
legend('gamma','gd','gdd')
grid on

figure(3)
plot(tt,S(1,:),tt,Sd(1,:),tt(1:end-1),diff(S(1,:))./diff(tt(1:end)),tt,Sdd(1,:),tt(1:end-1),diff(Sd(1,:))./diff(tt(1:end)))
grid on
legend('x','xd','xd#','xdd','xdd#')
title('Posizioni x traiettoria')

figure(4)
plot(tt,S(2,:),tt,Sd(2,:),tt(1:end-1),diff(S(2,:))./diff(tt(1:end)),tt,Sdd(2,:),tt(1:end-1),diff(Sd(2,:))./diff(tt(1:end)))
grid on
ylim([-0.2 1.4])
legend('y','yd','yd#','ydd','ydd#')
title('Posizioni y traiettoria')

figure(5)
plot(tt,S(3,:),tt,Sd(3,:),tt(1:end-1),diff(S(3,:))./diff(tt(1:end)),tt,Sdd(3,:),tt(1:end-1),diff(Sd(3,:))./diff(tt(1:end)))
grid on
legend('z','zd','zd#','zdd','zdd#')
title('Posizioni z traiettoria')

figure(6)
subplot(3,1,1)
plot(tt,cc1)
grid on
subplot(3,1,2)
plot(tt,cc2)
grid on
subplot(3,1,3)
plot(tt,cc3)
grid on

figure(7)
plot(tt,ww1,tt,ww2,tt,ww3,tt,ww4,tt,ww)
legend('ww1','ww2','ww3','ww4','ww tot')
grid on

figure(8)
plot(tt,Ek)
legend('Ek')
grid on

figure(9)
plot(tt,Ep)
grid on
legend('Ep')

figure(10)
plot(tt,ww,tt(1:end-1),diff(Ek+Ep)./diff(tt))
grid on 
legend('ww','dEtot/dt')







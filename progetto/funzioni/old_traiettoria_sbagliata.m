% Definizione della traiettoria a forma di t stilizzata del robot SCARA per il progetto di Servo System and Robotics 
% Tosoni Simone, matricola 731405

clear 
close all
clc

scaling_factor=0.05;

figure(1)

% disegno il segmento orizontale
P2=[-2,3,0]';
A=[0.5,3,0]';

delta_s=A-P2;

T=5;
n=100;
dt=T/n;
t=0:dt:T;

for i=1:n
    tt=(i-1)*T/(n-1);
    s(i)=cicloidale(tt,T,0,1);
    P(:,i)=P2+s(i)*delta_s;
end

%P=P*scaling_factor


plot3(P(1,:),P(2,:),P(3,:),'o-',"linewidth",2)
hold on

% Serve segmento di collagamento AB
B=[-1,4,0]';
delta_s=B-A;

T=5;
n=100;
dt=T/n;
t=0:dt:T;

for i=1:n
    tt=(i-1)*T/(n-1);
    s(i)=cicloidale(tt,T,0,1);
    R(:,i)=A+s(i)*delta_s;
end

%R=R*scaling_factor

plot3(R(1,:),R(2,:),R(3,:),'o-',"linewidth",2)

% disegno il segmento verticale
B=[-1,4,0]';
C=[-1,0,0]';

delta_s=C-B;

T=5;
n=100;
dt=T/n;
t=0:dt:T;

for i=1:n
    tt=(i-1)*T/(n-1);
    s(i)=cicloidale(tt,T,0,1);
    Q(:,i)=B+s(i)*delta_s;
end

%Q=Q*scaling_factor

plot3(Q(1,:),Q(2,:),Q(3,:),'o-',"linewidth",2)


% disegno il semicerchio
P3=[1,0,0]';
Pc=[0,0,0]';

delta_s=P3-C;

% T=5;
% n=100;
% dt=T/n;
% t=0:dt:T;

% Robot Trajectory Planning_Chapter-9-part3-2023 per la slide

alpha=0;
raggio=1;

Rot=[cos(alpha) -sin(alpha) 0;sin(alpha) cos(alpha) 0; 0 0 1];

% W(:)=Pc+Rot*(C-Pc) 

for i=1:n
    tt=(i-1)*T/(n-1);
    s(i)=cicloidale(tt,T,0,pi);
    alpha(i) = s(i)/raggio;
    Rot=[cos(alpha(i)) -sin(alpha(i)) 0;sin(alpha(i)) cos(alpha(i)) 0; 0 0 1];
    W(:,i)=Pc+Rot*(C-Pc);

end

%Z=Z*scaling_factor

plot3(W(1,:),W(2,:),W(3,:),'o-',"linewidth",2)

% Rappresento traiettoria nello spazio

hold on
grid on
axis("equal")
view([-30 30])
xlabel('x')
ylabel('y')
zlabel('z')
xlim([-3 2])
ylim([-2 5])
zlim([-1 1])
plot3([-3 2],[0 0],[0 0],'k')
title("Traiettoria assegnata")

% concateno i vettori P,Q,R,W dei vari tratti in una unica traiettoria
trajectory(:,:)=[P(:,:) R(:,:) Q(:,:) W(:,:)];
legend("P","R","Q","W")



% % per routare il piano xy su cui giace la traiettoria uso le matrici di rototraslazione
% 
% mtraj_1=[cos(pi/2) -sin(pi/2) 0 0.5;
%          sin(pi/2) 0 cos(pi/2) 0.4;
%          0 0 1 0;
%          0 0 0 1];
%        
% mtraj_2=[1 0 0 0;
%          0 cos(pi/6) -sin(pi/6) 0;
%          0 sin(pi/6) cos(pi/6) 0;
%          0 0 0 1];
%          
% mtraj=mtraj_1*mtraj_2;         
%     
% for i=1:length(tratto_1_i)   
%   tratto_1_f(:,i)=mtraj*tratto_1_i(:,i);
% end
% 
% for j=1:oriz_col   
%   tratto_2_f(:,j)=mtraj*tratto_2_i(:,j);
% end
% 
% trajectory = [tratto_1_f tratto_2_f]
% 
% geometria_SCARA
% 
% Qi=[0 0 0 0]';  % Pos iniziale traiettoria
% Qf=[pi/4 -pi/4 0.1 pi]'; % pos finale traiettoria
% 
% % caratteristiche della legge di moto
% T=5;
% n=100;
% dt=T/(n-1);
% tt=0:dt:T;
% h1=figure(2); % 3D plot of manipulator
% set(h1,'name','3D representation xyz')
% hold on
% grid on
% tit = title('manipulators poses and trajectory');
% xlabel('x')
% ylabel('y')
% zlabel('z')
% axis("equal")
% %plot3(x,y,z,'g') % if x(t) y(t) z(t) are available plot trajectory 
%                   % x,y,z, are n*i arrays
% PlotSCARA(Qi,L,'r',2)     % Robot in pos. iniz.
% PlotSCARA(Qf,L,'b',2)   % Robot in pos. fin.
% legend('Robot in Si',' ',' ','Robot in Sf',' ',' ');
% 
% view(27,26) %point of view azimut, elevation

% % QUESTA CINEMATIVÌCA DIRETTA NON MI SERVE
% % % definizione della traiettoria ai giunti
% % Q=zeros(4,n);
% % 
% % for i=1:n
% %     t=(i-1)*T/(n-1);
% %     tt(i)=t;
% %     [a(i) ad(i) add(i)]=cicloidale(t,T,Qi(1),Qf(1)-Qi(1));
% %     [b(i) bd(i) bdd(i)]=cicloidale(t,T,Qi(2),Qf(2)-Qi(2));
% %     [s(i) sd(i) sdd(i)]=cicloidale(t,T,Qi(3),Qf(3)-Qi(3));
% %     [g(i) gd(i) gdd(i)]=cicloidale(t,T,Qi(4),Qf(4)-Qi(4));
% % 
% %     % definizione delle matrici M del robot e calcolo delle posizioni della
% %     % traiettoria
% %     Q(:,i)=[a(i) b(i) s(i) g(i)]';
% %     m01=M01(Q(:,i),L);
% %     m12=M12(Q(:,i),L);
% %     m23=M23(Q(:,i),L);
% %     m34=M34(Q(:,i),L);
% %     M=m01*m12*m23*m34;
% %     S(:,i)=[M(1,4),M(2,4),M(3,4)];
% % 
% %     % definizione delle matrici L riferite al loro polo i-1
% %     L010=[0 -1 0 0;
% %           1  0 0 0;
% %           0  0 0 0;
% %           0  0 0 0];
% %     L121=[0 -1 0 0;
% %           1  0 0 0;
% %           0  0 0 0;
% %           0  0 0 0];
% %     L232=[0 0 0 0;
% %           0 0 0 0;
% %           0 0 0 -1;
% %           0 0 0 0];
% %     L343=[0 -1 0 0;
% %           1 0 0 0;
% %           0 0 0 0;
% %           0 0 0 0];
% %           
% %     
% %     % calcolo delle matrici definite rispetto al polo 0 origine del sdr
% %     % mondo
% %     L120=(m01*L121*m01^-(1));
% %     m02=m01*m12;
% %     L230=(m02*L232*m02^(-1));
% %     m03=m01*m12*m23;
% %     L340=(m03*L343*pinv(m03));
% %     m04=m01*m12*m23*m34;
% % 
% %     % calcolo delle velocità x,y,z
% %     W010=L010*ad(i);
% %     W121=L121*bd(i);
% %     W232=L232*sd(i);
% %     W343=L343*gd(i);
% % 
% %     W120=m01*W121*m01^(-1);
% %     
% %     W231=m12*W232*pinv(m12);
% %     W230=m01*W231*pinv(m01);
% %     
% %     W342=m23*W343*pinv(m23);
% %     W341=m12*W342*m12^(-1);
% %     W340=m01*W341*m01^(-1);
% %     
% %     W02=W010+W120;
% %     W03=W010+W120+W230;
% %     W04=W010+W120+W230+W340;
% %     
% %     Sd(:,i)=W04*m04(:,4);
% % 
% %     % calcolo delle accelerazioni del gripper 
% %     H010=L010*add(i)+L010^(2)*ad(i)^2;
% %     H121=L121*bdd(i)+L121^(2)*bd(i)^2;
% %     H232=L232*sdd(i)+L232^(2)*sd(i)^2;
% %     H343=L343*gdd(i)+L343^(2)*sd(i)^2;
% %     
% %     H120=m01*H121*m01^(-1);
% %     
% %     H231=m12*H232*pinv(m12);
% %     H230=m01*H231*pinv(m01);
% %     
% %     H342=m23*H343*pinv(m23);
% %     H341=m12*H342*pinv(m12);
% %     H340=m01*H341*pinv(m01);
% %     
% %     H020=H010+H120+2*W010*W120;
% %     H030=H020+H230+2*W02*W230;
% %     H040=H030+H340+2*W03*W340;
% % 
% %     Sdd(:,i)=H040*m04(:,4);
% % 
% % end
% 
% % CINEMATICA INVERSA SUI PUNTI DELLA TRAIETTORIA DEFINITI FINO AD ORA
% 
% 
% 
% % for i=1:length(tratto_1_f)-1
% % 
% %     Si=tratto_1_f(:,i)
% %     Sf=tratto_1_f(:,i+1)
% % 
% %     if (i == 1)
% %         Qi0=[-0.1 2*pi/3 0 0]';
% %         Qf0=[+0.1 2*pi/3 0 0]';
% % 
% %         [Qi(:,i) val]=SCARAinv(Si,L,Qi0,1e-6,55)
% %         [Qf(:,i) val]=SCARAinv(Sf,L,Qf0,1e-6,55)
% %         disp("primo giro")
% %     else
% %         disp(i)
% %         [Qi(:,i) val]=SCARAinv(Si,L,Qi(:,i-1),1e-6,55)
% %         [Qf(:,i) val]=SCARAinv(Sf,L,Qi(:,i-1),1e-6,55)
% %         disp(i+" giro")
% %     end
% %     
% %     T=1;
% %     n=100;
% %     dt=T/(n-1);
% %     tt=0:dt:T;
% %     Q0=Qi(:,i);
% %     
% %     for i=1:n
% %         t=(i-1)*T/(n-1);
% %         tt(i)=t;
% %         [x(i) xd(i) xdd(i)]=cicloidale(t,T,Si(1),Sf(1)-Si(1));
% %         [y(i) yd(i) ydd(i)]=cicloidale(t,T,Si(2),Sf(2)-Si(2));
% %         [z(i) zd(i) zdd(i)]=cicloidale(t,T,Si(3),Sf(3)-Si(3));
% %         [g(i) gd(i) gdd(i)]=cicloidale(t,T,Si(4),Sf(4)-Si(4));
% %         
% %         S(:,i)=[x(i) y(i) z(i) g(i)]';
% %         Sd(:,i)=[xd(i) yd(i) zd(i) gd(i)]';
% %         Sdd(:,i)=[xdd(i) ydd(i) zdd(i) gdd(i)]';
% %         
% %         QQ(:,i)=SCARAinv(S(:,i),L,Q0,1e-6,55);
% %         J=SCARAjac(QQ(:,i),L);
% %         QQd(:,i)=pinv(J)*Sd(:,i);
% %         Jp=SCARAjacP(QQ(:,i),QQd(:,i),L);
% %         QQdd(:,i)=J^(-1)*(Sdd(:,i)-Jp*QQd(:,i));
% %         Q0=QQ(:,i);
% %         
% %     end
% % end
% % % M=diag([0.5 0.5 0.5 10 5 5 5 7 7 7 0.45 9 9 9 1]);
% % 
% % figure(3)
% % plot(tt,S(1,:),tt,S(2,:),tt,S(3,:))
% % grid on
% % ylim([-1 1.2])
% % legend('x [m]','y [m]','z [m]','Location','best')
% % title('Coordinate posizione gripper')
% % 
% % figure(4)
% % plot(tt,Sd(1,:),tt,Sd(2,:),tt,Sd(3,:),tt(1:end-1),diff(S(1,:))./diff(tt(1:end)),tt(1:end-1),diff(S(2,:))./diff(tt(1:end)),tt(1:end-1),diff(S(3,:))./diff(tt(1:end)),[tt(1) tt(end)],[0 0],'k')
% % grid on
% % legend('xd [m/s]','yd [m/s]','zd [m/s]','xd#','yd#','zd#','Location','best')
% % title('Velocità gripper')
% % 
% % figure(5)
% % plot(tt,Sdd(1,:),tt,Sdd(2,:),tt,Sdd(3,:),tt(1:end-1),diff(Sd(1,:))./diff(tt(1:end)),tt(1:end-1),diff(Sd(2,:))./diff(tt(1:end)),tt(1:end-1),diff(Sd(3,:))./diff(tt(1:end)),[tt(1) tt(end)],[0 0],'k')
% % grid on
% % legend("xdd [m/s^2]",'ydd [m/s^2]','zdd [m/s^2]','xdd#','ydd#','zdd#',"Location",'best')
% % title('Accelerazioni gripper')
% % 
% % figure(6)
% % subplot(3,1,1,"align")
% % plot(tt,S(4,:),[tt(1) tt(end)],[0 0],'k')
% % grid on
% % legend('g [rad]',"Location",'best')
% % title('Orientamento gripper')
% % subplot(3,1,2,"align")
% % plot(tt,Sd(4,:),tt(1:end-1),diff(S(4,:))./diff(tt(1:end)),[tt(1) tt(end)],[0 0],'k')
% % grid on
% % legend('gd [rad/s]','gd# ')
% % title('Velocità orientamento gripper')
% % subplot(3,1,3,"align")
% % plot(tt,Sdd(4,:),tt(1:end-1),diff(Sd(4,:))./diff(tt(1:end)),[tt(1) tt(end)],[0 0],'k')
% % grid on
% % legend('gdd [rad/s^2]','gdd# ',"Location",'best')
% % title("Accelerazione orientamento gripper");
% % 
% % figure(7)
% % plot(tt,QQ(1,:),tt,QQ(2,:),tt,QQ(4,:))
% % grid on
% % legend('j1 [rad]','j2 [rad]','g [rad]','Location','best')
% % title('Coordinate giunti rotoidali')
% % 
% % figure(8)
% % plot(tt,QQd(1,:),tt,QQd(2,:),tt,QQd(4,:),tt(1:end-1),diff(QQ(1,:))./diff(tt(1:end)),tt(1:end-1),diff(QQ(2,:))./diff(tt(1:end)),tt(1:end-1),diff(QQ(4,:))./diff(tt(1:end)),[tt(1) tt(end)],[0 0],'k')
% % grid on
% % legend('j1d [rad/s]','j2d [rad/s]','gd [rad/s]','j1d#','j2d#','gd#','Location','best')
% % title('Velocità giunti rotoidali')
% % 
% % figure(9)
% % plot(tt,QQdd(1,:),tt,QQdd(2,:),tt,QQdd(4,:),tt(1:end-1),diff(QQd(1,:))./diff(tt(1:end)),tt(1:end-1),diff(QQd(2,:))./diff(tt(1:end)),tt(1:end-1),diff(QQd(4,:))./diff(tt(1:end)),[tt(1) tt(end)],[0 0],'k')
% % grid on
% % legend("j1dd [rad/s^2]",'j2dd [rad/s^2]','gdd [rad/s^2]','j1dd#','j2dd#','gdd#',"Location",'best')
% % title('Accelerazioni giunti rotoidali')
% % 
% % figure(10)
% % subplot(3,1,1,"align")
% % plot(tt,QQ(3,:),[tt(1) tt(end)],[0 0],'k')
% % grid on
% % legend('s [m]',"Location",'best')
% % title('Posizione giunto prismatico')
% % subplot(3,1,2,"align")
% % plot(tt,QQd(3,:),tt(1:end-1),diff(QQ(3,:))./diff(tt(1:end)),[tt(1) tt(end)],[0 0],'k')
% % grid on
% % legend('sd [m/s]','sd# ')
% % title('Velocità giunto prismatico')
% % subplot(3,1,3,"align")
% % plot(tt,QQdd(3,:),tt(1:end-1),diff(QQd(3,:))./diff(tt(1:end)),[tt(1) tt(end)],[0 0],'k')
% % grid on
% % legend('sdd [m/s^2]','sdd# ',"Location",'best')
% % title("Accelerazione giunto prismatico");
% % 
% % % figure(11)
% % % plot_SCARA_trajectory(Qi,Qf,S(:,:),L,11,[26 23])
% % 
% % 
% % figure(2)
% % hold on
% % plot3(S(1,:),S(2,:),S(3,:))
% % 
% % % plot traiettoria
% % % plot3(tratto_1_i(1,:),tratto_1_i(2,:),tratto_1_i(3,:),'o-',"linewidth",2)
% % % plot3(tratto_2_i(1,:),tratto_2_i(2,:),tratto_2_i(3,:),'o-',"linewidth",2)
% % 
% % plot3(tratto_1_f(1,:),tratto_1_f(2,:),tratto_1_f(3,:),'o-',"linewidth",2)
% % plot3(tratto_2_f(1,:),tratto_2_f(2,:),tratto_2_f(3,:),'o-',"linewidth",2)
% % hold off
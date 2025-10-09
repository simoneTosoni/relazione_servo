    % Geometria robot SCARA per il progetto di Servo System and Robotics 
% Tosoni Simone, matricola 731405

% R R P R robot

% lunghezze link
h = 0.3;    % cambiare la geometria del robot e controllare se serve tutto questo
l1 = 0.55;   % 
l2 = 0.55;   % 
l3 = 0.35;   % 

L = [h l1 l2 l3]';

gh=h/2;
g1=l1/2; % posizione baricentro l1
g2=l2/2; % posizione baricentro l2
g3=l3/2; % posizione baricentro l3
G=[gh g1 g2 g3]; % vettore posizioni centri di massa link

% Masse link
m4 = 2; 
m3 = 5;
m2 = 16;
m1 = 30;

% inerzie link
J1 = 1/12*m1*g1^2;
J2 = 1/12*m2*g2^2;
J3 = 0;
J4 = 0.5; % da controllare


% inerzie dei linkm DA SISTEMARE
% Jx1=0;
% Jy1=0.427;
% Jz1=Jy1;
% 
% Jx2=0;
% Jy2=0.286;
% Jz2=Jy2;
% 
% Jx3=0;
% Jy3=0.0533;
% Jz3=Jy3;

% costruisco le matrici d'inerzia del robot DA SISTEMARE CON SOPRA
Jg1=[0 0 0 0;
    0 0 0 0;
    0 0 J1 0;
    0 0 0 m1];
    
Jg2=[0 0 0 0;
    0 0 0 0;
    0 0 J2 0;
    0 0 0 m2];

Jg3=[0 0 0 0;
    0 0 0 0;
    0 0 0 0;
    0 0 0 m3];
   
Jg4=[0 0 0 0;
    0 0 0 0;
    0 0 0 0;
    0 0 0 m4];

M=diag([m4 m4 m4 J4 m3 m3 m3 m2 m2 m2 J2 m1 m1 m1 J1]);
Fse=[0 0 0 0 0 0 0 0 0 0 0 0 0 0 0]';
Ag=[0 0 9.81 0 0 0 9.81 0 0 9.81 0 0 0 9.81 0]';

% [x_j4;
%  y_j4;
%  z_j4;
%  i_j4;
%  x_j3;
%  y_j3;
%  z_j3;
%  x_j2;
%  y_j2;
%  z_j2;
%  i_j2;
%  x_j1;
%  y_j1;
%  z_j1;
%  i_j1];


% specificare limiti degli angoli ai giunti in qualche modo
% motion range
% J1= 300°
% J2= 300°
% J3= 300mm
% J4= 1440°
% 
% % max speed
% vj1= 440 %°/s
% vj2= 500 %°/s
% vj3= 2800 %mm/s
% vj4= 1700 %°/s

v_max_robot=[440 500 2800 1700]';

a_max_robot=[20 20 100 100]';



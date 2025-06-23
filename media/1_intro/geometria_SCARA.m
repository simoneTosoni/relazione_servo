% Geometria robot SCARA per il progetto di Servo System and Robotics 
% Tosoni Simone, matricola 731405

% R R P R robot

% lunghezze link
h = 0.3;    % cambiare la geometria del robot e controllare se serve tutto questo
l1 = 0.55;   % 
l2 = 0.55;   % 
l3 = 0.3;   % 

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
J1 = 1/3*m1*l1^2;
J2 = 1/3*m2*l2^2;
% J3 = 1/3*m3*R3^2;
J4 = 0.45;

M=diag([m4 m4 m4 0.45 m3 m3 m3 m2 m2 m2 J2 m1 m1 m1 J1]);



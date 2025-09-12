% definisco l'ascissa curvilinea come la somma di tutti i tratti della
% traiettoria definita per punti [x;y;z;1]

P=[-2,3,0,1;
    -1,3,0,1;
    0,3,0,1;
    0.5,3.2,0,1;
    0,3.8,0,1;
   -1,4,0,1;
   -1,2,0,1;
   -1,0,0,1;
   -0.4,-1,0,1
   1,0,0,1]';


figure
for i=1:length(P)
    plot(P(1,i),P(2,i),'-xb')
    hold on
end
grid on
title("Traiettoria vista ortogonale")

% Trasformo i punti nello spazio

scala=[0.1 0.1 0.1 1]';

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

for i=1:length(P)
    nodi_trasformati(:,i)=mtraj*P(:,i).*scala; 
end

% Aggiungo punto P1 inizio e fine della traiettoria 
P1=[0.1 0.3 0.15 1]';
nodi_trasformati=[P1 nodi_trasformati P1];

figure
for i=1:length(nodi_trasformati)
    plot3(nodi_trasformati(1,i),nodi_trasformati(2,i),nodi_trasformati(3,i),'x-b')
    hold on
end
grid on
title("Traiettoria nello spazio")
%close all


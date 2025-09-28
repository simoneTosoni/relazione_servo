% calcolo grandezze cinematiche polso robot nel tratto centrale

q_j4=-pi/4; % gamma finale del giunto 4
Si=SCARAdir([q_j1_1(end);q_j2_1(end);q_j3_1(end);q_j4_1(end)],L); % ignoro le coordinate degli altri assi
Sf=SCARAdir([q_j1_1(end);q_j2_1(end);q_j3_1(end);q_j4],L); % ignoro le coordinate degli altri assi

n_j4=length(Scumulata);
% dt_j4=Tcumulata(end)/(n_j4-1);
% tt_j4=0:dt_j4:Tcumulata(end);

for i=1:n_j4
    tempo_j4=(i-1)*Tcumulata/(n_j4-1);
%     tt(i)=tempo;
    [phi(i), phid(i), phidd(i)]=cicloidale(Tcumulata(i),Tcumulata(end),Si(4),Sf(4)-Si(4));
end


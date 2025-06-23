function [Qn NotOk]= SCARAinv(S,L,Q0,toll,i_max)
% Calcila la cinematica inversa del robot SCARA per via numerica
% col metodo di Newton.
%   
% S=[x y z] posa assoluta del robot
% L=[h l1 l2 l3] dimensioni del robot ( lunghezze link )
% Q0 posa iniziale del robot 
% toll tolleranza del metodo di Newton
% i_max numero max di iterazioni del metodo

iter=0;
%Sv=S;
Qv=Q0;
Sv=SCARAdir(Q0,L);

for i=0:i_max
    J=SCARAjac(Qv,L);
    Qn=Qv+pinv(J)*(S-Sv);
    
    Sn=SCARAdir(Qn,L);
    
    if (norm(S-Sn,Inf)) < toll 
        Qn;
        NotOk=0;
        return 
    end

    if i == i_max
        NotOk=1;
        Qn=zeros(3,1);
        return 
    end

    % aggiorno 
    Qv=Qn;
    Sv=Sn;
end

NotOk=0;

end


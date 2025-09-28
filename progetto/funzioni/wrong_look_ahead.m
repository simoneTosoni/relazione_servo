function [v_s,v_t,s_t,t,v_lim,a_s,a_t] = look_ahead(Si,Sf,v_max,a_max,d_max,v_iniziale,v_finale,n_punti)
% Calcola il profilo di v nel tempo (v_t) e nello spazio (v_t) con
% l'algoritmo look ahead,
% v_lim vettore con i limiti di velocità nell'intervallo

% Si posizione iniziale
% Sf posizione finale
% vmax velocità massima
% amax accelerazione massima
% dmax decelerazione massima
% vi velocità nel punto iniziale
% vf velocità nel punto finale

% PROVA AD USARE LA Scumulata PER TUTTO L'ALGORITMO AL POSTO DI CRREARE QUA
% DENTRO UNO SAZIO FITTIZIO A INTERVALLI CONSTANTI, DOVREBBE COREGGERE IL
% PLOT DELLA VELOCITÀ IN FUNZIONE DELLO SPAZIO

a_s=[];
a_t=[];
s_t=[Si];

s_tot=Sf-Si;

v = sqrt((v_iniziale^2 * d_max + v_finale^2 * a_max + 2 * a_max * d_max * s_tot) / (a_max + d_max));

% Verifica se il profilo è trapezoidale o triangolare
if v < v_max
    % Profilo triangolare (il robot non raggiunge v_max)
    v_max_eff = sqrt((v_iniziale^2 * d_max + v_finale^2 * a_max + 2 * a_max * d_max * s_tot) / (a_max + d_max));
    % Tempo di accelerazione e decelerazione
    t_a = (v_max_eff-v_iniziale) / a_max;
    t_d = (v_max_eff-v_finale) / d_max ; 
    t_c = 0;
    
    s_a = (v_max_eff+v_iniziale) / 2 * t_a;
    s_d = (v_finale+v_max_eff) / 2 * t_d;
    s_c = 0;
else
    % Profilo trapezoidale
    v_max_eff = v_max;
    % Tempo di accelerazione e decelerazione
    t_a = (v_max_eff-v_iniziale) / a_max;
    t_d = (v_max_eff-v_finale) / d_max ; 

    s_a = (v_max_eff+v_iniziale) / 2 * t_a;
    s_d = (v_finale+v_max_eff) / 2 * t_d;

    t_c = (s_tot-s_a-s_d) / v_max_eff; % Tempo a velocità costante
    s_c = v_max_eff * t_c;
end


% Tempo totale
t_tot = t_a + t_c + t_d;

% Creazione di punti per spazio e tempo
s = linspace(0, s_tot, n_punti);
t = linspace(0, t_tot, n_punti);
% v_s = zeros(size(s));
v_t = zeros(size(t));
f=1;

% Calcolo della velocità in funzione dello spazio
for i = 1:length(s)
    if s(i) < s_a
        % Fase di accelerazione
        v_s(i) = sqrt(v_iniziale^2 + 2 * a_max * s(i));
        a_s=[a_s a_max];
    elseif s(i)>s_a && s(i) <= (s_a + s_c)
        % Velocità costante
        v_s(i) = v_max_eff;
        a_s=[a_s 0];
    elseif s(i)>(s_a+s_c) && s(i)<=(s_a+s_c+s_d) && v_max_eff^2 + 2 * d_max * ((s_a+s_c) - s(i))>=0
        % Fase di decelerazione
        v_s(i) = sqrt(v_max_eff^2 + 2 * d_max * ((s_a+s_c) - s(i)))
        a_s=[a_s -d_max];
    else 
        disp("ERRORE CALCOLO v_s");
    end
end

fc=1;
% Calcolo della velocità in funzione del tempo
for i = 1:length(t)
    if t(i) < t_a
        % Fase di accelerazione
        v_t(i) = v_iniziale + a_max * t(i)
        a_t=[a_t a_max];
        if (i==1)
            s_t(i)=Si+v_iniziale*t(i)+0.5*a_max*t(i)^2
        else
            s_t(i)=s_t(i-1)+v_t(i-1)*t(i)+0.5*a_max*t(i)^2
        end
    elseif t(i) < (t_a + t_c) && t(i) > t_a
        % Velocità costante
        v_t(i) = v_max_eff
        a_t=[a_t 0];
        if (fc == 1)
%             s_t(i)=s_a+v_t(i)*(t(i)-t_a)
            pos_c = s_t(end);
            fc = 0;
        end
        s_t(i) = pos_c + v_max_eff * (t(i) - t_a)
    else
        % Fase di decelerazione
        v_t(i) = v_max_eff + d_max * ((t_a + t_c) - t(i));
        if (v_t(i)==v_finale) % controllo se la velocità del traatto successivo è uguale alla velocità dell'ultimo punto di questo tratto
            a_t=[a_t 0];
        else
            a_t=[a_t -d_max]; 
        end
        if (i==1)
            s_t(i) = s_a + s_c + s_d + v_max_eff * ((t_a + t_c + t_d) - t(i)) + 0.5 * d_max * ((t_a + t_c + t_d) - t(i))^2
        else
            s_t(i) = + s_t(i-1) + v_t(i) * ((t_a + t_c + t_d) - t(i)) + 0.5 * d_max * ((t_a + t_c + t_d) - t(i))^2
        end
    end
end

% creo il vettore con le velocità limite da plottare
for i=1:n_punti
    if i == 1
        v_lim(i)=v_iniziale;
    elseif i == n_punti
        v_lim(i)=v_finale;
    else
        v_lim(i)=v_max;
    end
end

% PLOT DI DEBUG
figure
subplot(2,2,1)
plot(t,v_t,"DisplayName","v_t")
grid on
legend()
subplot(2,2,2)
plot(s,v_s,"DisplayName","v_s")
grid on
legend()
subplot(2,2,3)
plot(t,s_t,"DisplayName","s")
grid on
legend()
subplot(2,2,4)
plot(t,a_t,"DisplayName","a_s")
grid on
legend()


end
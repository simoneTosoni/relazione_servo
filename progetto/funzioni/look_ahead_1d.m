clc; clear; close all;

traiettoria_2
close all

% Parametri del sistema
v_max = 1;      % Velocità massima (m/s)
v_iniziale = 0.2;
v_finale = 0;
a_max = 1;      % Accelerazione massima (m/s^2)
d_max = 1;      % Decelerazione massima (m/s^2)
s_tot = S(end);     % Distanza totale del percorso (m)

% Calcolo della velocità massima finale
% s_primo = linspace(0, s_tot, 1000);
% v = 0;
% for i=s_primo
%     if (sqrt(v_iniziale^2 + 2 * a_max * i) - sqrt(v_finale^2 + 2 * d_max * (s_tot - i))) < abs(0.01)
%         v = sqrt(v_iniziale^2 + 2 * a_max * i); % di v in realtà conosco la formula analitica
%     end 
% end

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
s = linspace(0, s_tot, 1000);
t = linspace(0, t_tot, 1000);
v_s = zeros(size(s));
v_t = zeros(size(t));

% Calcolo della velocità in funzione dello spazio
for i = 1:length(s)
    if s(i) < s_a
        % Fase di accelerazione
        v_s(i) = sqrt(v_iniziale^2 + 2 * a_max * s(i));
    elseif s(i) < (s_a + s_c)
        % Velocità costante
        v_s(i) = v_max_eff;
    else
        % Fase di decelerazione
        v_s(i) = sqrt(v_finale^2 + 2 * d_max * (s_tot - s(i)));
    end
end

% Calcolo della velocità in funzione del tempo
for i = 1:length(t)
    if t(i) < t_a
        % Fase di accelerazione
        v_t(i) = v_iniziale + a_max * t(i);
    elseif t(i) < (t_a + t_c)
        % Velocità costante
        v_t(i) = v_max_eff;
    else
        % Fase di decelerazione
        v_t(i) = v_max_eff - d_max * (t(i) - (t_a + t_c));
    end
end

% Creazione dei grafici
figure;

% Velocità vs Spazio
subplot(3,1,1);
plot(s, v_s, 'b', 'LineWidth', 2);
xlabel('Spazio percorso (m)');
ylabel('Velocità (m/s)');
title('Profilo di velocità in funzione dello spazio');
grid on;

% Velocità vs Tempo
subplot(3,1,2);
plot(t, v_t, 'r', 'LineWidth', 2);
xlabel('Tempo (s)');
ylabel('Velocità (m/s)');
title('Profilo di velocità in funzione del tempo');
grid on;

% Velocità vs Tempo
subplot(3,1,3);
plot(t(1:end-1), diff(v_t), 'r', 'LineWidth', 2);
xlabel('Tempo (s)');
ylabel('Velocità (m/s)');
title('Profilo di velocità in funzione del tempo');
grid on;



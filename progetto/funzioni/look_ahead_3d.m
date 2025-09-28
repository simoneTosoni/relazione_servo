clc; clear; close all;

traiettoria_2
close all

% Parametri del sistema
v_max = [2;1;1];      % Velocità massima (m/s)
v_iniziale = [0;0;0];
v_finale = [0;0;0];
a_max = [2;1;1];      % Accelerazione massima (m/s^2)
d_max = [10;10;10];      % Decelerazione massima (m/s^2)
s_tot = S(end);  % Distanza totale del percorso (m)

v = sqrt((v_iniziale.^2 .* d_max + v_finale.^2 .* a_max + 2 .* a_max .* d_max .* s_tot) ./ (a_max + d_max));

% Verifica se il profilo è trapezoidale o triangolare
for i=1:3
    if v(i) < v_max(i)
        % Profilo triangolare (il robot non raggiunge v_max)
        v_max_eff(i) = sqrt((v_iniziale(i)^2 * d_max(i) + v_finale(i)^2 * a_max(i) + 2 * a_max(i) * d_max(i) * s_tot(i)) / (a_max(i) + d_max(i)));
        % Tempo di accelerazione e decelerazione
        t_a(i) = (v_max_eff(i)-v_iniziale(i)) / a_max(i);
        t_d(i) = (v_max_eff(i)-v_finale(i)) / d_max(i); 
        t_c(i) = 0;
        
        s_a(i) = (v_max_eff(i)+v_iniziale(i)) / 2 * t_a(i);
        s_d(i) = (v_finale(i)+v_max_eff(i)) / 2 * t_d(i);
        s_c(i) = 0;
    else
        % Profilo trapezoidale
        v_max_eff(i) = v_max(i);
        % Tempo di accelerazione e decelerazione
        t_a(i) = (v_max_eff(i)-v_iniziale(i)) / a_max(i);
        t_d(i) = (v_max_eff(i)-v_finale(i)) / d_max(i) ; 
    
        s_a(i) = (v_max_eff(i)+v_iniziale(i)) / 2 * t_a(i);
        s_d(i) = (v_finale(i)+v_max_eff(i)) / 2 * t_d(i);
    
        t_c(i) = (s_tot-s_a(i)-s_d(i)) / v_max_eff(i); % Tempo a velocità costante
        s_c(i) = v_max_eff(i) * t_c(i);
    end
end


% Tempo totale
t_tot = t_a + t_c + t_d;

% Creazione di punti per spazio e tempo
s_x = linspace(0, s_tot, 1000);
s_y = linspace(0, s_tot, 1000);
s_z = linspace(0, s_tot, 1000);


s = [s_x; s_y; s_z];

v_s = [];

t_x = linspace(0, t_tot(1), 1000);
t_y = linspace(0, t_tot(2), 1000);
t_z = linspace(0, t_tot(3), 1000);

t = [t_x; t_y; t_z];

% v_s = zeros(4,size(s));
% v_t = zeros(4,size(t));

% Calcolo della velocità in funzione dello spazio
for j=1:3
    for i = 1:length(s)
        if s(j,i) < s_a(j)
            % Fase di accelerazione
            v_s(j,i) = sqrt(v_iniziale(j)^2 + 2 * a_max(j) * s(j,i));
        elseif s(j,i) < (s_a(j) + s_c(j))
            % Velocità costante
            v_s(j,i) = v_max_eff(j);
        else
            % Fase di decelerazione
            v_s(j,i) = sqrt(v_finale(j)^2 + 2 * d_max(j) .* (s_tot - s(j,i)));
        end
    end
end


% % Calcolo della velocità in funzione del tempo
for j=1:3
    for i = 1:length(t)
        if t(j,i) < t_a(j)
            % Fase di accelerazione
            v_t(j,i) = v_iniziale(j) + a_max(j) * t(j,i);
        elseif t(j,i) < (t_a(j) + t_c(j))
            % Velocità costante
            v_t(j,i) = v_max_eff(j);
        else
            % Fase di decelerazione
            v_t(j,i) = v_max_eff(j) - d_max(j) * (t(j,i) - (t_a(j) + t_c(j)));
        end
    end
end


%% PLOT DEI GRAFICI

% asse X
figure;

% Velocità vs Spazio
subplot(2,1,1);
plot(s(1,:), v_s(1,:), 'b', 'LineWidth', 2);
xlabel('Spazio percorso (m)');
ylabel('Velocità (m/s)');
title('Profilo di velocità asse x');
grid on;

% Velocità vs Tempo
subplot(2,1,2);
plot(t(1,:), v_t(1,:), 'r', 'LineWidth', 2);
xlabel('Tempo (s)');
ylabel('Velocità (m/s)');
title('Profilo di velocità in funzione del tempo');
grid on;

% asse Y
figure;

% Velocità vs Spazio
subplot(2,1,1);
plot(s(2,:), v_s(2,:), 'b', 'LineWidth', 2);
xlabel('Spazio percorso (m)');
ylabel('Velocità (m/s)');
title('Profilo di velocità asse y');
grid on;

% Velocità vs Tempo
subplot(2,1,2);
plot(t(2,:), v_t(2,:), 'r', 'LineWidth', 2);
xlabel('Tempo (s)');
ylabel('Velocità (m/s)');
title('Profilo di velocità in funzione del tempo');
grid on;

% asse Z
figure;

% Velocità vs Spazio
subplot(2,1,1);
plot(s(3,:), v_s(3,:), 'b', 'LineWidth', 2);
xlabel('Spazio percorso (m)');
ylabel('Velocità (m/s)');
title('Profilo di velocità asse z');
grid on;

% Velocità vs Tempo
subplot(2,1,2);
plot(t(3,:), v_t(3,:), 'r', 'LineWidth', 2);
xlabel('Tempo (s)');
ylabel('Velocità (m/s)');
title('Profilo di velocità in funzione del tempo');
grid on;





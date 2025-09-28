function kappa = curvatura3D(x, y, z)
% CURVATURA3D Calcola la curvatura discreta in 3D da punti (x, y, z)
% Input:
%   x, y, z - vettori della traiettoria 3D (uguale lunghezza)
% Output:
%   kappa  - vettore della curvatura in ciascun punto

n = length(x);
kappa = zeros(1, n);

for i = 2:n-1
    % Tre punti consecutivi
    P1 = [x(i-1), y(i-1), z(i-1)];
    P2 = [x(i),   y(i),   z(i)];
    P3 = [x(i+1), y(i+1), z(i+1)];

    % Vettori locali
    v1 = P2 - P1;
    v2 = P3 - P2;
    chord = P3 - P1;

    % Calcolo curvatura
    cross_prod = cross(v2, v1);
    denom = norm(chord) * norm(v1) * norm(v2);

    if denom > 0
        kappa(i) = 2 * norm(cross_prod) / denom;
    else
        kappa(i) = 0;
    end
end

% Estendi bordo con valori vicini
kappa(1) = kappa(2);
kappa(end) = kappa(end-1);
end

function [x_spline, y_spline, z_spline,t_spline] = interpolazione_spline(vettore_nodi, t, n_points)
    
    X=vettore_nodi(1,:);
    Y=vettore_nodi(2,:);
    Z=vettore_nodi(3,:);
    

    % Generazione dei punti spline
    t_spline = linspace(t(1), t(end), n_points);
    x_spline = spline(t, X, t_spline);
    y_spline = spline(t, Y, t_spline);
    z_spline = spline(t, Z, t_spline);

%     pp_x = csape(t, X,'complete');
%     pp_y = csape(t, Y,'complete');
%     pp_z = csape(t, Z,'complete');
% 
%     x_spline = ppval(pp_x, t_spline);   % valuto spline per X
%     y_spline = ppval(pp_y, t_spline);   % valuto spline per Y
%     z_spline = ppval(pp_z, t_spline);   % valuto spline per Z
    
end
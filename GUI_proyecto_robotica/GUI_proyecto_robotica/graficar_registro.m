% graficar_registro.m
% Grafica, para cada articulacion, la q DESEADA (puntos enviados) contra la q
% LEIDA (encoder), en un mismo figure con 3 subplots (q1, q2, q3).
%
% Usa el .mat que graba el boton "Guardar registro..." de GUI_control
% (variables t, qref, qenc, Q_deseada, ...).
%
% Los puntos deseados no tienen tiempo propio: se les asigna el instante en
% que la referencia del firmware pasa por ellos. Como el firmware va en linea
% recta en q de un punto al siguiente, el largo recorrido por la referencia
% (en el espacio articular) dice por que punto deseado va en cada momento.

clear; clc;

%% ---------------- opciones ----------------
archivo     = '';      % '' = elegir con una ventana, o por ej. 'registro_trayectoria.mat'
MOSTRAR_REF = true;    % true = dibuja tambien la referencia que interpola el firmware

%% ---------------- cargar ----------------
if isempty(archivo)
    [f, p] = uigetfile('*.mat', 'Elegi el registro de la trayectoria');
    if isequal(f, 0), return; end
    archivo = fullfile(p, f);
end
S = load(archivo);
faltan = setdiff({'t', 'qref', 'qenc', 'Q_deseada'}, fieldnames(S));
if ~isempty(faltan)
    error('El archivo no es un registro de GUI_control (faltan: %s).', strjoin(faltan, ', '));
end

t    = S.t(:);          % [s]   M lecturas
qref = S.qref;          % [deg] Mx3 referencia del firmware
qenc = S.qenc;          % [deg] Mx3 encoders (q1 = referencia, no tiene encoder)
Qd   = S.Q_deseada;     % [deg] Nx3 puntos enviados

%% ---------------- tiempo de cada punto deseado ----------------
% largo recorrido en q por la referencia y por la trayectoria deseada
sr = [0; cumsum(sqrt(sum(diff(qref).^2, 2)))];
sd = [0; cumsum(sqrt(sum(diff(Qd).^2, 2)))];

% la referencia arranca en el primer punto deseado: se descuenta lo que haya
% de diferencia al principio (ruido o un arranque un poco corrido)
d0 = norm(qref(1,:) - Qd(1,:));
sd = sd + d0;

[sru, iu] = unique(sr, 'first');      % sr no decrece; unique deja valores estrictamente crecientes
tu = t(iu);
if numel(sru) >= 2
    td = interp1(sru, tu, sd, 'linear', NaN);
else
    td = nan(size(sd));               % la referencia nunca se movio
end
% primer punto: el instante en que la referencia arranca
k0 = find(sr > 0, 1);
if ~isempty(k0), td(1) = t(max(k0 - 1, 1)); end
% puntos que la referencia no llego a recorrer (trayectoria abortada) quedan en NaN
if any(isnan(td))
    fprintf('Ojo: %d de %d puntos deseados no se alcanzaron a recorrer (quedan afuera del grafico).\n', ...
        nnz(isnan(td)), numel(td));
end

%% ---------------- grafico ----------------
nom = {'q1  (sin encoder: la leida es la referencia)', 'q2', 'q3'};
fig = figure('Name', ['Registro: ' archivo], 'NumberTitle', 'off', 'Color', 'w');
ax = gobjects(1, 3);
for j = 1:3
    ax(j) = subplot(3, 1, j);
    hold(ax(j), 'on'); grid(ax(j), 'on'); box(ax(j), 'on');

    h = gobjects(0);  leg = {};
    h(end+1) = plot(ax(j), td, Qd(:,j), 'k.-', 'MarkerSize', 8, 'LineWidth', 0.8);
    leg{end+1} = 'deseada (puntos enviados)';
    if MOSTRAR_REF
        h(end+1) = plot(ax(j), t, qref(:,j), '-', 'Color', [0 0.45 0.74]);
        leg{end+1} = 'referencia (firmware)';
    end
    h(end+1) = plot(ax(j), t, qenc(:,j), '-', 'Color', [0.85 0.2 0.1], 'LineWidth', 1.2);
    leg{end+1} = 'leida (encoder)';

    title(ax(j), nom{j});
    ylabel(ax(j), sprintf('q%d [deg]', j));
    legend(ax(j), h, leg, 'Location', 'best');
end
xlabel(ax(3), 't [s]');
linkaxes(ax, 'x');

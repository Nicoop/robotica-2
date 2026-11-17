function ctl = panel_torques(parent, getReg)
% PANEL_TORQUES  Pestana con el torque teorico de la trayectoria que se ejecuta.
%
%   ctl = panel_torques(app.TabTorque, @() app.Reg)
%   ctl.detener()   libera el timer (al cerrar la GUI)
%
% getReg devuelve el registro de GUI_control (struct Reg con t, qref, qenc,
% Qdes, Paradas, Activo). La pestana lo lee cada 0.5 s mientras el robot se
% mueve, calcula los torques con torque_modelo y los grafica:
%   - arriba: torque en cada articulacion (tau_j)
%   - abajo : torque en el eje de cada motor (tau_m, con reductores y acoplamiento)
% Las franjas grises son los tramos con el vaso en el efector.
%
% Carga:  'Siempre con vaso' | 'Sin vaso' | 'Con vaso entre paradas' (toma en la
% parada n, suelta en la parada m; las paradas se numeran en el orden en que el
% robot frena; en una trayectoria por puntos, cada punto es una parada).
%
% Una vez calculados los torques:
%   'Exportar...'           guarda los valores en .xlsx, .csv o .mat
%   'Ver grafico completo'  abre una ventana aparte (q, dq, tau_j y tau_m con el
%                           eje de tiempo enlazado) para hacer zoom, pan, ver
%                           valores y ocultar curvas haciendo clic en la leyenda

g = uigridlayout(parent, [4 1]);
g.RowHeight = {58, '1x', '1x', 40};
g.Padding = [8 8 8 8];  g.RowSpacing = 6;

% ---------------- opciones ----------------
po = uigridlayout(g, [2 6]);
po.RowHeight = {22, 24};  po.Padding = [0 0 0 0];
po.ColumnWidth = {'1.6x', '1x', '1x', '1.4x', '1x', '1x'};
uilabel(po, 'Text', 'Carga');
uilabel(po, 'Text', 'Toma (n de parada)');
uilabel(po, 'Text', 'Suelta (n de parada)');
uilabel(po, 'Text', 'Senal de q');
uilabel(po, 'Text', 'En vivo');
uilabel(po, 'Text', '');
ddCarga = uidropdown(po, 'Items', {'Siempre con vaso', 'Sin vaso', 'Con vaso entre paradas'}, ...
    'Value', 'Siempre con vaso', 'ValueChangedFcn', @(~,~) actualizar(true));
edToma = uieditfield(po, 'numeric', 'Value', 1, 'Limits', [1 1000], 'RoundFractionalValues', 'on', ...
    'ValueChangedFcn', @(~,~) actualizar(true));
edSuelta = uieditfield(po, 'numeric', 'Value', 2, 'Limits', [1 1000], 'RoundFractionalValues', 'on', ...
    'ValueChangedFcn', @(~,~) actualizar(true));
ddSenal = uidropdown(po, 'Items', {'Referencia (qref)', 'Encoder (qenc)'}, 'Value', 'Referencia (qref)', ...
    'ValueChangedFcn', @(~,~) actualizar(true));
chkVivo = uicheckbox(po, 'Text', 'Seguir', 'Value', true);
btn = uibutton(po, 'Text', 'Actualizar', 'ButtonPushedFcn', @(~,~) actualizar(true)); %#ok<NASGU>

% ---------------- graficos ----------------
ax1 = uiaxes(g);  ax2 = uiaxes(g);
col = lines(3);
for a = [ax1 ax2], grid(a, 'on'); hold(a, 'on'); a.FontSize = 9; end
title(ax1, 'Torque en las articulaciones'); ylabel(ax1, '\tau_j [N m]');
title(ax2, 'Torque en el eje de cada motor (con reductores y acoplamiento del codo)');
ylabel(ax2, '\tau_m [N m]'); xlabel(ax2, 't [s]');
for j = 1:3
    l1(j) = plot(ax1, NaN, NaN, '-', 'Color', col(j,:), 'LineWidth', 1.3, 'DisplayName', sprintf('q%d', j)); %#ok<AGROW>
    l2(j) = plot(ax2, NaN, NaN, '-', 'Color', col(j,:), 'LineWidth', 1.3, 'DisplayName', sprintf('motor %d', j)); %#ok<AGROW>
end
legend(ax1, l1, 'Location', 'northeast', 'FontSize', 8);
legend(ax2, l2, 'Location', 'northeast', 'FontSize', 8);

% ---------------- barra inferior: estado + exportar + ventana ----------------
pb = uigridlayout(g, [1 3]);
pb.ColumnWidth = {'1x', 120, 160};  pb.Padding = [0 0 0 0];  pb.ColumnSpacing = 8;
lbl = uilabel(pb, 'Text', 'Esperando datos del registro...', 'WordWrap', 'on', 'FontSize', 11);
btnExp = uibutton(pb, 'Text', 'Exportar...', 'Enable', 'off', ...
    'Tooltip', 'Guardar t, q, dq, ddq, carga, tau_j y tau_m en .xlsx, .csv o .mat', ...
    'ButtonPushedFcn', @(~,~) exportar());
btnVer = uibutton(pb, 'Text', 'Ver grafico completo', 'Enable', 'off', ...
    'Tooltip', 'Abre una ventana aparte con zoom, pan y lectura de valores', ...
    'ButtonPushedFcn', @(~,~) ver_completo());

% ultimo resultado calculado (lo usan Exportar y Ver grafico completo)
Sult = [];  infoUlt = [];

% ---------------- timer ----------------
ultimoN = -1;
tm = timer('ExecutionMode', 'fixedSpacing', 'Period', 0.5, 'BusyMode', 'drop', ...
           'TimerFcn', @(~,~) actualizar(false));
start(tm);
ctl = struct('actualizar', @() actualizar(true), 'detener', @() cerrar(), ...
             'exportar', @() exportar(), 'ver', @() ver_completo());
actualizar(true);

    function actualizar(forzar)
        if ~isvalid(parent), return; end
        if ~forzar && ~chkVivo.Value, return; end
        try
            if ~forzar && parent.Parent.SelectedTab ~= parent, return; end   % solo calcula si la pestana esta a la vista
        catch
        end
        try
            R = getReg();
        catch
            return;
        end
        n = numel(R.t);
        if n < 10
            lbl.Text = 'Esperando datos del registro (ejecuta una trayectoria)...';
            habilitar(false);
            return;
        end
        if ~forzar && n == ultimoN && ~R.Activo, return; end
        ultimoN = n;
        if ddSenal.Value(1) == 'E', q = R.qenc; else, q = R.qref; end
        carga = vector_carga(R, ddCarga.Value, edToma.Value, edSuelta.Value, q);
        try
            S = torque_modelo(R.t, q, carga);
        catch ME
            if strcmp(ME.identifier, 'torque_modelo:sinModelo')
                lbl.Text = ['Falta construir el modelo. Con el robot quieto, ejecuta  torque_modelo  en la ' ...
                            'ventana de comandos (una sola vez, tarda un par de minutos) y apreta Actualizar.'];
            else
                lbl.Text = ['Error al calcular el torque: ' ME.message];
            end
            return;
        end
        if isempty(S.t)
            lbl.Text = 'Muy pocas muestras todavia.';
            return;
        end
        for j = 1:3
            set(l1(j), 'XData', S.t, 'YData', S.tau_j(:,j));
            set(l2(j), 'XData', S.t, 'YData', S.tau_m(:,j));
        end
        sombrear(ax1, S.t, S.carga, S.tau_j);
        sombrear(ax2, S.t, S.carga, S.tau_m);
        mj = max(abs(S.tau_j), [], 1);  mm = max(abs(S.tau_m), [], 1);
        lbl.Text = sprintf(['Maximo |tau| en las articulaciones [N m]: q1 %.3f | q2 %.3f | q3 %.3f      ' ...
            'En los motores: %.3f | %.3f | %.3f      %s'], mj, mm, texto_estado(R, S));

        % guarda el resultado para exportar / ver en grande
        Sult = S;
        infoUlt = struct('senal', ddSenal.Value, 'carga', ddCarga.Value, ...
            'toma', edToma.Value, 'suelta', edSuelta.Value, 'activo', logical(R.Activo), ...
            'momento', datetime('now'));
        habilitar(true);
    end

    function habilitar(on)
        if on, e = 'on'; else, e = 'off'; end
        if isvalid(btnExp), btnExp.Enable = e; end
        if isvalid(btnVer), btnVer.Enable = e; end
    end

    function exportar()
        if isempty(Sult), return; end
        S = Sult;  info = infoUlt;                       % copia: el timer puede recalcular mientras tanto
        fig = ancestor(parent, 'figure');
        nombre = ['torques_' char(datetime('now', 'Format', 'yyyyMMdd_HHmmss'))];
        [f, p] = uiputfile({'*.xlsx', 'Excel (*.xlsx)'; '*.csv', 'Texto separado por comas (*.csv)'; ...
                            '*.mat', 'MATLAB (*.mat)'}, 'Guardar los torques', nombre);
        try, figure(fig); catch, end                     % devuelve el foco a la GUI
        if isequal(f, 0), return; end
        arch = fullfile(p, f);
        [~, ~, ext] = fileparts(arch);
        T  = tabla_torques(S);
        Ti = tabla_info(S, info);
        try
            switch lower(ext)
                case '.mat'
                    torques = T;  resumen = Ti;          %#ok<NASGU>
                    modelo  = S;                         %#ok<NASGU>  % S tal cual (q en rad)
                    save(arch, 'torques', 'resumen', 'modelo', 'info');
                case '.csv'
                    writetable(T, arch);
                    [p2, n2] = fileparts(arch);
                    writetable(Ti, fullfile(p2, [n2 '_info.csv']));
                otherwise                                % .xlsx
                    if isfile(arch), delete(arch); end   % si no, writetable mezcla con lo viejo
                    writetable(T,  arch, 'Sheet', 'torques');
                    writetable(Ti, arch, 'Sheet', 'info');
            end
            lbl.Text = sprintf('Guardado: %s  (%d muestras)', arch, height(T));
        catch ME
            uialert(fig, ME.message, 'No se pudo guardar el archivo');
        end
    end

    function ver_completo()
        if isempty(Sult), return; end
        S = Sult;  info = infoUlt;
        if info.activo, est = ' (registro en curso: es una foto de este momento)'; else, est = ''; end
        f = figure('Name', sprintf('Torques - %s - %s', info.carga, char(info.momento, 'HH:mm:ss')), ...
                   'NumberTitle', 'off', 'Color', 'w', 'Units', 'normalized', 'Position', [0.08 0.06 0.84 0.86]);
        datos = { rad2deg(S.q),   'q [deg]',            'Posicion articular',            'q';
                  rad2deg(S.dq),  'dq [deg/s]',         'Velocidad articular',           'q';
                  S.tau_j,        '\tau_j [N m]',       'Torque en las articulaciones',  'q';
                  S.tau_m,        '\tau_m [N m]',       'Torque en el eje de cada motor', 'motor ' };
        nf = size(datos, 1);
        ax = gobjects(nf, 1);
        for i = 1:nf
            ax(i) = subplot(nf, 1, i, 'Parent', f);
            hold(ax(i), 'on'); grid(ax(i), 'on'); box(ax(i), 'on');
            Y = datos{i, 1};
            h = gobjects(3, 1);
            for j = 1:3
                h(j) = plot(ax(i), S.t, Y(:,j), '-', 'Color', col(j,:), 'LineWidth', 1.2, ...
                            'DisplayName', sprintf('%s%d', datos{i,4}, j));
                etiquetas_datatip(h(j), datos{i,2});
            end
            sombrear(ax(i), S.t, S.carga, Y);
            m = max(abs(Y), [], 1);
            title(ax(i), sprintf('%s   (max |.|: %.3g | %.3g | %.3g)', datos{i,3}, m));
            ylabel(ax(i), datos{i,2});
            legend(ax(i), h, 'Location', 'eastoutside', 'ItemHitFcn', @alternar_curva);
        end
        xlabel(ax(nf), 't [s]');
        linkaxes(ax, 'x');
        xlim(ax(1), [S.t(1) S.t(end)]);
        sgtitle(f, {sprintf('Senal: %s  |  Carga: %s%s', info.senal, info.carga, est), ...
            'Rueda = zoom  |  arrastrar = mover  |  clic en una curva = valor  |  clic en la leyenda = ocultar/mostrar  |  doble clic = volver a ver todo'}, ...
            'FontSize', 9);
    end

    function cerrar()
        try, stop(tm); delete(tm); catch, end
    end
end


function T = tabla_torques(S)
% una fila por muestra; angulos en grados para que coincidan con la GUI
qd = rad2deg(S.q);  vd = rad2deg(S.dq);  ad = rad2deg(S.ddq);
T = table(S.t(:), qd(:,1), qd(:,2), qd(:,3), vd(:,1), vd(:,2), vd(:,3), ad(:,1), ad(:,2), ad(:,3), ...
    double(S.carga(:)), S.tau_j(:,1), S.tau_j(:,2), S.tau_j(:,3), S.tau_m(:,1), S.tau_m(:,2), S.tau_m(:,3), ...
    'VariableNames', {'t_s', 'q1_deg', 'q2_deg', 'q3_deg', 'dq1_degps', 'dq2_degps', 'dq3_degps', ...
    'ddq1_degps2', 'ddq2_degps2', 'ddq3_degps2', 'vaso', ...
    'tau_j1_Nm', 'tau_j2_Nm', 'tau_j3_Nm', 'tau_m1_Nm', 'tau_m2_Nm', 'tau_m3_Nm'});
end

function Ti = tabla_info(S, info)
mj = max(abs(S.tau_j), [], 1);  mm = max(abs(S.tau_m), [], 1);
rj = sqrt(mean(S.tau_j.^2, 1)); rm = sqrt(mean(S.tau_m.^2, 1));
par = {'Fecha'; 'Senal de q'; 'Carga'; 'Toma (parada)'; 'Suelta (parada)'; 'Registro completo'; ...
       'Muestras'; 'Periodo [s]'; 'Duracion [s]'; ...
       'Max |tau_j| q1 q2 q3 [N m]'; 'RMS tau_j q1 q2 q3 [N m]'; ...
       'Max |tau_m| m1 m2 m3 [N m]'; 'RMS tau_m m1 m2 m3 [N m]'};
val = {char(info.momento, 'yyyy-MM-dd HH:mm:ss'); info.senal; info.carga; ...
       sprintf('%d', info.toma); sprintf('%d', info.suelta); mat2str(~info.activo); ...
       sprintf('%d', numel(S.t)); sprintf('%.4f', median(diff(S.t))); sprintf('%.3f', S.t(end) - S.t(1)); ...
       sprintf('%.4f  %.4f  %.4f', mj); sprintf('%.4f  %.4f  %.4f', rj); ...
       sprintf('%.4f  %.4f  %.4f', mm); sprintf('%.4f  %.4f  %.4f', rm)};
Ti = table(par, val, 'VariableNames', {'Parametro', 'Valor'});
end

function etiquetas_datatip(h, unidad)
% datatips con nombres claros (MATLAB R2019a o posterior; si no, deja los de siempre)
try
    h.DataTipTemplate.DataTipRows(1).Label = 't [s]';
    h.DataTipTemplate.DataTipRows(2).Label = [h.DisplayName '  ' strrep(unidad, '\tau', 'tau')];
catch
end
end

function alternar_curva(~, ev)
% clic en la leyenda: oculta / muestra la curva
if strcmp(ev.Peer.Visible, 'on'), ev.Peer.Visible = 'off'; else, ev.Peer.Visible = 'on'; end
end


function c = vector_carga(R, modo, toma, suelta, q)
% true donde el vaso esta en el efector
N = numel(R.t);
switch modo
    case 'Siempre con vaso', c = true(N, 1);
    case 'Sin vaso',         c = false(N, 1);
    otherwise
        c = false(N, 1);
        if ~isfield(R, 'Paradas') || isempty(R.Paradas) || max(toma, suelta) > numel(R.Paradas), return; end
        [tLlega, tSale] = tiempos_parada(R.t, q, R.Qdes, R.Paradas);
        c = R.t >= tSale(toma) & R.t < tLlega(suelta);
end
end

function [tLlega, tSale] = tiempos_parada(t, q, Qdes, idx)
% instante en que la q llega a cada parada y en que se va de ella
tol = 1.0;                                  % [deg]
ns = numel(idx);
tLlega = inf(1, ns);  tSale = inf(1, ns);
k0 = 1;
for s = 1:ns
    d = sqrt(sum((q(k0:end, :) - Qdes(idx(s), :)).^2, 2));
    kl = find(d < tol, 1);
    if isempty(kl), return; end
    kl = kl + k0 - 1;
    tLlega(s) = t(kl);
    ks = find(sqrt(sum((q(kl:end, :) - Qdes(idx(s), :)).^2, 2)) >= tol, 1);
    if isempty(ks), return; end
    ks = ks + kl - 1;
    tSale(s) = t(ks);
    k0 = ks;
end
end

function sombrear(ax, t, c, Y)
% franjas grises donde hay vaso
delete(findall(ax, 'Tag', 'franja_carga'));
yl = [min(Y(:)), max(Y(:))];
if ~all(isfinite(yl)) || diff(yl) < 1e-6, yl = [-0.01 0.01]; end
m = 0.08 * diff(yl);  yl = yl + [-m m];
ax.YLim = yl;
if ~any(c), return; end
d = diff([false; c(:); false]);
ini = find(d == 1);  fin = find(d == -1) - 1;
for k = 1:numel(ini)
    p = patch(ax, t([ini(k) fin(k) fin(k) ini(k)]), yl([1 1 2 2]), [0.5 0.5 0.5], ...
        'FaceAlpha', 0.12, 'EdgeColor', 'none', 'Tag', 'franja_carga', 'HandleVisibility', 'off');
    uistack(p, 'bottom');
end
end

function s = texto_estado(R, S)
if R.Activo, s = 'Registrando...'; else, s = 'Registro completo.'; end
if any(S.carga), s = [s sprintf(' Con vaso el %.0f %% del tiempo.', 100*mean(S.carga))]; end
end
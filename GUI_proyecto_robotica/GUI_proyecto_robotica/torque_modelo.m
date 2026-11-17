function [S, tau_j, tau_m] = torque_modelo(t, qdeg, carga)
% TORQUE_MODELO  Torque teorico (dinamica inversa) a partir de una q(t) medida.
%
% Version para la GUI de dinamica_inversa.m: recibe los angulos tal cual los
% informa el STM32 (grados, convencion de la GUI: q2 = q3 = 0 es el brazo
% vertical) con sus instantes de llegada, calcula velocidad y aceleracion
% suavizando, y devuelve el torque en las articulaciones y en cada motor.
%
%   S = torque_modelo(t, qdeg, carga)
%     t     : N x 1  tiempo de cada muestra [s]
%     qdeg  : N x 3  q1 q2 q3 [deg] (por ej. Reg.qref o Reg.qenc)
%     carga : escalar o N x 1 logico. true = el vaso (199 g) esta en el efector
%   S.t [s], S.q [rad], S.dq [rad/s], S.ddq [rad/s^2] (remuestreados a 40 Hz),
%   S.carga, S.tau_j [N m] (articulaciones), S.tau_m [N m] (eje de cada motor)
%
%   torque_modelo()   sin argumentos: construye (una sola vez) y guarda en
%                     torque_modelo_cache.mat los dos modelos, sin y con vaso.
%                     HACERLO ANTES de usar la GUI: construir el modelo bloquea
%                     MATLAB un par de minutos y no se puede hacer con el robot moviendose.
%
% Los parametros estan en parametros() mas abajo (mismos valores que
% dinamica_inversa.m; si cambias uno, cambialo en los dos; se borra solo la cache).
% Ver tau_m: A, eta y Jm en parametros(). Requiere Symbolic Toolbox solo para
% construir el modelo la primera vez.

if nargin == 0                              % construir y guardar la cache
    modelos(true);
    return
end
mods = modelos(false);
if isempty(mods)
    error('torque_modelo:sinModelo', ...
          'Falta construir el modelo: ejecuta  torque_modelo  una vez (sin argumentos, tarda un par de minutos).');
end

t = t(:);
[t, iu] = unique(t);                       % tiempos crecientes
qdeg = qdeg(iu, :);
N = numel(t);
if isscalar(carga), carga = repmat(logical(carga), N, 1);
else, carga = logical(carga(:)); carga = carga(iu); end
S = struct('t', [], 'q', [], 'dq', [], 'ddq', [], 'carga', [], 'tau_j', [], 'tau_m', []);
if N < 8 || t(end) - t(1) < 0.3, tau_j = []; tau_m = []; return; end

% --- remuestreo uniforme y derivadas suavizadas --------------------------
dt  = 0.025;                               % [s] (el Q llega a ~40 Hz)
sig = 3;                                   % [muestras] ancho del filtro gaussiano (75 ms)
tu  = (t(1):dt:t(end)).';
qu  = interp1(t, deg2rad(qdeg), tu, 'linear');
cu  = interp1(t, double(carga), tu, 'nearest') > 0.5;
qs  = suavizar(qu, sig);
dq  = suavizar(derivar(qs, dt), sig);
ddq = suavizar(derivar(dq, dt), sig);

% --- torque en las articulaciones: tau_j = M ddq + C dq + g --------------
n = numel(tu);
tau_j = zeros(n, 3);
for k = 1:n
    f = mods{1 + cu(k)};
    q = qs(k,:);  v = dq(k,:);
    Mk = f.fM(q(1), q(2), q(3));
    Ck = f.fCdq(q(1), q(2), q(3), v(1), v(2), v(3));
    gk = f.fg(q(1), q(2), q(3));
    tau_j(k,:) = (Mk*ddq(k,:).' + Ck + gk).';
end

% --- torque en el eje de cada motor (reductores, acoplamiento del codo) --
P   = parametros(false);
A   = P.A;  eta = P.eta(:).';  Jm = P.Jm(:).';
tfi = (inv(A).' * tau_j.').';              % torque ideal en cada motor (trabajo virtual)
thd = (A * dq.').';                        % velocidad de cada motor [rad/s]
thdd = (A * ddq.').';                      % aceleracion de cada motor
emp = double((tfi .* thd) >= 0);           % 1: el motor empuja la carga
tau_m = thdd .* Jm + tfi .* (emp ./ eta + (1 - emp) .* eta);

S = struct('t', tu, 'q', qs, 'dq', dq, 'ddq', ddq, 'carga', cu, 'tau_j', tau_j, 'tau_m', tau_m);
end


function y = suavizar(x, sig)
% media movil gaussiana por columnas, con los extremos replicados
m = ceil(3*sig);
k = exp(-0.5*((-m:m)/sig).^2);  k = k / sum(k);
xp = [repmat(x(1,:), m, 1); x; repmat(x(end,:), m, 1)];
y = conv2(xp, k(:), 'valid');
end

function d = derivar(x, dt)
d = zeros(size(x));
for j = 1:size(x, 2), d(:,j) = gradient(x(:,j), dt); end
end


%% ========================================================================
%  CACHE DE LOS MODELOS (sin vaso / con vaso)
%  ========================================================================
function mods = modelos(construir)
% construir = false: solo usa la cache (memoria o disco); si no hay devuelve []
persistent cache
if ~isempty(cache), mods = cache; return; end
archivo = fullfile(fileparts(mfilename('fullpath')), 'torque_modelo_cache.mat');
firma = firma_parametros();
if isfile(archivo)
    try
        D = load(archivo);
        if isfield(D, 'firma') && isequal(D.firma, firma) && isfield(D, 'mods')
            cache = D.mods;  mods = cache;  return;
        end
    catch
    end
end
if ~construir, mods = []; return; end
fprintf('torque_modelo: construyendo los modelos (una sola vez, puede tardar un par de minutos)...\n');
mods = cell(1, 2);
for k = 1:2
    [fM, fCdq, fg] = construir_modelo(k == 2);
    mods{k} = struct('fM', fM, 'fCdq', fCdq, 'fg', fg);
end
try
    save(archivo, 'mods', 'firma');
catch
    warning('torque_modelo:cache', 'No se pudo guardar la cache; se reconstruye en cada sesion.');
end
cache = mods;
end

function f = firma_parametros()
f = [];
for c = [false true]
    P = parametros(c);
    f = [f, P.d(:).', P.a(:).', double(P.alpha(:)).', P.offset(:).', P.m(:).', ...
         reshape(vertcat(P.rC{:}), 1, []), reshape(vertcat(P.I{:}), 1, []), P.g0(:).']; %#ok<AGROW>
end
end


%% ========================================================================
%  PARAMETROS DEL ROBOT  
%  ========================================================================
function P = parametros(con_carga)
% --- Transmisiones: theta_motor = A*q (angulos en el eje de cada motor) ---
% Base  : relacion total 4
% Hombro: relacion total 4
% Codo  : pinon 20 / corona 30 (x1.5) y luego correa polea 21 / polea 63 (x3).
%         Supuesto: motor en el plato; la corona de 30 y la polea de 21 giran libres
%         sobre el eje de q2; la correa va por el eslabon 2 hasta la polea
%         de 63, fija al eslabon 3. Con phi = giro de la polea de 21 respecto del
%         plato: q3 = (phi - q2)/3  ->  theta_m3 = 1.5*phi = 1.5*(q2 + 3*q3).
i_base   = 4;       
i_hombro = 4;
i_cod_g  = 30/20;
i_cod_b  = 63/21;
P.A = [ i_base   0          0
        0        i_hombro   0
        0        i_cod_g    i_cod_g*i_cod_b ];   % fila 3 = acoplamiento q2-q3
P.eta = [1, 1, 0.9];            % rendimiento de cada transmision 
P.Jm  = [5e-6, 5e-6, 5e-6];     % inercia del rotor [kg*m^2] 

% --- Tabla DH (convencion clasica: Rz(theta) Tz(d) Tx(a) Rx(alpha)) ------
d1 = 0.089;          % [m] altura del eje de q2 sobre el origen de la base
d2 = 0;              
d3 = 0;             
a1 = 0.034;          % [m] distancia entre z0 y z1 (a lo largo de x1)
a2 = 0.15;           % [m] distancia entre los ejes de q2 y q3
a3 = 0.15;           % [m] distancia entre el eje de q3 y el extremo del eslabon 3

P.d      = [d1, d2, d3];
P.a      = [a1, a2, a3];
P.alpha  = [-sym(pi)/2, 0, 0];  % simbolico para que cos(pi/2) sea exactamente 0
                                % Con -pi/2: q2 > 0 BAJA el brazo desde la horizontal
                                % (brazo vertical hacia arriba: q2 = -pi/2)
P.offset = [0, -pi/2, 0];       % theta_i = q_i + offset_i. IGUAL que R.offset de robot.m:
                                % en la GUI q2 = 0 es el brazo VERTICAL (theta2 = -pi/2)

% --- Gravedad en el sistema base (z0 hacia arriba) -----------------------
P.g0 = [0; 0; -9.807];

% --- Masas [kg] ----------------------------------------------------------
m_pap_total = 0.594;            
P.m = [0.521 + m_pap_total, 0.188, 0.079];

% --- Eslabon 1 (base): solo importa Izz respecto del eje z0 --------------
% Plato + corona + soporte + buje (archivo CAD, eje por el origen) + 2 PAP.
Izz_resto  = 1.174e-3;          % [kg*m^2]
d_pap      = 0.052;             % [m] distancia de los PAP al eje
I_pap_prop = 2*1.0e-4;          % [kg*m^2] inercia propia estimada de los 2 PAP
Izz_base   = Izz_resto + m_pap_total*d_pap^2 + I_pap_prop;

% --- Centro de masa de cada eslabon, expresado en SU PROPIO frame i ------
% (vector desde el origen del frame i hasta el CM, en ejes del frame i).
% Eslabon 1: CM sobre z0 (no aporta traslacion ni gravedad).
% Eslabones 2 y 3: CM del CAD (origen en el eje de giro, x a lo largo del
% eslabon) menos [a_i; 0; 0], porque el frame i DH queda en el extremo.
P.rC = {[-a1; 0; 0], ...                                    % eslabon 1
        [40.143e-3 - a2; -4.005e-3;  2.649e-3], ...         % eslabon 2
        [49.046e-3 - a3; -3.837e-3; -0.127e-3]};            % eslabon 3

% --- Tensor de inercia respecto del CM, en ejes paralelos al frame i -----
% (Ec. 19-22 del paper). Unidades kg*m^2.
% Eslabon 1: con alpha1 = -pi/2 el eje y1 es paralelo a z0, por eso Izz_base
% va en la posicion (2,2). Eslabones 2 y 3: desde las propiedades fisicas del
% CAD (momentos principales = m*R^2 con los radios de giro, rotados a los
% ejes del CAD). El z del CAD coincide con el z del DH (z1 = -Y de la base).
P.I = { diag([0, Izz_base, 0]), ...                         % eslabon 1
        [ 1.50370e-4   2.83909e-5   1.72323e-5
          2.83909e-5   6.30969e-4  -1.74756e-6
          1.72323e-5  -1.74756e-6   5.78610e-4 ], ...      % eslabon 2
        [ 2.00577e-5   1.09970e-5  -4.82905e-7
          1.09970e-5   2.43748e-4   4.14609e-8
         -4.82905e-7   4.14609e-8   2.47876e-4 ] };        % eslabon 3

% --- Carga en el extremo: vaso con agua (unido rigidamente al eslabon 3) --
if con_carga
    m_c = 0.199;                % [kg] vaso con agua (CAD: physprop.txt)

    %  LONGITUD DESDE EL ULTIMO ESLABON HASTA EL CM DEL VASO 
    L_cm = 0.100;               % [m] distancia desde el origen del frame 3 (extremo del eslabon 3)
                                %     hasta el CM del vaso con agua, medida sobre el eje Y3.
                                %     Positiva si el vaso queda del lado +Y3, negativa si del -Y3.
                                

    r_c = [0; L_cm; 0];         % [m] CM del vaso en el frame 3 (el eje del vaso va sobre Y3)

    % Inercia propia del vaso en su CM, en ejes paralelos al frame 3 (x3 = X del vaso,
    % y3 = eje del vaso, z3 = Z del vaso). Del CAD, con los radios de giro:
    %   K1 = K2 = 27,447 mm (ejes transversales), K3 = 23,593 mm (eje del vaso)
    % El vaso es simetrico respecto de su eje: la orientacion de X y Z no importa.
    K_trans = 27.447e-3;        % [m]
    K_eje   = 23.593e-3;        % [m]
    I_c = m_c * diag([K_trans^2, K_eje^2, K_trans^2]);   % = diag(1,499e-4, 1,108e-4, 1,499e-4)

    m3 = P.m(3);   r3 = P.rC{3};
    mt = m3 + m_c;
    rt = (m3*r3 + m_c*r_c)/mt;              % CM del conjunto eslabon 3 + vaso
    e3 = r3  - rt;
    ec = r_c - rt;
    It = P.I{3} + m3*((e3.'*e3)*eye(3) - e3*e3.') ...
       + I_c    + m_c*((ec.'*ec)*eye(3) - ec*ec.');   % Steiner
    P.m(3) = mt;   P.rC{3} = rt;   P.I{3} = It;
end

% --- Control de que no quede nada sin completar --------------------------
rc  = vertcat(P.rC{:});
II  = vertcat(P.I{:});
chk = [P.d(:); P.a(:); P.m(:); rc(:); II(:); P.A(:); P.eta(:); P.Jm(:)];
if any(isnan(chk))
    error('torque_modelo:parametros', ...
          'Faltan parametros por completar en parametros().');
end
end


%% ========================================================================
%  CONSTRUCCION SIMBOLICA DEL MODELO
%  ========================================================================
function [fM, fCdq, fg, M, Cdq, g] = construir_modelo(con_carga)
n = 3;
P = parametros(con_carga);

syms q1 q2 q3 dq1 dq2 dq3 real
q  = [q1; q2; q3];
dq = [dq1; dq2; dq3];

theta = q + P.offset(:);

M  = sym(zeros(n));
Pot = sym(0);
T0 = sym(eye(4));

for i = 1:n
    % Transformacion homogenea 0 -> i  (Ec. 15)
    T0 = T0 * dh(theta(i), P.d(i), P.a(i), P.alpha(i));
    R  = simplify(T0(1:3,1:3));

    % Velocidad angular en el frame i: S(w_i) = R' * dR/dt   (Ec. 10)
    dR = sym(zeros(3));
    for k = 1:n
        dR = dR + diff(R, q(k)) * dq(k);
    end
    S = simplify(R.' * dR);
    w = [S(3,2); S(1,3); S(2,1)];            % vector axial de S(w)
    JR = jacobian(w, dq);                    % Ec. 25

    % Posicion del CM en la base: [p;1] = T0i*[rC;1]          (Ec. 14)
    pC = T0 * [sym(P.rC{i}); 1];
    pC = simplify(pC(1:3));
    JT = jacobian(pC, q);                    % Ec. 24

    % Matriz de inercia generalizada                          (Ec. 28)
    M = M + P.m(i) * (JT.' * JT) + JR.' * sym(P.I{i}) * JR;

    % Energia potencial                                       (Ec. 17)
    Pot = Pot - P.m(i) * (sym(P.g0).' * pC);
end
M = simplify(M);

% Vector de gravedad                                          (Ec. 45)
g = simplify(jacobian(Pot, q).');

% dM/dq segun la Definicion 2 (Ec. 31): matriz n x (n*n), donde el bloque
% de la columna j de M contiene las derivadas respecto de q1..qn.
dMdq = sym(zeros(n, n*n));
for j = 1:n
    for k = 1:n
        dMdq(:, (j-1)*n + k) = diff(M(:, j), q(k));
    end
end

% Vector de Coriolis/centrifugo                               (Ec. 46)
In  = eye(n);
Cdq = dMdq * kron(In, dq) * dq - 0.5 * (dMdq * kron(dq, In)).' * dq;
Cdq = simplify(Cdq);

% Funciones numericas rapidas
fM   = matlabFunction(M,   'Vars', {q1, q2, q3});
fCdq = matlabFunction(Cdq, 'Vars', {q1, q2, q3, dq1, dq2, dq3});
fg   = matlabFunction(g,   'Vars', {q1, q2, q3});
end


function T = dh(th, d, a, al)
% Transformacion DH clasica: Rz(th) * Tz(d) * Tx(a) * Rx(al)
T = [cos(th), -sin(th)*cos(al),  sin(th)*sin(al), a*cos(th);
     sin(th),  cos(th)*cos(al), -cos(th)*sin(al), a*sin(th);
     0,        sin(al),          cos(al),         d;
     0,        0,                0,               1];
end

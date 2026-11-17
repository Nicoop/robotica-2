function Q = cin_inv_3GDL(R, T, q0, mejor)

% CINEMATICA INVERSA - ROBOT 3 GDL
%
% Entradas:
%   R      -> objeto SerialLink
%   T      -> matriz homogenea con la posicion deseada [x y z] expresada en sistema 0
%   q0     -> postura actual [q1 q2 q3]
%   mejor  -> false: devuelve las 4 soluciones
%             true : devuelve la solucion mas cercana a q0
%
% Salida:
%   Q      -> si mejor=false : matriz 3x4 con todas las soluciones
%             si mejor=true  : vector 3x1 con la mejor solucion


%% Verificar argumentos

if nargin == 3
    mejor = false;
elseif nargin ~= 4
    error('Argumentos incorrectos');
end

%% Guardar offsets

offsets = R.offset;

% Para realizar la cinematica inversa usamos las variables
% articulares correspondientes directamente a la DH.
R.offset = zeros(3,1);


%% Convertir P a vector columna

P = T(1:3,4);

if length(P) ~= 3
    error('P debe ser un vector de posicion [x y z]');
end


d1 = R.links(1).d;
a2 = R.links(2).a;
a3 = R.links(3).a;

q1 = calcular_q1(P);

q21 = calcular_q2(R, q1(1), P);
q22 = calcular_q2(R, q1(2), P);

q311 = calcular_q3(R, q1(1), q21(1), P);
q312 = calcular_q3(R, q1(1), q21(2), P);

q321 = calcular_q3(R, q1(2), q22(1), P);
q322 = calcular_q3(R, q1(2), q22(2), P);


% ACOMODAR LAS 4 SOLUCIONES
%
%        Sol 1       Sol 2       Sol 3       Sol 4
%
% q1     q1(1)       q1(1)       q1(2)       q1(2)
% q2     q21(1)      q21(2)      q22(1)      q22(2)
% q3     q311        q312        q321        q322


qq = zeros(3,4);

qq(:,1) = [q1(1); q21(1); q311];
qq(:,2) = [q1(1); q21(2); q312];
qq(:,3) = [q1(2); q22(1); q321];
qq(:,4) = [q1(2); q22(2); q322];


R.offset = offsets;

qq = qq - offsets(:) * ones(1,4);


if ~mejor
    Q = qq;
    return;
end


q0_col = q0(:);

normas = inf(1,4);
es_valida = true(1,4);

for i = 1:4

    % Verificar limites articulares
    if ~isempty(R.qlim)

        fuera_limite = ...
            qq(:,i) < R.qlim(:,1) | ...
            qq(:,i) > R.qlim(:,2);

        if any(fuera_limite)
            es_valida(i) = false;
        end

    end

    % Distancia respecto de q0
    if es_valida(i)
        normas(i) = norm(qq(:,i) - q0_col);
    end

end


%% Buscar la solucion mas cercana

[min_norma, pos] = min(normas);


%% Si ninguna es valida

if isinf(min_norma)

    warning('cin_inv_3GDL:SinSolucionValida', ...
        ['Ninguna solucion respeta los limites articulares. ', ...
         'Se devuelve la solucion mas cercana sin considerar limites.']);

    diferencias = qq - q0_col * ones(1,4);

    normas_todas = zeros(1,4);

    for i = 1:4
        normas_todas(i) = norm(diferencias(:,i));
    end

    [~, pos] = min(normas_todas);

end


%% Solucion final

Q = qq(:,pos);

end

function q1 = calcular_q1(P)

x0 = P(1);
y0 = P(2);

q1(1) = atan2(y0,x0);

% Segunda configuracion
if q1(1) > 0
    q1(2) = q1(1) - pi;
else
    q1(2) = q1(1) + pi;
end

end


function q2 = calcular_q2(R, q1, P)

T1 = R.links(1).A(q1).double;

p1 = invHomog(T1) * [P;1];

x1 = p1(1);
y1 = p1(2);

r = hypot(x1,y1);

a2 = R.links(2).a;
a3 = R.links(3).a;

B = atan2(y1,x1);

cos_d = (a2^2 + r^2 - a3^2) / (2*a2*r);

cos_d = max(-1,min(1,cos_d));

d = acos(cos_d);

q2(1) = B - d;
q2(2) = B + d;

end


function q3 = calcular_q3(R, q1, q2, P)

T1 = R.links(1).A(q1).double;
T2 = R.links(2).A(q2).double;
T02 = T1 * T2;

p2 = invHomog(T02) * [P;1];

x2 = p2(1);
y2 = p2(2);

q3 = atan2(y2,x2);

end


function iT = invHomog(T)

iT = eye(4);

iT(1:3,1:3) = T(1:3,1:3)';

iT(1:3,4) = ...
    -iT(1:3,1:3) * T(1:3,4);

end
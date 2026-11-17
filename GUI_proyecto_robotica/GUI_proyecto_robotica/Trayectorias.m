classdef Trayectorias < handle
% Trayectorias - Biblioteca de trayectorias articulares (sin interfaz grafica).
%
%   Una biblioteca es una lista de trayectorias; cada trayectoria es una
%   secuencia ORDENADA de puntos con nombre. Cada punto guarda:
%       q   [1x3] coordenadas articulares [deg]  (las mismas que se envian en MOVE)
%       xyz [1x3] posicion cartesiana del TCP [m] (cinematica directa de MATLAB)
%       iman        estado del iman al guardar el punto: 1 = ON, 0 = OFF,
%                   NaN = desconocido (no habia STATE del STM32 o archivo viejo)
%
%   El orden es la posicion en el vector 'puntos' (el campo 'orden' se
%   mantiene sincronizado). Los nombres son unicos (sin distinguir
%   mayusculas): entre trayectorias, y entre los puntos de una trayectoria.
%
%   Persistencia: archivo .mat con la variable 'biblioteca'
%   (version, guardado, trayectorias). Ver guardar / cargar / importar.
%
%   Todos los errores de usuario usan el identificador 'Trayectorias:*' y un
%   mensaje legible para el operador.

    properties (SetAccess = private)
        Lista   = struct('id',{},'nombre',{},'creada',{},'modificada',{},'puntos',{})
        Archivo = ""        % archivo .mat asociado ("" = todavia no guardado)
        Sucio   = false     % hay cambios sin guardar
    end

    properties (Constant)
        MAX_NOMBRE = 40
        VERSION    = 2
    end

    methods
        %% ------------------------- consulta -------------------------
        function n = cantidad(obj)
            n = numel(obj.Lista);
        end

        function c = nombres(obj)
            c = cell(1, numel(obj.Lista));
            for i = 1:numel(obj.Lista), c{i} = obj.Lista(i).nombre; end
        end

        function i = indicePorNombre(obj, nombre)
            i = find(strcmpi(obj.nombres(), char(strtrim(string(nombre)))), 1);
            if isempty(i), i = 0; end
        end

        function c = nombresPuntos(obj, i)
            obj.chkTray(i);
            pts = obj.Lista(i).puntos;
            c = cell(1, numel(pts));
            for k = 1:numel(pts), c{k} = pts(k).nombre; end
        end

        function nombre = nombreUnicoTray(obj, base)
            nombre = Trayectorias.unico(char(base), obj.nombres());
        end

        function nombre = nombreUnicoPunto(obj, i, base)
            obj.chkTray(i);
            nombre = Trayectorias.unico(char(base), obj.nombresPuntos(i));
        end

        %% ----------------------- trayectorias -----------------------
        function i = crear(obj, nombre)
            nombre = Trayectorias.limpiar(nombre, 'La trayectoria');
            obj.chkNombreTray(nombre, 0);
            obj.Lista(end+1) = Trayectorias.nuevaTray(obj.nuevoId(), nombre, ...
                Trayectorias.puntosVacios());
            i = numel(obj.Lista);
            obj.Sucio = true;
        end

        function renombrar(obj, i, nombre)
            obj.chkTray(i);
            nombre = Trayectorias.limpiar(nombre, 'La trayectoria');
            obj.chkNombreTray(nombre, i);
            obj.Lista(i).nombre = nombre;
            obj.tocar(i);
        end

        function eliminar(obj, i)
            obj.chkTray(i);
            obj.Lista(i) = [];
            obj.Sucio = true;
        end

        function j = duplicar(obj, i)
            obj.chkTray(i);
            t = obj.Lista(i);
            nombre = obj.nombreUnicoTray([t.nombre '_copia']);
            pts = t.puntos;
            for k = 1:numel(pts), pts(k).id = obj.nuevoId(); end
            obj.Lista(end+1) = Trayectorias.nuevaTray(obj.nuevoId(), nombre, pts);
            j = numel(obj.Lista);
            obj.Sucio = true;
        end

        %% ------------------------- puntos ---------------------------
        function k = agregarPunto(obj, i, nombre, q, xyz, iman)
            % iman (opcional): 1 = ON, 0 = OFF, [] / omitido = desconocido (NaN)
            if nargin < 6, iman = []; end
            obj.chkTray(i);
            nombre = Trayectorias.limpiar(nombre, 'El punto');
            obj.chkNombrePunto(i, nombre, 0);
            q   = Trayectorias.chkQ(q);
            xyz = Trayectorias.normXYZ(xyz);
            iman = Trayectorias.normIman(iman);
            p = struct('id', obj.nuevoId(), 'nombre', nombre, 'q', q, ...
                       'xyz', xyz, 'iman', iman, 'orden', 0);
            pts = obj.Lista(i).puntos;
            pts(end+1) = p;
            obj.Lista(i).puntos = pts;
            obj.renumerar(i);
            k = numel(pts);
        end

        function eliminarPunto(obj, i, k)
            obj.chkPunto(i, k);
            obj.Lista(i).puntos(k) = [];
            obj.renumerar(i);
        end

        function k2 = duplicarPunto(obj, i, k)
            obj.chkPunto(i, k);
            pts = obj.Lista(i).puntos;
            p = pts(k);
            p.id = obj.nuevoId();
            p.nombre = Trayectorias.unico([p.nombre '_copia'], obj.nombresPuntos(i));
            pts = [pts(1:k), p, pts(k+1:end)];     % justo despues del original
            obj.Lista(i).puntos = pts;
            obj.renumerar(i);
            k2 = k + 1;
        end

        function renombrarPunto(obj, i, k, nombre)
            obj.chkPunto(i, k);
            nombre = Trayectorias.limpiar(nombre, 'El punto');
            obj.chkNombrePunto(i, nombre, k);
            obj.Lista(i).puntos(k).nombre = nombre;
            obj.tocar(i);
        end

        function editarPunto(obj, i, k, q, xyz, iman)
            % Cambia las coordenadas de un punto (no toca nombre ni orden).
            % El estado del iman solo cambia si se pasa 'iman' (6to argumento).
            obj.chkPunto(i, k);
            obj.Lista(i).puntos(k).q   = Trayectorias.chkQ(q);
            obj.Lista(i).puntos(k).xyz = Trayectorias.normXYZ(xyz);
            if nargin >= 6
                obj.Lista(i).puntos(k).iman = Trayectorias.normIman(iman);
            end
            obj.tocar(i);
        end

        function k2 = moverPunto(obj, i, k, destino)
            % Mueve el punto k a la posicion 'destino' (1..n). No altera
            % nombres ni coordenadas, solo el orden. Devuelve la nueva posicion.
            obj.chkPunto(i, k);
            n = numel(obj.Lista(i).puntos);
            destino = min(max(round(destino), 1), n);
            if destino == k, k2 = k; return; end
            idx = 1:n;
            idx(k) = [];
            idx = [idx(1:destino-1), k, idx(destino:end)];
            obj.Lista(i).puntos = obj.Lista(i).puntos(idx);
            obj.renumerar(i);
            k2 = destino;
        end

        %% ------------------------ validacion ------------------------
        function [ok, msg] = validarEjecucion(obj, i, limites)
            % limites: 3x2 [min max] en grados (misma convencion que el firmware)
            ok = false; msg = "";
            if i < 1 || i > numel(obj.Lista)
                msg = "No hay una trayectoria seleccionada."; return;
            end
            pts = obj.Lista(i).puntos;
            if isempty(pts)
                msg = "La trayectoria '" + string(obj.Lista(i).nombre) + "' no tiene puntos."; return;
            end
            for k = 1:numel(pts)
                q = pts(k).q;
                if numel(q) ~= 3 || any(~isfinite(q))
                    msg = "El punto '" + string(pts(k).nombre) + "' tiene coordenadas invalidas."; return;
                end
                bad = find(q < limites(:,1)' | q > limites(:,2)', 1);
                if ~isempty(bad)
                    msg = sprintf("El punto '%s' viola el limite de q%d (%.2f fuera de [%g, %g]).", ...
                        pts(k).nombre, bad, q(bad), limites(bad,1), limites(bad,2));
                    return;
                end
            end
            ok = true;
        end

        %% ------------------------ persistencia ----------------------
        function guardar(obj, archivo)
            if nargin < 2 || strlength(string(archivo)) == 0, archivo = obj.Archivo; end
            if strlength(string(archivo)) == 0
                error('Trayectorias:archivo', 'No hay un archivo asociado: use "Guardar como".');
            end
            biblioteca = struct('version', obj.VERSION, ...
                                'guardado', Trayectorias.ahora(), ...
                                'trayectorias', obj.Lista);       %#ok<NASGU>
            try
                save(char(archivo), 'biblioteca', '-v7');
            catch ME
                error('Trayectorias:archivo', 'No se pudo escribir el archivo: %s', ME.message);
            end
            obj.Archivo = string(archivo);
            obj.Sucio = false;
        end

        function cargar(obj, archivo)
            % Reemplaza la biblioteca por el contenido del archivo (validado)
            obj.Lista = Trayectorias.leerArchivo(archivo);
            obj.Archivo = string(archivo);
            obj.Sucio = false;
        end

        function n = importar(obj, archivo)
            % Agrega las trayectorias de otro archivo (renombra las repetidas)
            nueva = Trayectorias.leerArchivo(archivo);
            for t = 1:numel(nueva)
                tr = nueva(t);
                tr.nombre = obj.nombreUnicoTray(tr.nombre);
                tr.id = obj.nuevoId();
                for k = 1:numel(tr.puntos), tr.puntos(k).id = obj.nuevoId(); end
                obj.Lista(end+1) = tr;
            end
            n = numel(nueva);
            if n > 0, obj.Sucio = true; end
        end

        function exportar(obj, i, archivo)
            % Guarda UNA trayectoria en su propio archivo (sin cambiar Archivo/Sucio)
            obj.chkTray(i);
            biblioteca = struct('version', obj.VERSION, ...
                                'guardado', Trayectorias.ahora(), ...
                                'trayectorias', obj.Lista(i));    %#ok<NASGU>
            try
                save(char(archivo), 'biblioteca', '-v7');
            catch ME
                error('Trayectorias:archivo', 'No se pudo escribir el archivo: %s', ME.message);
            end
        end
    end

    %% ---------------------------------------------------------------
    methods (Access = private)
        function chkTray(obj, i)
            if ~(isnumeric(i) && isscalar(i) && i >= 1 && i <= numel(obj.Lista) && i == fix(i))
                error('Trayectorias:indice', 'Trayectoria inexistente.');
            end
        end

        function chkPunto(obj, i, k)
            obj.chkTray(i);
            n = numel(obj.Lista(i).puntos);
            if ~(isnumeric(k) && isscalar(k) && k >= 1 && k <= n && k == fix(k))
                error('Trayectorias:indice', 'Punto inexistente.');
            end
        end

        function chkNombreTray(obj, nombre, excepto)
            otros = obj.nombres();
            if excepto > 0, otros(excepto) = []; end
            if any(strcmpi(otros, nombre))
                error('Trayectorias:duplicado', 'Ya existe una trayectoria llamada "%s".', nombre);
            end
        end

        function chkNombrePunto(obj, i, nombre, excepto)
            otros = obj.nombresPuntos(i);
            if excepto > 0, otros(excepto) = []; end
            if any(strcmpi(otros, nombre))
                error('Trayectorias:duplicado', ...
                    'Ya existe un punto llamado "%s" en esta trayectoria.', nombre);
            end
        end

        function renumerar(obj, i)
            for k = 1:numel(obj.Lista(i).puntos)
                obj.Lista(i).puntos(k).orden = k;
            end
            obj.tocar(i);
        end

        function tocar(obj, i)
            obj.Lista(i).modificada = Trayectorias.ahora();
            obj.Sucio = true;
        end

        function id = nuevoId(obj)
            % Identificador interno unico (no depende del nombre)
            existentes = strings(0,1);
            for t = 1:numel(obj.Lista)
                existentes(end+1) = string(obj.Lista(t).id);                 %#ok<AGROW>
                for k = 1:numel(obj.Lista(t).puntos)
                    existentes(end+1) = string(obj.Lista(t).puntos(k).id);   %#ok<AGROW>
                end
            end
            id = '';
            while isempty(id) || any(existentes == string(id))
                id = sprintf('%s_%05d', char(datetime('now','Format','yyyyMMddHHmmssSSS')), randi(99999));
            end
        end
    end

    %% ---------------------------------------------------------------
    methods (Static)
        function pts = puntosVacios()
            pts = struct('id',{},'nombre',{},'q',{},'xyz',{},'iman',{},'orden',{});
        end

        function t = nuevaTray(id, nombre, puntos)
            t = struct('id', id, 'nombre', nombre, ...
                       'creada', Trayectorias.ahora(), ...
                       'modificada', Trayectorias.ahora(), ...
                       'puntos', puntos);
        end

        function s = ahora()
            s = char(datetime('now','Format','yyyy-MM-dd HH:mm:ss'));
        end

        function nombre = limpiar(nombre, sujeto)
            nombre = char(strtrim(string(nombre)));
            if isempty(nombre)
                error('Trayectorias:nombre', '%s necesita un nombre.', sujeto);
            end
            if numel(nombre) > Trayectorias.MAX_NOMBRE
                error('Trayectorias:nombre', 'El nombre no puede superar %d caracteres.', ...
                    Trayectorias.MAX_NOMBRE);
            end
        end

        function nombre = unico(base, existentes)
            nombre = base; n = 1;
            while any(strcmpi(existentes, nombre))
                n = n + 1;
                nombre = sprintf('%s%d', base, n);
            end
        end

        function q = chkQ(q)
            if ~(isnumeric(q) && numel(q) == 3 && all(isfinite(q(:))))
                error('Trayectorias:coordenadas', 'q debe ser un vector de 3 valores numericos finitos.');
            end
            q = double(q(:)');
        end

        function iman = normIman(iman)
            % 1/true = ON, 0/false = OFF, vacio o NaN = desconocido (NaN)
            if isempty(iman) || (isnumeric(iman) && isscalar(iman) && isnan(iman))
                iman = NaN; return;
            end
            if ~((isnumeric(iman) || islogical(iman)) && isscalar(iman) && any(iman == [0 1]))
                error('Trayectorias:coordenadas', 'El estado del iman debe ser 0 (OFF), 1 (ON) o vacio.');
            end
            iman = double(iman);
        end

        function s = textoIman(iman)
            % Texto para mostrar en la interfaz
            if isnan(iman),     s = '?';
            elseif iman == 1,   s = 'ON';
            else,               s = 'OFF';
            end
        end

        function xyz = normXYZ(xyz)
            if isempty(xyz), xyz = nan(1,3); end
            if ~(isnumeric(xyz) && numel(xyz) == 3)
                error('Trayectorias:coordenadas', 'xyz debe tener 3 valores.');
            end
            xyz = double(xyz(:)');          % NaN permitido: modelo cinematico no disponible
        end

        function lista = leerArchivo(archivo)
            % Lee y VALIDA un archivo de biblioteca. Devuelve la lista normalizada.
            if ~isfile(char(archivo))
                error('Trayectorias:archivo', 'El archivo no existe: %s', char(archivo));
            end
            try
                S = load(char(archivo));
            catch ME
                error('Trayectorias:archivo', 'No se pudo leer el archivo: %s', ME.message);
            end
            if ~isfield(S, 'biblioteca') || ~isstruct(S.biblioteca) || ...
               ~isfield(S.biblioteca, 'trayectorias')
                error('Trayectorias:archivo', 'El archivo no es una biblioteca de trayectorias valida.');
            end
            raw = S.biblioteca.trayectorias;
            if ~isstruct(raw)
                error('Trayectorias:archivo', 'El contenido de "trayectorias" no es valido.');
            end
            lista = struct('id',{},'nombre',{},'creada',{},'modificada',{},'puntos',{});
            for t = 1:numel(raw)
                r = raw(t);
                if ~all(isfield(r, {'nombre','puntos'}))
                    error('Trayectorias:archivo', 'Trayectoria %d: faltan campos.', t);
                end
                nombre = Trayectorias.limpiar(r.nombre, sprintf('La trayectoria %d', t));
                if any(strcmpi({lista.nombre}, nombre))
                    error('Trayectorias:archivo', 'Nombre de trayectoria repetido: "%s".', nombre);
                end
                pts = Trayectorias.puntosVacios();
                rp = r.puntos;
                if ~isempty(rp) && ~(isstruct(rp) && all(isfield(rp, {'nombre','q'})))
                    error('Trayectorias:archivo', '"%s": puntos invalidos.', nombre);
                end
                for k = 1:numel(rp)
                    pn = Trayectorias.limpiar(rp(k).nombre, sprintf('El punto %d de "%s"', k, nombre));
                    if any(strcmpi({pts.nombre}, pn))
                        error('Trayectorias:archivo', '"%s": punto repetido "%s".', nombre, pn);
                    end
                    q = Trayectorias.chkQ(rp(k).q);
                    xyz = nan(1,3);
                    if isfield(rp, 'xyz'), xyz = Trayectorias.normXYZ(rp(k).xyz); end
                    iman = NaN;                       % archivos viejos: sin dato de iman
                    if isfield(rp, 'iman'), iman = Trayectorias.normIman(rp(k).iman); end
                    pid = sprintf('P%d_%d', t, k);
                    if isfield(rp, 'id') && ~isempty(rp(k).id), pid = char(rp(k).id); end
                    pts(end+1) = struct('id', pid, 'nombre', pn, 'q', q, ...
                                        'xyz', xyz, 'iman', iman, 'orden', k); %#ok<AGROW>
                end
                tid = sprintf('T%d', t);
                if isfield(r, 'id') && ~isempty(r.id), tid = char(r.id); end
                tr = Trayectorias.nuevaTray(tid, nombre, pts);
                if isfield(r, 'creada'),     tr.creada     = char(r.creada);     end
                if isfield(r, 'modificada'), tr.modificada = char(r.modificada); end
                lista(end+1) = tr;                                           %#ok<AGROW>
            end
        end
    end
end

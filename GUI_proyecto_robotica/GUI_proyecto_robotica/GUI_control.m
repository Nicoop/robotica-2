classdef GUI_control < matlab.apps.AppBase
% GUI_control - Supervisor del brazo 3 GDL: control manual, CI / P2P cartesiano,
%               gestor de trayectorias, vista 3D y ejecucion secuencial.
%
% FLUJO DE USO (no cambia)
%   1) Ejecutar robot.m (crea R en el workspace base).
%   2) Abrir GUI_conexion, elegir puerto y pulsar Conectar. Cuando el STM32
%      contesta al "Q" con una linea "st,...", GUI_conexion crea GUI_control y le
%      entrega el puerto con open_gui_control(puerto). GUI_control NO se abre sin conexion.
%   3) Primera vez: poner el robot en la pose de calibracion y "Calibrar (C)".
%      Despues de cada reinicio: "Referenciar (R)". Recien ahi se puede mover (G).
%
% ARCHIVOS: GUI_control.m (este), Trayectorias.m (biblioteca de trayectorias,
%           sin interfaz), cin_inv_3GDL.m, robot.m, GUI_conexion.mlapp (el
%           handshake cambia: ver GUI_conexion_handshake.txt).
%
% CONVENCIONES (iguales a la version que ya funcionaba)
%   * q1..q3 en grados, en coordenadas DH del modelo SerialLink (R). Son las
%     mismas que usa el firmware en G y en Q (el cero lo define la pose en que se
%     calibra con C). No hay mapeo en la frontera serie.
%   * Mover sliders o editar campos SOLO cambia la posicion OBJETIVO en MATLAB.
%     Nada se envia al STM32 sin una accion explicita (Enviar, Mover P2P,
%     Enviar punto, Ejecutar, botones de comandos, consola).
%   * "q informada" = ultimo Q del STM32: posicion medida por el encoder en los
%     ejes que lo tienen (en un eje sin encoder es la posicion comandada).
%     La vista 3D la dibuja en gris, distinta del objetivo.
%
% PROTOCOLO (firmware nuevo, ver app.c)  -  lineas terminadas en \n, texto CSV
%   MATLAB -> STM32 : cada linea lleva checksum "*HH" (XOR de los caracteres
%                     anteriores, en hexa). Comandos:
%                       Q                 estado
%                       G,q1,q2,q3[,v]    ir a pose (P2P, grados, v en grados/s)
%                       J,eje,delta[,v]   mover un eje relativo
%                       M,0|1             liberar / habilitar motores
%                       C,q1,q2,q3        calibrar: el robot esta FISICAMENTE en esa pose
%                       R                 referenciar con la calibracion guardada
%                       L,eje,min,max     limites articulares (graba en flash)
%                       X                 frenar (y borrar falla)
%                       W                 lecturas crudas de los encoders
%   STM32 -> MATLAB : respuestas  ok | ok,<libres> | err,<motivo>[,eje] |
%                       st,<estado>,<motores>,qref1..3,qenc1..3,<libres>,<falla>
%                     avisos asincronicos que empiezan con "!" (!ready, !fault,
%                     !warn, !comm_lost, !stopped, !released, !rx_overflow).
%   Estados: NOREF (hay que calibrar/referenciar), IDLE, RUN, HOLD, FAULT.
%   El firmware NO confirma "llegada" con un evento. El P2P de la ejecucion de
%   trayectorias lo resuelve asi (ver ejec_tick): el "ok" del G + un Q posterior
%   con estado IDLE + q informada cerca del objetivo.
%   Este firmware no tiene iman: HAY_IMAN = false (el soporte queda en el codigo).
%
% PESTANA "TRAYECTORIA CONTINUA"
%   Test de rectas 3D (P1 -> P2), circulos (plano del brazo = solo q2 y q3, o XY/XZ/YZ)
%   o un .mat de puntos densos (Q Nx3 en grados, Nx2 = q2 q3, o XYZ). Velocidad
%   (articular o TCP constante), ida y vuelta y repeticiones. Graficos 2D con la zona
%   alcanzable, los limites articulares y la trayectoria; los puntos que se salen se
%   marcan (naranja = limites, rojo = alcance, violeta = salto) y no se puede ejecutar.
%   Se ejecuta como stream: G al primer punto, S,v,a, P..., E. El firmware tiene
%   cola de 512 tramos y encadena los P sin frenar ("ok,<libres>" a cada P).

    %% ---------------------------------------------------------------
    %  Componentes
    %% ---------------------------------------------------------------
    properties (Access = public)
        UIFigure           matlab.ui.Figure
        TabGroup           matlab.ui.container.TabGroup
        TabControl         matlab.ui.container.Tab
        TabTray            matlab.ui.container.Tab
        LblBarra           matlab.ui.control.Label

        % --- control manual (componentes originales) ---
        NaNLabel_3         matlab.ui.control.Label
        NaNLabel_2         matlab.ui.control.Label
        ConnectLamp        matlab.ui.control.Lamp
        ConnectLampLabel   matlab.ui.control.Label
        NaNLabel           matlab.ui.control.Label
        ESTADOLabel        matlab.ui.control.Label
        Panel              matlab.ui.container.Panel
        cmdEditField       matlab.ui.control.EditField
        cmdEditFieldLabel  matlab.ui.control.Label
        EnviarButton       matlab.ui.control.Button
        q3EditField        matlab.ui.control.NumericEditField
        q2EditField        matlab.ui.control.NumericEditField
        q1EditField        matlab.ui.control.NumericEditField
        q3Slider           matlab.ui.control.Slider
        q3SliderLabel      matlab.ui.control.Label
        q2Slider           matlab.ui.control.Slider
        q2SliderLabel      matlab.ui.control.Label
        q1Slider           matlab.ui.control.Slider
        q1SliderLabel      matlab.ui.control.Label
        PanelXYZ           matlab.ui.container.Panel
        xEditFieldLabel    matlab.ui.control.Label
        yEditFieldLabel    matlab.ui.control.Label
        zEditFieldLabel    matlab.ui.control.Label
        xEditField         matlab.ui.control.NumericEditField
        yEditField         matlab.ui.control.NumericEditField
        zEditField         matlab.ui.control.NumericEditField
        CalcularCIButton   matlab.ui.control.Button
        MoverP2PButton     matlab.ui.control.Button
        IKLabel            matlab.ui.control.Label
        XYZLabel           matlab.ui.control.Label

        % --- control manual (nuevos) ---
        XYZObjLabel        matlab.ui.control.Label
        RestablecerButton  matlab.ui.control.Button
        BtnStop            matlab.ui.control.Button
        PanelCmd           matlab.ui.container.Panel
        BtnMotOn           matlab.ui.control.Button
        BtnMotOff          matlab.ui.control.Button
        BtnRef             matlab.ui.control.Button
        BtnCal             matlab.ui.control.Button
        BtnLim             matlab.ui.control.Button
        LblVel             matlab.ui.control.Label
        VelEditField       matlab.ui.control.NumericEditField

        % --- vista 3D ---
        Panel3D            matlab.ui.container.Panel
        Ax3D               matlab.ui.control.UIAxes
        ChkRecorrido       matlab.ui.control.CheckBox
        BtnVista           matlab.ui.control.Button
        LblLeyenda         matlab.ui.control.Label

        % --- trayectorias: lista y archivo ---
        PanelTrayLista     matlab.ui.container.Panel
        ListaTray          matlab.ui.control.ListBox
        BtnNuevaTray       matlab.ui.control.Button
        BtnRenTray         matlab.ui.control.Button
        BtnDupTray         matlab.ui.control.Button
        BtnDelTray         matlab.ui.control.Button
        BtnAbrir           matlab.ui.control.Button
        BtnGuardar         matlab.ui.control.Button
        BtnGuardarComo     matlab.ui.control.Button
        BtnImportar        matlab.ui.control.Button
        BtnExportar        matlab.ui.control.Button
        LblArchivo         matlab.ui.control.Label

        % --- trayectorias: puntos ---
        PanelPuntos        matlab.ui.container.Panel
        LblTrayNombre      matlab.ui.control.Label
        TablaPuntos        matlab.ui.control.Table
        LblSecuencia       matlab.ui.control.Label
        LblPuntoInfo       matlab.ui.control.Label
        BtnAddPunto        matlab.ui.control.Button
        BtnDupPunto        matlab.ui.control.Button
        BtnDelPunto        matlab.ui.control.Button
        BtnActPunto        matlab.ui.control.Button
        BtnSubir           matlab.ui.control.Button
        BtnBajar           matlab.ui.control.Button
        BtnIrPunto         matlab.ui.control.Button
        BtnEnvPunto        matlab.ui.control.Button

        % --- trayectorias: ejecucion ---
        PanelEjec          matlab.ui.container.Panel
        DdModo             matlab.ui.control.DropDown
        TMinEdit           matlab.ui.control.NumericEditField
        BtnEjecutar        matlab.ui.control.Button
        BtnPausa           matlab.ui.control.Button
        BtnDetener         matlab.ui.control.Button
        LblEjecEstado      matlab.ui.control.Label
        LblProgreso        matlab.ui.control.Label

        % --- trayectoria continua ---
        TabCont            matlab.ui.container.Tab
        TabTorque          matlab.ui.container.Tab
        R1x                matlab.ui.control.NumericEditField
        R1y                matlab.ui.control.NumericEditField
        R1z                matlab.ui.control.NumericEditField
        R2x                matlab.ui.control.NumericEditField
        R2y                matlab.ui.control.NumericEditField
        R2z                matlab.ui.control.NumericEditField
        BtnR1Tcp           matlab.ui.control.Button
        BtnR2Tcp           matlab.ui.control.Button
        RectSep            matlab.ui.control.NumericEditField
        BtnRectGen         matlab.ui.control.Button
        DdContPlano        matlab.ui.control.DropDown
        ContFijoLabel      matlab.ui.control.Label
        ContFijoEdit       matlab.ui.control.NumericEditField
        CircULabel         matlab.ui.control.Label
        CircVLabel         matlab.ui.control.Label
        CircU              matlab.ui.control.NumericEditField
        CircV              matlab.ui.control.NumericEditField
        CircR              matlab.ui.control.NumericEditField
        CircAng0           matlab.ui.control.NumericEditField
        CircVueltas        matlab.ui.control.NumericEditField
        CircSentido        matlab.ui.control.DropDown
        CircSep            matlab.ui.control.NumericEditField
        BtnCircCentro      matlab.ui.control.Button
        BtnCircGen         matlab.ui.control.Button
        DdContContenido    matlab.ui.control.DropDown
        BtnContCargar      matlab.ui.control.Button
        BtnContGuardar     matlab.ui.control.Button
        BtnContLimpiar     matlab.ui.control.Button
        LblContArchivo     matlab.ui.control.Label
        DdContVelModo      matlab.ui.control.DropDown
        ContVel            matlab.ui.control.NumericEditField
        ContAcc            matlab.ui.control.NumericEditField
        ChkContIdaVuelta   matlab.ui.control.CheckBox
        ContRep            matlab.ui.control.NumericEditField
        BtnContEjecutar    matlab.ui.control.Button
        BtnContIrInicio    matlab.ui.control.Button
        BtnContDetener     matlab.ui.control.Button
        LblContEstado      matlab.ui.control.Label
        LblContProg        matlab.ui.control.Label
        ChkFrenarCrit      matlab.ui.control.CheckBox
        ContPausa          matlab.ui.control.NumericEditField
        ContTFreno         matlab.ui.control.NumericEditField
        AxLat              matlab.ui.control.UIAxes
        AxSup              matlab.ui.control.UIAxes
        AxQ23              matlab.ui.control.UIAxes
        AxQt               matlab.ui.control.UIAxes
        LblContInfo        matlab.ui.control.Label
        BtnRegVer          matlab.ui.control.Button
        BtnRegGuardar      matlab.ui.control.Button
        EnsUmbral          matlab.ui.control.NumericEditField
        ChkEnsSuavizar     matlab.ui.control.CheckBox
        BtnEnsGrabar       matlab.ui.control.Button
        BtnEnsParar        matlab.ui.control.Button
        BtnEnsMotOn        matlab.ui.control.Button
        BtnEnsGuardar      matlab.ui.control.Button
        LblEnsInfo         matlab.ui.control.Label
        SuavVentana        matlab.ui.control.NumericEditField
        SuavSep            matlab.ui.control.NumericEditField
        BtnSuavizar        matlab.ui.control.Button
        BtnDeshacer        matlab.ui.control.Button
        LblSuavInfo        matlab.ui.control.Label
        SuavCrit           matlab.ui.control.EditField
        SuavStop           matlab.ui.control.EditField
        BtnEnsParada       matlab.ui.control.Button
        BtnEnsMarcar       matlab.ui.control.Button
    end

    %% ---------------------------------------------------------------
    %  Estado y configuracion
    %% ---------------------------------------------------------------
    properties (Access = private)
        Puerto
        Timer
        SincPendiente = true
        Robot                                   % SerialLink (modelo DH) o []

        % Limites articulares [deg]. El firmware los toma de la flash (comando L,
        % boton "Limites -> STM32") y rechaza con "err,limit,eje" lo que se pase.
        % AJUSTAR a los limites reales medidos. Sentido igual al modelo (robot.m):
        % q2 y q3 positivos = los eslabones BAJAN. Cero (q = 0) = eslabones verticales. Si un valor informado queda fuera de estos rangos,
        % la barra deslizante no lo puede mostrar (queda clavada en el extremo).
        q1_limites = [-180 180];
        q2_limites = [-10 90];
        q3_limites = [-10 130];
        q_limites  = [-180 180; -10 90; -10 130];   % 3x2, se arma en startupFcn

        QActual = [0 0 0]                       % ultima q INFORMADA por el STM32 [deg] (encoder)
        QValida = false                         % QActual es valida (el robot esta referenciado)
        EstadoRobot = ""                        % ultimo estado informado (NOREF, IDLE, RUN, HOLD, FAULT)
        Motores = NaN                           % 1 con torque, 0 sueltos
        FallaCod = 0                            % 0 ninguna, 1 motor trabado, 2 encoder
        NSt = 0                                 % cantidad de lineas "st," recibidas
        UltimoTx = ""                           % ultima linea enviada (con checksum), salvo los Q del polling
        UltimoRx = []                           % tic de la ultima linea recibida
        ImanActual = NaN                        % ultimo estado de iman informado (sin uso: HAY_IMAN = false): 1 ON, 0 OFF, NaN = sin dato

        Lib                                     % Trayectorias (handle)
        TrayIdx = 0                             % trayectoria seleccionada
        PuntoIdx = 0                            % punto seleccionado
        H3 = struct()                           % handles graficos de la vista 3D

        % Estado de la ejecucion de trayectorias (maquina de estados)
        Ejec = struct('Activa', false, 'Fase', "", 'TrayIdx', 0, 'Idx', 0, 'N', 0, ...
                      'Q', zeros(0,3), 'Iman', zeros(0,1), 'ImanObj', NaN, 'Nombres', {{}}, 'T0', [], 'Bucle', false, ...
                      'Ciclo', 1, 'PausaPend', false, 'TMin', 3, 'Ack', false, 'NStAck', 0)

        BtnsConexion = {}                       % requieren puerto conectado y sin ejecucion
        CtrlsManual = {}                        % se bloquean durante la ejecucion
        BtnsEdicion = {}                        % se bloquean durante la ejecucion
        CtrlsCont = {}                          % pestana trayectoria continua (se bloquean al ejecutar)

        % Trayectoria continua: Q [deg] y P (TCP [m]) de los puntos + estado del stream
        H2 = struct()                           % handles de los graficos 2D de la trayectoria continua
        TimerLog = []                           % timer de Q rapido mientras se registra
        TimerEns = []                           % timer de Q rapido mientras se graba a mano
        Ens = struct('Activo', false, 'Q', zeros(0,3), 'T0', [], 'NLect', 0, 'Crit', zeros(1,0), 'Stop', zeros(1,0))   % grabacion a mano
        UltUI = []                              % ultimo refresco de la interfaz durante el registro
        PanelTorque = []                        % pestana de torques estimados (panel_torques.m)
        % Registro de la ultima trayectoria continua (ver reg_*)
        Reg = struct('t', zeros(0,1), 'est', strings(0,1), 'qref', zeros(0,3), 'qenc', zeros(0,3), ...
                     'Qdes', zeros(0,3), 'Pdes', zeros(0,3), 'Desc', "", 'Activo', false, 'T0', [], ...
                     'TStop', inf, 'Fecha', "", 'Paradas', zeros(1,0))
        Cont = struct('Q', zeros(0,3), 'P', zeros(0,3), 'Estado', zeros(0,1), 'Desc', "", 'Activa', false, 'Fase', "", ...
                      'SoloIr', false, 'Lineas', {{}}, 'EsP', false(1,0), 'Idx', 0, 'Inflight', 0, ...
                      'Recibidos', 0, 'AckP', 0, 'Libres', NaN, 'LibresMax', NaN, 'NPuntos', 0, ...
                      'Ack', false, 'NStAck', 0, 'T0', [], 'TOk', [], 'TInicio', [], 'TMax', 30, ...
                      'TEst', 0, 'TRx', 4, 'Ant', [], 'Crit', zeros(1,0), 'Stop', zeros(1,0), ...
                      'Sent', false(1,0), 'EsperaIdle', false, 'Parado', false, 'NStPausa', 0, ...
                      'TPausa', [], 'NParadas', 0, 'ParadaAct', 0)
    end

    properties (Constant, Access = private)
        TIMEOUT_RX = 4          % [s] sin datos del STM32 durante una ejecucion => abortar
        TMAX_PUNTO = 30         % [s] maximo para completar un punto => abortar
        TOL_Q = 2               % [deg] tolerancia q informada vs objetivo para dar el punto por hecho
        HAY_IMAN = false        % el firmware nuevo no tiene comando de iman
        V_P2P_DEFECTO = 10      % [deg/s] velocidad del P2P (velocidad sobre la trayectoria articular)
        Q_CAL = [0 0 0]         % [deg] pose fisica en la que se calibra (comando C) = la de las marcas
        T_IMAN_MIN = 0.5        % [s] espera tras confirmar el cambio de iman (agarre/soltado fisico)
        T_IMAN_MAX = 3          % [s] maximo esperando que el firmware confirme el iman => abortar
        TMIN_DEFECTO = 3        % [s] espera minima por punto (valor inicial del campo)
        ARCHIVO_DEFECTO = 'trayectorias.mat'   % se busca junto a GUI_control.m
        S_UP = char(9650)
        S_DOWN = char(9660)
        S_ARROW = char(8594)
        S_DOT = char(9679)
        S_PLAY = char(9654)
        CONT_VENTANA = 16       % lineas del stream enviadas sin "ok" (buffer RX del STM32 = 1024 bytes)
        CONT_PROF_MAX = 200     % bloques maximos en la cola del planificador (tiene 512)
        CONT_SALTO_MAX = 5      % [deg] salto maximo entre puntos seguidos al generar
        CONT_PUNTOS_MAX = 20000 % maximo de puntos de una trayectoria continua
        CONT_V_ART_MAX = 200    % [deg/s] tope de la velocidad articular pedida (el firmware limita por eje)
        CONT_PPS_MAX = 150      % [puntos/s] por encima de esto el serie a 115200 no da abasto
        SCURVE_S = 0.050        % [s] perfil en S del firmware (SCURVE_MS): alarga cada frenada esto
        CONT_POLL = 0.025       % [s] periodo del Q durante el registro (~40 lecturas/s, ~3 kB/s de vuelta)
        REG_COLA = 1.0          % [s] se sigue registrando despues de terminar (ver como se asienta)
    end

    %% ---------------------------------------------------------------
    %  API publica (la usa GUI_conexion)
    %% ---------------------------------------------------------------
    methods (Access = public)
        function open_gui_control(app, puerto)
            app.Puerto = puerto;
            app.UltimoRx = tic;
            app.is_connected();
            flush(app.Puerto);
            configureCallback(app.Puerto, "terminator", @(src,evt) app.leer_linea(src));

            app.Timer = timer("ExecutionMode","fixedSpacing","Period",0.2, ...
                              "BusyMode","drop","TimerFcn",@(~,~) app.poll_status());
            start(app.Timer);
            app.enviar("Q");
        end
    end

    %% ---------------------------------------------------------------
    %  Utilidades
    %% ---------------------------------------------------------------
    methods (Access = private)
        function avisar(app, msg, tipo)
            % Mensaje en la barra inferior. tipo: 'info' | 'ok' | 'aviso' | 'error'
            if nargin < 3, tipo = 'info'; end
            switch tipo
                case 'ok',    c = [0 0.45 0];
                case 'aviso', c = [0.75 0.45 0];
                case 'error', c = [0.8 0 0];
                otherwise,    c = [0.35 0.35 0.35];
            end
            if ~isempty(app.LblBarra) && isvalid(app.LblBarra)
                app.LblBarra.Text = char(msg);
                app.LblBarra.FontColor = c;
            end
        end

        function alerta(app, titulo, msg)
            app.avisar(msg, 'error');
            try
                uialert(app.UIFigure, char(msg), char(titulo));
            catch
            end
        end

        function b = mkBtn(~, parent, txt, cb, tip, peligro)
            b = uibutton(parent, 'push');
            b.Text = txt;
            b.ButtonPushedFcn = @(~,~) cb();
            if nargin >= 5 && ~isempty(tip), b.Tooltip = tip; end
            if nargin >= 6 && peligro
                b.BackgroundColor = [0.80 0.20 0.17];
                b.FontColor = [1 1 1];
                b.FontWeight = 'bold';
            end
        end

        function s = onoff(~, tf)
            if tf, s = 'on'; else, s = 'off'; end
        end

        function d = carpeta_defecto(~)
            % Carpeta donde reside GUI_control.m (donde se busca/crea trayectorias.mat)
            d = fileparts(which('GUI_control'));
            if isempty(d), d = pwd; end
        end
    end

    %% ---------------------------------------------------------------
    %  Modelo del robot y cinematica
    %% ---------------------------------------------------------------
    methods (Access = private)
        function obtener_robot(app)
            % Usa el R (SerialLink) que crea robot.m en el workspace base.
            if ~evalin('base', "exist('R','var')")
                error("No existe R en el workspace. Ejecuta robot.m antes de abrir la GUI.");
            end
            Rb = evalin('base', 'R');
            try
                app.Robot = copy(Rb);          % copia: no toca el R original
            catch
                app.Robot = Rb;
            end
            app.Robot.qlim = deg2rad(app.q_limites);
        end

        function p = tcp(app, qdh_deg)
            T = double(app.Robot.fkine(deg2rad(qdh_deg(:)')));
            p = T(1:3,4);
        end

        function xyz = fk_xyz(app, q)
            % Cinematica directa para guardar en los puntos. NaN si no hay modelo.
            xyz = nan(1,3);
            if isempty(app.Robot), return; end
            try
                xyz = app.tcp(q)';
            catch
            end
        end

        function [q, ok, msg] = resolver_cin(app, P)
            % Devuelve q (1x3, DH, grados) = solucion de cin_inv_3GDL
            % valida (alcanzable + dentro de limites) mas cercana a QActual.
            q = []; ok = false;
            if isempty(app.Robot)
                msg = "Robot no cargado: ejecuta robot.m y reabri la GUI"; return;
            end
            P = P(:);
            T = [eye(3) P; 0 0 0 1];       % cin_inv_3GDL solo usa la posicion

            try
                Qall = cin_inv_3GDL(app.Robot, T, deg2rad(app.QActual), false);   % 3x4 [rad]
            catch ME
                msg = "Error en CI: " + string(ME.message);
                return;
            end

            lim  = deg2rad(app.q_limites);
            qa   = deg2rad(app.QActual);
            tolP = 1e-3;     % [m]  error de posicion admitido (verifica alcance)
            tolL = 1e-6;     % [rad] tolerancia en limites

            mejor_d = inf;
            alcanzable = false;

            for i = 1:size(Qall,2)
                qi = Qall(:,i)';

                % Llevar cada articulacion a su ventana [lim_min, lim_min+2*pi)
                for j = 1:3
                    lo = lim(j,1) - tolL;
                    qi(j) = lo + mod(qi(j) - lo, 2*pi);
                end

                % Verificar que realmente llega al punto (acos saturado => fuera de alcance)
                p = double(app.Robot.fkine(qi));
                if norm(p(1:3,4) - P) > tolP
                    continue;
                end
                alcanzable = true;

                % Limites articulares
                if any(qi' < lim(:,1) - tolL) || any(qi' > lim(:,2) + tolL)
                    continue;
                end

                d = norm(qi - qa);
                if d < mejor_d
                    mejor_d = d;
                    q = rad2deg(qi);
                end
            end

            if isinf(mejor_d)
                if alcanzable
                    msg = "Punto alcanzable, pero las soluciones violan limites articulares";
                else
                    msg = "Punto fuera del espacio de trabajo";
                end
                q = [];
                return;
            end

            ok = true;
            msg = sprintf("CI OK  q=[%.1f %.1f %.1f]  (solo calculada; no enviada)", q);
        end

        function resolver_y_mostrar(app, mover)
            P = [app.xEditField.Value; app.yEditField.Value; app.zEditField.Value];
            [q, ok, msg] = app.resolver_cin(P);
            app.IKLabel.Text = char(msg);

            if ~ok
                app.IKLabel.FontColor = [0.8 0 0];
                return;
            end
            app.IKLabel.FontColor = [0 0.45 0];
            app.setQObjetivo(q);

            if mover
                app.enviar_move();      % flujo existente: "Mover P2P" = CI + envio explicito
            end
        end
    end

    %% ---------------------------------------------------------------
    %  Posicion objetivo (controles articulares)
    %% ---------------------------------------------------------------
    methods (Access = private)
        function q = qObjetivo(app)
            q = [app.q1EditField.Value app.q2EditField.Value app.q3EditField.Value];
        end

        function setQObjetivo(app, q)
            % Solo actualiza controles, TCP objetivo y vista 3D. NO envia nada.
            q = min(max(q(:)', app.q_limites(:,1)'), app.q_limites(:,2)');
            app.q1EditField.Value = q(1);  app.q1Slider.Value = q(1);
            app.q2EditField.Value = q(2);  app.q2Slider.Value = q(2);
            app.q3EditField.Value = q(3);  app.q3Slider.Value = q(3);
            app.actualizar_objetivo();
        end

        function actualizar_objetivo(app)
            % Cinematica directa de la posicion OBJETIVO (no es una medicion)
            if isempty(app.Robot)
                app.XYZObjLabel.Text = 'TCP objetivo: modelo no cargado (ejecuta robot.m)';
                return;
            end
            try
                p = app.tcp(app.qObjetivo());
                app.XYZObjLabel.Text = sprintf('TCP objetivo (FK) = [%.4f  %.4f  %.4f] m', p);
            catch ME
                app.XYZObjLabel.Text = char("TCP objetivo: error FK (" + string(ME.message) + ")");
            end
            app.actualizar_3d();
        end

        function config_limits_art(app)
            app.q1EditField.Limits = app.q_limites(1,:);
            app.q1Slider.Limits    = app.q_limites(1,:);

            app.q2EditField.Limits = app.q_limites(2,:);
            app.q2Slider.Limits    = app.q_limites(2,:);

            app.q3EditField.Limits = app.q_limites(3,:);
            app.q3Slider.Limits    = app.q_limites(3,:);
        end

        function ok = enviar_move(app)
            % Envio EXPLICITO de la posicion objetivo: P2P  G,q1,q2,q3,v
            ok = false;
            if app.ocupado()
                app.avisar('Hay una trayectoria en ejecucion: detenela antes de enviar movimientos manuales.', 'aviso');
                return;
            end
            if ~app.conectado()
                app.alerta('Sin conexion', 'No hay conexion con el STM32.');
                return;
            end
            if ~app.robot_listo_para_mover(), return; end
            qd = min(max(app.qObjetivo(), app.q_limites(:,1)'), app.q_limites(:,2)');
            ok = app.enviar(sprintf("G,%.3f,%.3f,%.3f,%.2f", qd, app.VelEditField.Value));
        end

        function ok = robot_listo_para_mover(app)
            % Chequeos locales antes de un G: referenciado, sin falla, sin movimiento en curso
            ok = false;
            est = string(app.EstadoRobot);
            if strlength(est) == 0
                app.alerta('Sin estado del robot', 'Todavia no se recibio ningun estado (st) del STM32.');
            elseif est == "NOREF"
                app.alerta('Robot sin referenciar', 'Usa "Referenciar (R)" (o "Calibrar (C)" si es la primera vez) antes de mover.');
            elseif est == "FAULT"
                app.alerta('Falla activa', sprintf('%s Usa "Detener (X)" para borrar la falla.', app.texto_falla()));
            elseif any(est == ["RUN","HOLD"])
                app.alerta('Robot ocupado', 'El robot esta en movimiento: espera a que termine o usa "Detener (X)".');
            else
                ok = true;
            end
        end

        function t = texto_falla(app)
            switch app.FallaCod
                case 1, t = 'Motor trabado (error de seguimiento del encoder).';
                case 2, t = 'Encoder sin respuesta: se perdio la referencia.';
                otherwise, t = 'Falla.';
            end
        end
    end

    %% ---------------------------------------------------------------
    %  Conexion serie y estado
    %% ---------------------------------------------------------------
    methods (Access = private)
        function tf = conectado(app)
            tf = false;
            try
                tf = isa(app.Puerto, "internal.Serialport") && isvalid(app.Puerto);
            catch
            end
        end

        function is_connected(app)
            if app.conectado()
                app.ConnectLamp.Color = [0.49 0.84 0.68];
            else
                app.ConnectLamp.Color = [0.84 0.49 0.49];
            end
            app.actualizar_botones();
        end

        function actualizar_botones(app)
            con  = app.conectado();
            ejec = app.Ejec.Activa;
            ocup = app.ocupado();                          % trayectoria por puntos o continua
            for k = 1:numel(app.BtnsConexion)
                app.BtnsConexion{k}.Enable = app.onoff(con && ~ocup);
            end
            for k = 1:numel(app.CtrlsManual)
                app.CtrlsManual{k}.Enable = app.onoff(~ocup);
            end
            for k = 1:numel(app.BtnsEdicion)
                app.BtnsEdicion{k}.Enable = app.onoff(~ocup);
            end
            for k = 1:numel(app.CtrlsCont)
                app.CtrlsCont{k}.Enable = app.onoff(~ocup);
            end
            hayCont = ~isempty(app.Cont.Q);
            app.BtnContEjecutar.Enable = app.onoff(con && ~ocup && hayCont);
            app.BtnContIrInicio.Enable = app.onoff(con && ~ocup && hayCont);
            app.BtnContGuardar.Enable  = app.onoff(hayCont);
            app.BtnContDetener.Enable  = app.onoff(app.Cont.Activa);
            hayReg = ~isempty(app.Reg.t) && ~app.Reg.Activo;
            app.BtnEnsGrabar.Enable  = app.onoff(con && ~ocup);
            app.BtnEnsParar.Enable   = app.onoff(app.Ens.Activo);
            app.BtnEnsMarcar.Enable  = app.onoff(app.Ens.Activo);
            app.BtnEnsParada.Enable  = app.onoff(app.Ens.Activo);
            app.BtnEnsMotOn.Enable   = app.onoff(con && ~ocup);
            app.BtnEnsGuardar.Enable = app.onoff(hayCont && ~ocup);
            app.BtnSuavizar.Enable   = app.onoff(hayCont && ~ocup);
            app.BtnDeshacer.Enable   = app.onoff(~ocup && isfield(app.Cont, 'Ant') && ~isempty(app.Cont.Ant));
            app.BtnRegVer.Enable     = app.onoff(hayReg);
            app.BtnRegGuardar.Enable = app.onoff(hayReg);
            app.BtnStop.Enable     = app.onoff(con);      % X sirve siempre que haya conexion
            app.BtnEjecutar.Enable = app.onoff(con && ~ocup);
            app.BtnPausa.Enable    = app.onoff(ejec);
            app.BtnDetener.Enable  = app.onoff(ejec);
            if ejec && (app.Ejec.Fase == "pausada" || app.Ejec.PausaPend)
                app.BtnPausa.Text = 'Reanudar';
            else
                app.BtnPausa.Text = 'Pausar';
            end
        end

        function l = con_cs(~, cmd)
            % Agrega el checksum "*HH": XOR de todos los caracteres anteriores
            cmd = upper(char(cmd));          % todo lo que se transmite va en MAYUSCULAS
            c = uint8(cmd);
            cs = uint8(0);
            for k = 1:numel(c), cs = bitxor(cs, c(k)); end
            l = sprintf('%s*%02X', cmd, cs);
        end

        function ok = enviar(app, cmd)
            ok = false;
            try
                lin = app.con_cs(strtrim(string(cmd)));
                writeline(app.Puerto, lin);
                app.UltimoTx = string(lin);
                app.NaNLabel_2.Text = char("TX: " + string(lin));
                ok = true;
            catch ME
                app.NaNLabel_3.Text = char("Error serie: " + string(ME.message));
                app.ConnectLamp.Color = [0.84 0.49 0.49];
                app.avisar("Error de comunicacion serie: " + string(ME.message), 'error');
                if app.Ejec.Activa
                    app.ejec_finalizar("Se perdio la comunicacion serie durante la ejecucion.", 'error', false);
                end
                if app.Cont.Activa
                    app.cont_finalizar("Se perdio la comunicacion serie durante la trayectoria continua.", 'error', false);
                end
            end
        end

        function poll_status(app)
            if ~isvalid(app) || isempty(app.UIFigure) || ~isvalid(app.UIFigure), return; end
            try
                ok = true;
                try
                    writeline(app.Puerto, app.con_cs("Q"));
                catch
                    ok = false;
                end
                if ~ok
                    app.ConnectLamp.Color = [0.84 0.49 0.49];
                    if app.Ejec.Activa
                        app.ejec_finalizar("Se perdio la comunicacion serie durante la ejecucion.", 'error', false);
                    end
                    if app.Cont.Activa
                        app.cont_finalizar("Se perdio la comunicacion serie durante la trayectoria continua.", 'error', false);
                    end
                    return;
                end
                if app.Ejec.Activa
                    app.ejec_tick();
                end
                if app.Cont.Activa
                    app.cont_tick();
                end
            catch ME
                disp(ME.message);
            end
        end

        function leer_linea(app, src)
            if ~isvalid(app) || isempty(app.UIFigure) || ~isvalid(app.UIFigure), return; end
            try
                l = readline(src);
                if ismissing(l) || strlength(l) == 0, return; end
                linea = strtrim(string(l));
                app.UltimoRx = tic;

                if startsWith(linea, "st,")
                    app.procesar_st(linea);
                    return;
                end

                app.NaNLabel_3.Text = char(linea);
                if startsWith(linea, "err,")
                    app.NaNLabel_3.FontColor = [0.8 0 0];
                    msg = app.traducir_err(linea);
                    msg = [msg '   [' char(linea) ' | ultimo TX: ' char(app.UltimoTx) ']'];
                    app.avisar(msg, 'error');
                    if app.Ejec.Activa
                        app.ejec_finalizar("El STM32 respondio: " + msg, 'error', ~startsWith(linea, "err,fault"));
                    end
                    if app.Cont.Activa
                        app.cont_finalizar("El STM32 respondio: " + msg, 'error', ~startsWith(linea, "err,fault"));
                    end
                elseif startsWith(linea, "!fault") || startsWith(linea, "!comm_lost") || startsWith(linea, "!rx_overflow")
                    app.NaNLabel_3.FontColor = [0.8 0 0];
                    app.avisar(char(linea), 'error');
                    if app.Ejec.Activa
                        app.ejec_finalizar("El STM32 informo: " + linea, 'error', false);
                    end
                    if app.Cont.Activa
                        app.cont_finalizar("El STM32 informo: " + linea, 'error', false);
                    end
                elseif startsWith(linea, "!warn")
                    app.NaNLabel_3.FontColor = [0.75 0.45 0];
                    app.avisar(char(linea), 'aviso');
                else
                    app.NaNLabel_3.FontColor = [0 0 0];
                    if startsWith(linea, "ok") && app.Ejec.Activa && app.Ejec.Fase == "mover" && ~app.Ejec.Ack
                        app.Ejec.Ack = true;         % el G ya esta en el planificador
                        app.Ejec.NStAck = app.NSt;   % solo valen los "st" recibidos despues
                    elseif startsWith(linea, "ok") && app.Cont.Activa
                        app.cont_ok(linea);          % control de flujo del stream
                    end
                end
                if startsWith(linea, "!ready")
                    app.SincPendiente = true;        % la placa se reseteo: hay que referenciar de nuevo
                    app.QValida = false;
                    app.avisar('El STM32 se reinicio: referencia perdida (usa Referenciar o Calibrar).', 'aviso');
                    if app.Ens.Activo, app.cb_ens_parar(); end
                    if app.Cont.Activa
                        app.cont_finalizar("El STM32 se reinicio durante la trayectoria continua.", 'error', false);
                    end
                end
            catch ME
                app.NaNLabel_3.Text = char("Error al procesar linea: " + string(ME.message));
            end
        end

        function procesar_st(app, linea)
            % st,<estado>,<motores>,qref1,qref2,qref3,qenc1,qenc2,qenc3,<libres>,<falla>
            t = split(linea, ",");
            if numel(t) < 11, return; end
            estado = strtrim(t(2));
            v = str2double(t(3:11))';
            if any(isnan(v)), return; end
            qenc = v(5:7);

            app.NSt = app.NSt + 1;
            app.EstadoRobot = estado;
            app.Motores = v(1);
            app.FallaCod = v(9);
            app.QValida = (estado ~= "NOREF");
            if app.Reg.Activo
                app.reg_muestra(estado, v(2:4), qenc);    % qref = v(2:4), qenc = v(5:7)
            end
            if app.Cont.Activa, app.cont_st(v(8)); end
            if app.Ens.Activo && app.QValida
                app.ens_muestra(qenc);                    % grabacion a mano
            end
            if (app.Reg.Activo || app.Ens.Activo) && app.QValida
                % con Q a ~40 Hz la interfaz se refresca solo cada 0.2 s (si no MATLAB se atrasa)
                if ~isempty(app.UltUI) && toc(app.UltUI) < 0.2
                    app.QActual = qenc;
                    return;
                end
                app.UltUI = tic;
                if app.Ens.Activo, app.setQObjetivo(qenc); end   % el objetivo sigue al brazo mientras se graba
            end

            if ~app.QValida
                app.NaNLabel.Text = sprintf("%s | motores %s | sin referencia", estado, app.onoff_txt(app.Motores));
                app.actualizar_botones();
                return;
            end

            qdh = qenc;
            app.QActual = qdh;
            app.NaNLabel.Text = sprintf("%s | motores %s | q = [%.1f  %.1f  %.1f]", ...
                estado, app.onoff_txt(app.Motores), qdh(1), qdh(2), qdh(3));
            if ~isempty(app.Robot)
                p = app.tcp(qdh);
                app.XYZLabel.Text = sprintf("TCP informado = [%.3f  %.3f  %.3f] m", p(1), p(2), p(3));
                app.actualizar_3d();
                app.cont_preview_robot();           % pose actual en los graficos 2D
            end

            % Barras deslizantes = posicion REAL informada por el Q:
            %  - mientras el robot se mueve (RUN / HOLD) siguen al robot en vivo
            %  - cuando termina (primer IDLE despues de moverse) quedan en la posicion final
            %  - despues de R, C o M,1 se cargan con la posicion leida
            % Con el robot quieto en IDLE no se tocan, asi se puede elegir un objetivo nuevo.
            if any(estado == ["RUN","HOLD"])
                app.setQObjetivo(qdh);
                app.SincPendiente = true;          % al llegar a IDLE se carga la posicion final
            elseif app.SincPendiente && estado == "IDLE"
                app.setQObjetivo(qdh);
                app.SincPendiente = false;
            end
            fuera = qdh < app.q_limites(:,1)' - 0.5 | qdh > app.q_limites(:,2)' + 0.5;   % 0.5 deg de tolerancia por el ruido del encoder
            if any(fuera)
                k = find(fuera, 1);
                app.avisar(sprintf('q%d = %.1f esta fuera de los limites de la GUI [%g, %g]: la barra no lo puede mostrar (ajusta q%d_limites).', ...
                    k, qdh(k), app.q_limites(k,1), app.q_limites(k,2), k), 'aviso');
            end
        end

        function s = onoff_txt(~, v)
            if v == 1, s = "ON"; elseif v == 0, s = "OFF"; else, s = "?"; end
        end

        function msg = traducir_err(~, linea)
            % err,<motivo>[,eje]
            p = split(string(linea), ",");
            mot = ""; ax = "";
            if numel(p) >= 2, mot = strtrim(p(2)); end
            if numel(p) >= 3, ax = strtrim(p(3)); end
            switch mot
                case "noref",  msg = "Robot sin referenciar: usa Referenciar (R) o Calibrar (C).";
                case "nocal",  msg = "No hay calibracion guardada: usa Calibrar (C).";
                case "busy",   msg = "El robot esta ocupado (en movimiento).";
                case "fault",  msg = "Hay una falla activa: usa Detener (X) para borrarla.";
                case "limit",  msg = "Fuera de limites en la articulacion " + ax + ".";
                case "enc",    msg = "El encoder " + ax + " no tiene una lectura valida.";
                case "magnet", msg = "El encoder " + ax + " no detecta el iman.";
                case "flash",  msg = "Error al grabar en la flash del STM32.";
                case "cs",     msg = "Error de checksum en el comando.";
                case "syntax", msg = "Comando mal formado.";
                case "state",  msg = "Comando no valido en este estado.";
                case "long",   msg = "Linea demasiado larga.";
                case "cmd",    msg = "Comando desconocido.";
                otherwise,     msg = string(linea);
            end
            msg = char(msg);
        end

        function cerrar(app)
            % Cierre ordenado: avisa de cambios sin guardar, detiene el movimiento
            % en curso y libera el timer y el puerto.
            try
                if ~isempty(app.Lib) && app.Lib.Sucio
                    op = uiconfirm(app.UIFigure, ...
                        'Hay cambios sin guardar en las trayectorias.', 'Cerrar', ...
                        'Options', {'Guardar','No guardar','Cancelar'}, ...
                        'DefaultOption', 1, 'CancelOption', 3);
                    if op == "Cancelar", return; end
                    if op == "Guardar"
                        app.cb_guardar();
                        if app.Lib.Sucio, return; end      % no se pudo guardar: no cerrar
                    end
                end
            catch
            end
            try
                if app.ocupado(), writeline(app.Puerto, app.con_cs("X")); end
            catch
            end
            try, stop(app.Timer); delete(app.Timer); catch, end
            app.reg_borrar_timer();
            app.ens_borrar_timer();
            try, app.PanelTorque.detener(); catch, end
            try, configureCallback(app.Puerto, "off"); delete(app.Puerto); catch, end
            delete(app);
        end
    end

    %% ---------------------------------------------------------------
    %  Vista 3D (misma cinematica del modelo SerialLink)
    %% ---------------------------------------------------------------
    methods (Access = private)
        function V = vertices_robot(app, qdeg)
            % Vertices de la cadena: base, [codo DH d, origen sistema i] por eslabon, TCP.
            % Usa las mismas matrices A de cada Link que fkine => coincide con la FK.
            R = app.Robot;
            T = eye(4);
            try, T = double(R.base); catch, end
            V = T(1:3,4)';
            q = deg2rad(qdeg(:)');
            for j = 1:numel(R.links)
                L = R.links(j);
                p1 = T(1:3,4) + T(1:3,3) * L.d;       % tras Rz(theta)*Tz(d)
                T = T * double(L.A(q(j)));            % tras Tx(a)*Rx(alpha)
                V = [V; p1'; T(1:3,4)'];              %#ok<AGROW>
            end
            try
                T = T * double(R.tool);
                V = [V; T(1:3,4)'];                   %#ok<AGROW>
            catch
            end
        end

        function crear_3d(app)
            ax = app.Ax3D;
            cla(ax);
            hold(ax, 'on'); grid(ax, 'on'); box(ax, 'on');
            xlim(ax, [-0.4 0.4]); ylim(ax, [-0.4 0.4]); zlim(ax, [-0.05 0.45]);   % workspace de robot.m (eslabones de 150 mm)
            daspect(ax, [1 1 1]);
            view(ax, [135 25]);
            xlabel(ax, 'X [m]'); ylabel(ax, 'Y [m]'); zlabel(ax, 'Z [m]');
            title(ax, 'Brazo 3 GDL');
            try
                ax.Interactions = [rotateInteraction zoomInteraction panInteraction];
            catch
            end

            % sistema base 0 (X rojo, Y verde, Z azul)
            plot3(ax, [0 0.06], [0 0], [0 0], 'r-', 'LineWidth', 1.5);
            plot3(ax, [0 0], [0 0.06], [0 0], 'g-', 'LineWidth', 1.5);
            plot3(ax, [0 0], [0 0], [0 0.06], 'b-', 'LineWidth', 1.5);

            gris = [0.62 0.62 0.62];
            azul = [0.00 0.45 0.74];
            nara = [0.93 0.55 0.10];
            H = struct();
            H.rec    = plot3(ax, nan, nan, nan, '-o', 'Color', [0.80 0.20 0.17], ...
                             'LineWidth', 1, 'MarkerSize', 5, 'MarkerFaceColor', [0.80 0.20 0.17]);
            H.inf    = plot3(ax, nan, nan, nan, '-', 'Color', gris, 'LineWidth', 4);
            H.infJ   = plot3(ax, nan, nan, nan, 'o', 'Color', gris, 'MarkerFaceColor', [0.85 0.85 0.85], 'MarkerSize', 8);
            H.infTcp = plot3(ax, nan, nan, nan, 's', 'Color', gris, 'MarkerFaceColor', gris, 'MarkerSize', 9);
            H.sel    = plot3(ax, nan, nan, nan, '--', 'Color', nara, 'LineWidth', 2);
            H.selTcp = plot3(ax, nan, nan, nan, 'd', 'Color', nara, 'MarkerFaceColor', nara, 'MarkerSize', 10);
            H.obj    = plot3(ax, nan, nan, nan, '-', 'Color', azul, 'LineWidth', 4);
            H.objJ   = plot3(ax, nan, nan, nan, 'o', 'Color', azul, 'MarkerFaceColor', [1 1 1], 'MarkerSize', 9, 'LineWidth', 2);
            H.objTcp = plot3(ax, nan, nan, nan, 's', 'Color', azul, 'MarkerFaceColor', azul, 'MarkerSize', 10);
            verde = [0.20 0.62 0.25];
            H.cont    = plot3(ax, nan, nan, nan, '-', 'Color', verde, 'LineWidth', 1.5);      % trayectoria continua
            H.contIni = plot3(ax, nan, nan, nan, 'o', 'Color', verde, 'MarkerFaceColor', verde, 'MarkerSize', 6);
            H.contBad = plot3(ax, nan, nan, nan, 'x', 'Color', [0.85 0.10 0.10], 'MarkerSize', 7, 'LineWidth', 1.5);  % violaciones
            H.contCrit = plot3(ax, nan, nan, nan, 'd', 'Color', [0 0 0], 'MarkerFaceColor', [1 0.85 0], 'MarkerSize', 9);  % puntos de paso
            H.contStop = plot3(ax, nan, nan, nan, 's', 'Color', [0 0 0], 'MarkerFaceColor', [0.90 0.15 0.15], 'MarkerSize', 10);  % puntos de parada
            app.H3 = H;
        end

        function actualizar_3d(app)
            % Objetivo (azul), q informada (gris) y punto seleccionado (naranja)
            if isempty(app.Robot) || isempty(fieldnames(app.H3)), return; end
            H = app.H3;
            try
                V = app.vertices_robot(app.qObjetivo());
                app.set3(H.obj, V);
                app.set3(H.objJ, V([1 3 5], :));
                app.set3(H.objTcp, V(end, :));

                if app.QValida
                    Vi = app.vertices_robot(app.QActual);
                    app.set3(H.inf, Vi);
                    app.set3(H.infJ, Vi([1 3 5], :));
                    app.set3(H.infTcp, Vi(end, :));
                end

                if app.punto_valido()
                    p = app.Lib.Lista(app.TrayIdx).puntos(app.PuntoIdx);
                    Vs = app.vertices_robot(p.q);
                    app.set3(H.sel, Vs);
                    app.set3(H.selTcp, Vs(end, :));
                else
                    app.set3(H.sel, nan(1,3));
                    app.set3(H.selTcp, nan(1,3));
                end
            catch
            end
        end

        function actualizar_3d_recorrido(app)
            % Recorrido previsto del TCP de la trayectoria seleccionada
            if isempty(app.Robot) || isempty(fieldnames(app.H3)), return; end
            P = nan(1,3);
            try
                if app.ChkRecorrido.Value && app.TrayIdx >= 1 && app.TrayIdx <= app.Lib.cantidad()
                    pts = app.Lib.Lista(app.TrayIdx).puntos;
                    if ~isempty(pts)
                        P = zeros(numel(pts), 3);
                        for k = 1:numel(pts)
                            P(k,:) = app.fk_xyz(pts(k).q);
                        end
                    end
                end
            catch
                P = nan(1,3);
            end
            app.set3(app.H3.rec, P);
        end

        function set3(~, h, V)
            set(h, 'XData', V(:,1), 'YData', V(:,2), 'ZData', V(:,3));
        end

        function cb_vista(app)
            view(app.Ax3D, [135 25]);
            xlim(app.Ax3D, [-0.4 0.4]); ylim(app.Ax3D, [-0.4 0.4]); zlim(app.Ax3D, [-0.05 0.45]);
        end
    end

    %% ---------------------------------------------------------------
    %  Callbacks: control manual
    %% ---------------------------------------------------------------
    methods (Access = private)
        function cb_slider(app, k, valor)
            ed = {app.q1EditField, app.q2EditField, app.q3EditField};
            ed{k}.Value = valor;
            app.actualizar_objetivo();       % solo objetivo local; no se envia nada
        end

        function cb_edit(app, k)
            ed = {app.q1EditField, app.q2EditField, app.q3EditField};
            sl = {app.q1Slider, app.q2Slider, app.q3Slider};
            sl{k}.Value = ed{k}.Value;
            app.actualizar_objetivo();
        end

        function cb_restablecer(app)
            if ~app.QValida
                app.avisar('Todavia no hay una posicion valida del STM32 (robot sin referenciar).', 'aviso');
                return;
            end
            app.setQObjetivo(app.QActual);
            app.avisar('Controles cargados con la q informada por el STM32 (no se envio nada).', 'info');
        end

        function cb_comando(app, cmd)
            if app.ocupado()
                app.avisar('Hay una trayectoria en ejecucion: solo se permite Detener.', 'aviso');
                return;
            end
            app.enviar(cmd);
            if any(string(cmd) == ["R","M,1"]), app.SincPendiente = true; end
        end

        function cb_calibrar(app)
            % C,q1,q2,q3: le dice al firmware "el robot esta AHORA en esta pose" (graba en flash)
            if app.ocupado(), return; end
            qc = app.Q_CAL;
            txt = sprintf(['El robot debe estar FISICAMENTE en la pose de calibracion.\n\n' ...
                'Pose: q = [%.2f  %.2f  %.2f] deg\n\n' ...
                '- Ejes con encoder: basta estar a menos de media vuelta de motor (45 deg en q2, 40 deg en q3).\n' ...
                '- Ejes SIN encoder: tiene que estar EXACTO, el firmware no puede medirlo.\n\n' ...
                'Se graba en la flash y el robot queda referenciado.'], qc);
            op = uiconfirm(app.UIFigure, txt, 'Calibrar', ...
                'Options', {'Calibrar en esta pose','Usar el objetivo actual','Cancelar'}, ...
                'DefaultOption', 3, 'CancelOption', 3);
            if op == "Cancelar", return; end
            if op == "Usar el objetivo actual", qc = app.qObjetivo(); end
            app.enviar(sprintf("C,%.3f,%.3f,%.3f", qc));
            app.SincPendiente = true;
        end

        function cb_limites(app)
            % L,eje,min,max para los 3 ejes (graba en flash)
            if app.ocupado(), return; end
            op = uiconfirm(app.UIFigure, ...
                'Se envian los limites articulares de esta GUI al STM32 y se graban en su flash.', ...
                'Limites -> STM32', 'Options', {'Enviar','Cancelar'}, 'DefaultOption', 2, 'CancelOption', 2);
            if op == "Cancelar", return; end
            for k = 1:3
                if ~app.enviar(sprintf("L,%d,%.2f,%.2f", k, app.q_limites(k,1), app.q_limites(k,2))), return; end
                pause(0.15);    % cada L graba la flash (~25 ms con el micro frenado): no mandar el siguiente encima
            end
        end

        function cb_stop(app)
            if app.Cont.Activa
                app.cont_finalizar("Trayectoria continua detenida por el usuario (se envio X).", 'aviso', true);
            elseif app.Ejec.Activa
                app.ejec_finalizar("Ejecucion detenida por el usuario (se envio X).", 'aviso', true);
            else
                app.enviar("X");
            end
        end

        % Consola cruda: los comandos van tal cual al firmware
        function cb_cmd(app)
            cmd = strtrim(string(app.cmdEditField.Value));
            app.cmdEditField.Value = "";
            if strlength(cmd) == 0, return; end
            cmd = upper(cmd);
            if app.ocupado() && ~any(cmd == ["X","Q","W"])
                app.avisar('Hay una trayectoria en ejecucion: solo se permite X, Q o W.', 'aviso');
                return;
            end
            app.enviar(cmd);             % G,.. J,.. M,.. C,.. R L,.. X Q W (el checksum se agrega solo)
            if cmd == "R" || cmd == "M,1" || startsWith(cmd, "C,")
                app.SincPendiente = true;  % cargar la posicion leida en las barras
            end
        end
    end

    %% ---------------------------------------------------------------
    %  Trayectorias: refresco de la vista
    %% ---------------------------------------------------------------
    methods (Access = private)
        function ok = edicion_permitida(app)
            ok = ~app.Ejec.Activa;
            if ~ok
                app.avisar('No se puede editar mientras hay una trayectoria en ejecucion.', 'aviso');
            end
        end

        function ok = hay_tray(app)
            ok = app.TrayIdx >= 1 && app.TrayIdx <= app.Lib.cantidad();
            if ~ok
                app.avisar('Selecciona o crea una trayectoria primero.', 'aviso');
            end
        end

        function ok = punto_valido(app)
            % Sin mensajes: hay un punto seleccionado y existe
            ok = false;
            if app.TrayIdx < 1 || app.TrayIdx > app.Lib.cantidad(), return; end
            n = numel(app.Lib.Lista(app.TrayIdx).puntos);
            ok = app.PuntoIdx >= 1 && app.PuntoIdx <= n;
        end

        function ok = hay_punto(app)
            ok = false;
            if ~app.hay_tray(), return; end
            ok = app.punto_valido();
            if ~ok
                app.avisar('Selecciona un punto en la tabla.', 'aviso');
            end
        end

        function seleccionar_tray(app, i)
            app.TrayIdx = i;
            app.PuntoIdx = 0;
            app.refrescar_todo();
        end

        function refrescar_todo(app)
            app.refrescar_lista();
            app.refrescar_tabla();
            app.refrescar_archivo();
            app.actualizar_3d_recorrido();
            app.actualizar_3d();
        end

        function refrescar_lista(app)
            n = app.Lib.cantidad();
            if n == 0
                app.ListaTray.Items = {};
                app.ListaTray.ItemsData = [];
                app.TrayIdx = 0;
                return;
            end
            app.TrayIdx = min(max(app.TrayIdx, 1), n);
            nom = app.Lib.nombres();
            for k = 1:n
                nom{k} = sprintf('%s   (%d)', nom{k}, numel(app.Lib.Lista(k).puntos));
            end
            app.ListaTray.Items = nom;
            app.ListaTray.ItemsData = 1:n;
            app.ListaTray.Value = app.TrayIdx;
        end

        function refrescar_tabla(app)
            if app.TrayIdx < 1 || app.TrayIdx > app.Lib.cantidad()
                app.TablaPuntos.Data = cell(0, 9);
                app.LblTrayNombre.Text = 'Sin trayectoria seleccionada';
                app.LblSecuencia.Text = 'Crea una trayectoria con "Nueva trayectoria".';
                app.LblPuntoInfo.Text = '';
                app.aplicar_estilos_tabla();
                return;
            end
            t = app.Lib.Lista(app.TrayIdx);
            n = numel(t.puntos);
            if app.PuntoIdx > n, app.PuntoIdx = n; end
            D = cell(n, 9);
            for k = 1:n
                p = t.puntos(k);
                D(k,:) = {k, p.nombre, round(p.q(1),2), round(p.q(2),2), round(p.q(3),2), ...
                          round(p.xyz(1),4), round(p.xyz(2),4), round(p.xyz(3),4), ...
                          Trayectorias.textoIman(p.iman)};
            end
            app.TablaPuntos.Data = D;
            app.LblTrayNombre.Text = sprintf('%s   -   %d punto(s)', t.nombre, n);
            app.refrescar_secuencia();
            app.refrescar_info_punto();
            app.aplicar_estilos_tabla();
        end

        function refrescar_secuencia(app)
            % Secuencia en orden: Punto_a -> Punto_b -> ...
            if app.TrayIdx < 1, app.LblSecuencia.Text = ''; return; end
            nom = app.Lib.nombresPuntos(app.TrayIdx);
            if isempty(nom)
                app.LblSecuencia.Text = ['Sin puntos. Define una posicion con los controles de "Control manual" ' ...
                    'y pulsa "+ Anadir punto actual".'];
                return;
            end
            ej = app.Ejec.Activa && app.Ejec.TrayIdx == app.TrayIdx && any(app.Ejec.Fase == ["mover","iman"]);
            for k = 1:numel(nom)
                if ej && k == app.Ejec.Idx
                    nom{k} = [app.S_PLAY ' ' nom{k}];
                end
                if k == app.PuntoIdx
                    nom{k} = ['[' nom{k} ']'];
                end
            end
            app.LblSecuencia.Text = ['Secuencia: ' strjoin(nom, ['  ' app.S_ARROW '  '])];
        end

        function refrescar_info_punto(app)
            if ~app.punto_valido()
                app.LblPuntoInfo.Text = 'Punto seleccionado: ninguno';
                return;
            end
            p = app.Lib.Lista(app.TrayIdx).puntos(app.PuntoIdx);
            app.LblPuntoInfo.Text = sprintf( ...
                'Punto %d "%s":  q = [%.2f  %.2f  %.2f] deg   XYZ = [%.4f  %.4f  %.4f] m   Iman: %s', ...
                p.orden, p.nombre, p.q, p.xyz, Trayectorias.textoIman(p.iman));
        end

        function aplicar_estilos_tabla(app)
            try
                removeStyle(app.TablaPuntos);
                nf = size(app.TablaPuntos.Data, 1);
                if app.PuntoIdx >= 1 && app.PuntoIdx <= nf
                    addStyle(app.TablaPuntos, uistyle('BackgroundColor', [0.80 0.90 1.00]), 'row', app.PuntoIdx);
                end
                if app.Ejec.Activa && app.Ejec.TrayIdx == app.TrayIdx && any(app.Ejec.Fase == ["mover","iman"]) ...
                        && app.Ejec.Idx >= 1 && app.Ejec.Idx <= nf
                    addStyle(app.TablaPuntos, uistyle('BackgroundColor', [0.80 0.95 0.80]), 'row', app.Ejec.Idx);
                end
            catch
            end
        end

        function refrescar_archivo(app)
            if strlength(app.Lib.Archivo) == 0
                txt = 'Archivo: (sin guardar)';
            else
                [~, nm, ex] = fileparts(char(app.Lib.Archivo));
                txt = ['Archivo: ' nm ex];
            end
            if app.Lib.Sucio
                txt = [txt '   ' app.S_DOT ' CAMBIOS SIN GUARDAR'];
                app.LblArchivo.FontColor = [0.75 0.45 0];
                app.TabTray.Title = ['Trayectorias ' app.S_DOT];
            else
                app.LblArchivo.FontColor = [0.35 0.35 0.35];
                app.TabTray.Title = 'Trayectorias';
            end
            app.LblArchivo.Text = txt;
        end
    end

    %% ---------------------------------------------------------------
    %  Trayectorias: callbacks (nunca envian nada por UART salvo "Enviar punto")
    %% ---------------------------------------------------------------
    methods (Access = private)
        function t = dlg_texto(~, titulo, prompt, def)
            t = "";
            try
                r = inputdlg(prompt, titulo, [1 50], {char(def)});
            catch
                r = {};
            end
            if ~isempty(r), t = string(strtrim(r{1})); end
        end

        function cb_lista(app)
            if isempty(app.ListaTray.Items), return; end
            app.TrayIdx = app.ListaTray.Value;
            app.PuntoIdx = 0;
            app.refrescar_tabla();
            app.actualizar_3d_recorrido();
            app.actualizar_3d();
        end

        function cb_nueva_tray(app)
            if ~app.edicion_permitida(), return; end
            nom = app.dlg_texto('Nueva trayectoria', 'Nombre de la trayectoria:', ...
                app.Lib.nombreUnicoTray('Trayectoria'));
            if strlength(nom) == 0, return; end
            try
                i = app.Lib.crear(nom);
                app.seleccionar_tray(i);
                app.avisar("Trayectoria creada: " + nom, 'ok');
            catch ME
                app.alerta('Nueva trayectoria', ME.message);
            end
        end

        function cb_ren_tray(app)
            if ~app.edicion_permitida() || ~app.hay_tray(), return; end
            nom = app.dlg_texto('Renombrar trayectoria', 'Nuevo nombre:', ...
                app.Lib.Lista(app.TrayIdx).nombre);
            if strlength(nom) == 0, return; end
            try
                app.Lib.renombrar(app.TrayIdx, nom);
                app.refrescar_todo();
            catch ME
                app.alerta('Renombrar', ME.message);
            end
        end

        function cb_dup_tray(app)
            if ~app.edicion_permitida() || ~app.hay_tray(), return; end
            try
                app.seleccionar_tray(app.Lib.duplicar(app.TrayIdx));
            catch ME
                app.alerta('Duplicar', ME.message);
            end
        end

        function cb_del_tray(app)
            if ~app.edicion_permitida() || ~app.hay_tray(), return; end
            nom = app.Lib.Lista(app.TrayIdx).nombre;
            op = uiconfirm(app.UIFigure, ...
                sprintf('Eliminar la trayectoria "%s" y todos sus puntos?', nom), ...
                'Eliminar trayectoria', 'Options', {'Eliminar','Cancelar'}, ...
                'DefaultOption', 2, 'CancelOption', 2, 'Icon', 'warning');
            if op ~= "Eliminar", return; end
            app.Lib.eliminar(app.TrayIdx);
            app.seleccionar_tray(min(app.TrayIdx, app.Lib.cantidad()));
        end

        %% ---- archivo ----
        function ok = confirmar_descartar(app)
            ok = true;
            if ~app.Lib.Sucio, return; end
            op = uiconfirm(app.UIFigure, ...
                'Hay cambios sin guardar que se perderan.', 'Cambios sin guardar', ...
                'Options', {'Descartar cambios','Cancelar'}, ...
                'DefaultOption', 2, 'CancelOption', 2, 'Icon', 'warning');
            ok = (op == "Descartar cambios");
        end

        function cb_abrir(app)
            if ~app.edicion_permitida() || ~app.confirmar_descartar(), return; end
            [f, d] = uigetfile({'*.mat','Biblioteca de trayectorias (*.mat)'}, 'Abrir trayectorias', app.carpeta_defecto());
            figure(app.UIFigure);
            if isequal(f, 0), return; end
            try
                app.Lib.cargar(fullfile(d, f));
                app.TrayIdx = min(1, app.Lib.cantidad());
                app.PuntoIdx = 0;
                app.refrescar_todo();
                app.avisar(sprintf('Cargadas %d trayectoria(s) de %s', app.Lib.cantidad(), f), 'ok');
            catch ME
                app.alerta('No se pudo abrir el archivo', ME.message);
            end
        end

        function cb_guardar(app)
            if strlength(app.Lib.Archivo) == 0
                app.cb_guardar_como();
                return;
            end
            try
                app.Lib.guardar();
                app.refrescar_archivo();
                app.avisar('Trayectorias guardadas.', 'ok');
            catch ME
                app.alerta('No se pudo guardar', ME.message);
            end
        end

        function cb_guardar_como(app)
            [f, d] = uiputfile({'*.mat','Biblioteca de trayectorias (*.mat)'}, 'Guardar trayectorias', ...
                fullfile(app.carpeta_defecto(), app.ARCHIVO_DEFECTO));
            figure(app.UIFigure);
            if isequal(f, 0), return; end
            try
                app.Lib.guardar(fullfile(d, f));
                app.refrescar_archivo();
                app.avisar("Trayectorias guardadas en " + string(f), 'ok');
            catch ME
                app.alerta('No se pudo guardar', ME.message);
            end
        end

        function cb_importar(app)
            if ~app.edicion_permitida(), return; end
            [f, d] = uigetfile({'*.mat','Biblioteca de trayectorias (*.mat)'}, 'Importar trayectorias', app.carpeta_defecto());
            figure(app.UIFigure);
            if isequal(f, 0), return; end
            try
                n = app.Lib.importar(fullfile(d, f));
                if app.TrayIdx == 0, app.TrayIdx = min(1, app.Lib.cantidad()); end
                app.refrescar_todo();
                app.avisar(sprintf('Importadas %d trayectoria(s). Recorda guardar.', n), 'ok');
            catch ME
                app.alerta('No se pudo importar', ME.message);
            end
        end

        function cb_exportar(app)
            if ~app.hay_tray(), return; end
            nom = matlab.lang.makeValidName(app.Lib.Lista(app.TrayIdx).nombre);
            [f, d] = uiputfile({'*.mat','Biblioteca de trayectorias (*.mat)'}, 'Exportar trayectoria', ...
                fullfile(app.carpeta_defecto(), [nom '.mat']));
            figure(app.UIFigure);
            if isequal(f, 0), return; end
            try
                app.Lib.exportar(app.TrayIdx, fullfile(d, f));
                app.avisar("Trayectoria exportada a " + string(f), 'ok');
            catch ME
                app.alerta('No se pudo exportar', ME.message);
            end
        end

        %% ---- tabla de puntos ----
        function cb_seleccion_celda(app, ev)
            if isempty(ev.Indices), return; end
            app.PuntoIdx = ev.Indices(1,1);
            app.refrescar_secuencia();
            app.refrescar_info_punto();
            app.aplicar_estilos_tabla();
            app.actualizar_3d();             % vista previa 3D (naranja); no toca controles ni UART
        end

        function cb_edicion_celda(app, ev)
            if ~app.edicion_permitida() || ~app.hay_tray()
                app.refrescar_tabla(); return;
            end
            k = ev.Indices(1);  c = ev.Indices(2);
            i = app.TrayIdx;
            try
                p = app.Lib.Lista(i).puntos(k);
                if c == 2
                    app.Lib.renombrarPunto(i, k, ev.NewData);
                elseif any(c == [3 4 5])
                    v = ev.NewData;
                    j = c - 2;
                    if ~(isnumeric(v) && isscalar(v) && isfinite(v))
                        error('Trayectorias:coordenadas', 'Ingresa un valor numerico valido.');
                    end
                    lim = app.q_limites(j,:);
                    if v < lim(1) || v > lim(2)
                        error('Trayectorias:limite', 'q%d debe estar entre %g y %g deg.', j, lim(1), lim(2));
                    end
                    q = p.q;  q(j) = v;
                    app.Lib.editarPunto(i, k, q, app.fk_xyz(q));   % recalcula XYZ
                end
            catch ME
                app.alerta('Edicion no valida', ME.message);
            end
            app.PuntoIdx = k;
            app.refrescar_todo();
        end

        %% ---- puntos ----
        function r = dlg_punto(app)
            % Dialogo modal: elegir trayectoria (o crear una) y nombrar el punto
            r = [];
            n = app.Lib.cantidad();
            p = app.UIFigure.Position;
            d = uifigure('Name', 'Anadir punto', 'WindowStyle', 'modal', ...
                'Position', [p(1)+p(3)/2-190, p(2)+p(4)/2-110, 380, 220]);
            U = struct('ok', false);

            uilabel(d, 'Position', [20 180 340 22], 'Text', 'Trayectoria destino:');
            U.dd = uidropdown(d, 'Position', [20 155 340 24], ...
                'Items', [app.Lib.nombres(), {'<Nueva trayectoria...>'}], ...
                'ItemsData', 1:(n+1));
            if app.TrayIdx >= 1 && app.TrayIdx <= n
                U.dd.Value = app.TrayIdx;
            else
                U.dd.Value = n + 1;
            end
            uilabel(d, 'Position', [20 120 340 22], 'Text', 'Nombre de la nueva trayectoria:');
            U.enT = uieditfield(d, 'text', 'Position', [20 95 340 24], ...
                'Value', char(app.Lib.nombreUnicoTray('Trayectoria')));
            uilabel(d, 'Position', [20 62 340 22], 'Text', 'Nombre del punto:');
            if app.TrayIdx >= 1 && app.TrayIdx <= n
                nomP = app.Lib.nombreUnicoPunto(app.TrayIdx, 'Punto');
            else
                nomP = 'Punto';
            end
            U.enP = uieditfield(d, 'text', 'Position', [20 37 340 24], 'Value', char(nomP));
            uibutton(d, 'push', 'Position', [190 5 80 26], 'Text', 'Aceptar', ...
                'ButtonPushedFcn', @(~,~) app.dlg_punto_ok(d));
            uibutton(d, 'push', 'Position', [280 5 80 26], 'Text', 'Cancelar', ...
                'ButtonPushedFcn', @(~,~) uiresume(d));
            U.dd.ValueChangedFcn = @(~,~) app.dlg_punto_dd(d);
            d.CloseRequestFcn = @(~,~) uiresume(d);
            d.UserData = U;
            app.dlg_punto_dd(d);

            uiwait(d);
            if isvalid(d)
                U = d.UserData;
                if U.ok
                    r = struct('nueva', U.dd.Value == n + 1, 'indice', U.dd.Value, ...
                               'nombreTray', string(U.enT.Value), 'nombrePunto', string(U.enP.Value));
                end
                delete(d);
            end
        end

        function dlg_punto_dd(app, d)
            U = d.UserData;
            nueva = U.dd.Value == app.Lib.cantidad() + 1;
            U.enT.Enable = app.onoff(nueva);
        end

        function dlg_punto_ok(~, d)
            U = d.UserData;
            U.ok = true;
            d.UserData = U;
            uiresume(d);
        end

        function cb_add_punto(app)
            if ~app.edicion_permitida(), return; end
            r = app.dlg_punto();
            if isempty(r), return; end
            try
                if r.nueva
                    i = app.Lib.crear(r.nombreTray);
                else
                    i = r.indice;
                end
                q = app.qObjetivo();
                xyz = app.fk_xyz(q);
                im = app.ImanActual;                     % estado del iman informado en este momento
                k = app.Lib.agregarPunto(i, r.nombrePunto, q, xyz, im);
                app.TrayIdx = i;
                app.PuntoIdx = k;
                app.refrescar_todo();
                if any(isnan(xyz))
                    app.avisar('Punto guardado SIN XYZ: el modelo cinematico no esta cargado (ejecuta robot.m).', 'aviso');
                elseif ~app.HAY_IMAN
                    app.avisar("Punto '" + r.nombrePunto + "' anadido a '" + string(app.Lib.Lista(i).nombre) + "'.", 'ok');
                elseif isnan(im)
                    app.avisar("Punto '" + r.nombrePunto + "' anadido SIN estado de iman (todavia no se recibio estado del STM32).", 'aviso');
                else
                    app.avisar("Punto '" + r.nombrePunto + "' anadido a '" + string(app.Lib.Lista(i).nombre) + ...
                        "' (iman " + string(Trayectorias.textoIman(im)) + ").", 'ok');
                end
            catch ME
                app.alerta('Anadir punto', ME.message);
                app.refrescar_todo();
            end
        end

        function cb_dup_punto(app)
            if ~app.edicion_permitida() || ~app.hay_punto(), return; end
            try
                app.PuntoIdx = app.Lib.duplicarPunto(app.TrayIdx, app.PuntoIdx);
                app.refrescar_todo();
            catch ME
                app.alerta('Duplicar punto', ME.message);
            end
        end

        function cb_del_punto(app)
            if ~app.edicion_permitida() || ~app.hay_punto(), return; end
            nom = app.Lib.Lista(app.TrayIdx).puntos(app.PuntoIdx).nombre;
            op = uiconfirm(app.UIFigure, sprintf('Eliminar el punto "%s"?', nom), 'Eliminar punto', ...
                'Options', {'Eliminar','Cancelar'}, 'DefaultOption', 2, 'CancelOption', 2, 'Icon', 'warning');
            if op ~= "Eliminar", return; end
            app.Lib.eliminarPunto(app.TrayIdx, app.PuntoIdx);
            app.PuntoIdx = min(app.PuntoIdx, numel(app.Lib.Lista(app.TrayIdx).puntos));
            app.refrescar_todo();
        end

        function mover_punto(app, delta)
            % Reordena: no cambia nombres ni coordenadas, solo la posicion en la secuencia
            if ~app.edicion_permitida() || ~app.hay_punto(), return; end
            try
                app.PuntoIdx = app.Lib.moverPunto(app.TrayIdx, app.PuntoIdx, app.PuntoIdx + delta);
                app.refrescar_todo();
            catch ME
                app.alerta('Reordenar', ME.message);
            end
        end

        function cb_act_punto(app)
            % Reemplaza las coordenadas del punto por la posicion OBJETIVO actual
            if ~app.edicion_permitida() || ~app.hay_punto(), return; end
            try
                q = app.qObjetivo();
                im = app.ImanActual;
                if isnan(im)        % sin dato actual: conservar el estado ya guardado en el punto
                    im = app.Lib.Lista(app.TrayIdx).puntos(app.PuntoIdx).iman;
                end
                app.Lib.editarPunto(app.TrayIdx, app.PuntoIdx, q, app.fk_xyz(q), im);
                app.refrescar_todo();
                app.avisar('Punto actualizado con la posicion objetivo y el estado actual del iman.', 'ok');
            catch ME
                app.alerta('Actualizar punto', ME.message);
            end
        end

        function cb_ir_punto(app)
            % Carga el punto en los controles (objetivo). NO envia nada.
            if ~app.hay_punto(), return; end
            p = app.Lib.Lista(app.TrayIdx).puntos(app.PuntoIdx);
            app.setQObjetivo(p.q);
            app.avisar("Punto '" + string(p.nombre) + "' cargado como objetivo (no enviado).", 'info');
        end

        function cb_env_punto(app)
            % Accion explicita: carga el punto y envia el P2P (G)
            if ~app.hay_punto(), return; end
            p = app.Lib.Lista(app.TrayIdx).puntos(app.PuntoIdx);
            app.setQObjetivo(p.q);
            app.enviar_move();
        end
    end

    %% ---------------------------------------------------------------
    %  Ejecucion de trayectorias (maquina de estados, sin bucles bloqueantes)
    %  Se avanza desde el timer de 0.5 s (poll_status -> ejec_tick). Un solo G
    %  en vuelo a la vez: el siguiente punto se envia solo cuando el actual se
    %  da por terminado.
    %% ---------------------------------------------------------------
    methods (Access = private)
        function cb_ejecutar(app)
            if app.ocupado(), return; end
            if ~app.conectado()
                app.alerta('Sin conexion', 'No hay conexion con el STM32.');
                return;
            end
            if ~app.hay_tray(), return; end

            [ok, msg] = app.Lib.validarEjecucion(app.TrayIdx, app.q_limites);
            if ~ok
                app.alerta('Trayectoria no valida', msg);
                return;
            end
            if ~app.robot_listo_para_mover(), return; end

            pts = app.Lib.Lista(app.TrayIdx).puntos;
            app.Ejec = struct('Activa', true, 'Fase', "mover", 'TrayIdx', app.TrayIdx, ...
                'Idx', 1, 'N', numel(pts), 'Q', vertcat(pts.q), 'Iman', vertcat(pts.iman), 'ImanObj', NaN, ...
                'Nombres', {{pts.nombre}}, ...
                'T0', [], 'Bucle', startsWith(string(app.DdModo.Value), "Bucle"), ...
                'Ciclo', 1, 'PausaPend', false, 'TMin', app.TMinEdit.Value, 'Ack', false, 'NStAck', 0);
            app.actualizar_botones();
            app.avisar('Ejecucion iniciada.', 'info');
            app.reg_iniciar(vertcat(pts.q), vertcat(pts.xyz), "Trayectoria: " + string(app.Lib.Lista(app.TrayIdx).nombre), 1:numel(pts));
            app.ejec_enviar_punto();
        end

        function ejec_enviar_punto(app)
            q = app.Ejec.Q(app.Ejec.Idx, :);
            app.Ejec.Ack = false;
            if ~app.enviar(sprintf("G,%.3f,%.3f,%.3f,%.2f", q, app.VelEditField.Value))
                return;      % enviar() ya finalizo la ejecucion por fallo serie
            end
            app.Ejec.T0 = tic;
            app.Ejec.Fase = "mover";
            app.ejec_ui();
        end

        function ejec_tick(app)
            E = app.Ejec;
            if ~E.Activa, return; end

            if ~isempty(app.UltimoRx) && toc(app.UltimoRx) > app.TIMEOUT_RX
                app.ejec_finalizar(sprintf("El STM32 no responde (sin datos hace mas de %g s).", app.TIMEOUT_RX), 'error', false);
                return;
            end
            est = string(app.EstadoRobot);
            if any(est == ["FAULT","NOREF"])
                app.ejec_finalizar("El robot informo estado " + est + ": ejecucion abortada.", 'error', false);
                return;
            end
            if ~any(E.Fase == ["mover","iman"]) || isempty(E.T0), return; end

            t = toc(E.T0);
            if E.Fase == "iman"
                % Esperar a que el firmware confirme el estado del iman pedido
                if app.ImanActual == E.ImanObj && t >= app.T_IMAN_MIN
                    app.ejec_punto_terminado();
                elseif t > app.T_IMAN_MAX
                    app.ejec_finalizar(sprintf("El STM32 no confirmo el iman %s en el punto '%s' (%g s).", ...
                        Trayectorias.textoIman(E.ImanObj), E.Nombres{E.Idx}, app.T_IMAN_MAX), 'error', true);
                end
                return;
            end

            llego = norm(app.QActual - E.Q(E.Idx, :)) <= app.TOL_Q;
            % libre: el G ya fue aceptado (ok) y un Q POSTERIOR dice IDLE
            libre = E.Ack && app.NSt > E.NStAck && est == "IDLE";
            if t >= E.TMin && libre && llego
                app.ejec_llegada();
            elseif t > app.TMAX_PUNTO
                app.ejec_finalizar(sprintf("El punto '%s' no se completo en %g s (q informada no alcanzo el objetivo).", ...
                    E.Nombres{E.Idx}, app.TMAX_PUNTO), 'error', true);
            end
        end

        function ejec_llegada(app)
            % Llegada al punto: aplica el estado de iman guardado (si se conoce
            % y difiere del actual) y recien despues pasa al siguiente punto.
            im = app.Ejec.Iman(app.Ejec.Idx);
            if ~app.HAY_IMAN || isnan(im) || app.ImanActual == im
                app.ejec_punto_terminado();
                return;
            end
            if ~app.enviar(sprintf("MAG %d", im))
                return;      % enviar() ya finalizo la ejecucion por fallo serie
            end
            app.Ejec.ImanObj = im;
            app.Ejec.T0 = tic;
            app.Ejec.Fase = "iman";
            app.ejec_ui();
        end

        function ejec_punto_terminado(app)
            app.Ejec.Idx = app.Ejec.Idx + 1;
            if app.Ejec.Idx > app.Ejec.N
                if app.Ejec.Bucle
                    app.Ejec.Idx = 1;
                    app.Ejec.Ciclo = app.Ejec.Ciclo + 1;
                else
                    app.ejec_finalizar("Trayectoria completada.", 'ok', false);
                    return;
                end
            end
            if app.Ejec.PausaPend
                app.Ejec.Fase = "pausada";
                app.Ejec.PausaPend = false;
                app.ejec_ui();
                app.actualizar_botones();
                return;
            end
            app.ejec_enviar_punto();
        end

        function cb_pausa(app)
            if ~app.Ejec.Activa, return; end
            if app.Ejec.Fase == "pausada"
                app.ejec_enviar_punto();            % reanudar: envia el siguiente punto
            else
                app.Ejec.PausaPend = ~app.Ejec.PausaPend;   % pausa al terminar el punto en curso
                app.ejec_ui();
            end
            app.actualizar_botones();
        end

        function cb_detener(app)
            if app.Ejec.Activa
                app.ejec_finalizar("Ejecucion detenida por el usuario (se envio X).", 'aviso', true);
            end
        end

        function ejec_finalizar(app, msg, tipo, enviarStop)
            if enviarStop
                try
                    writeline(app.Puerto, app.con_cs("X"));
                    app.NaNLabel_2.Text = 'TX: X';
                catch
                end
            end
            app.Ejec.Activa = false;
            app.Ejec.Fase = "";
            app.Ejec.PausaPend = false;
            app.reg_fin_trayectoria();               % sigue registrando REG_COLA s y cierra
            app.ejec_ui();
            app.actualizar_botones();
            if tipo == "error"
                app.alerta('Ejecucion', msg);
            else
                app.avisar(msg, tipo);
            end
        end

        function ejec_ui(app)
            E = app.Ejec;
            if E.Activa
                if E.Fase == "pausada"
                    txt = "Pausada";
                    prog = sprintf('Siguiente: punto %d/%d "%s"   |   ciclo %d', E.Idx, E.N, E.Nombres{E.Idx}, E.Ciclo);
                else
                    txt = "Ejecutando";
                    if E.Fase == "iman", txt = "Ejecutando (iman " + string(Trayectorias.textoIman(E.ImanObj)) + ")"; end
                    if E.PausaPend, txt = txt + " (pausa al terminar el punto)"; end
                    prog = sprintf('Punto %d/%d "%s"   |   ciclo %d', E.Idx, E.N, E.Nombres{E.Idx}, E.Ciclo);
                end
                app.LblEjecEstado.Text = char(txt);
                app.LblProgreso.Text = prog;
            else
                app.LblEjecEstado.Text = 'Inactiva';
                app.LblProgreso.Text = '';
            end
            app.refrescar_secuencia();
            app.aplicar_estilos_tabla();
        end
    end

    %% ---------------------------------------------------------------
    %  Trayectoria continua (stream S / P / E con control de flujo)
    %
    %  Genera rectas 3D y circulos (o carga un .mat de puntos densos), pasa todo
    %  a q con la CI y lo manda al firmware como un stream:
    %     G al primer punto -> espera IDLE -> S,v,a -> P,q1,q2,q3 ... -> E
    %  El firmware tiene una cola de 512 tramos (PLANNER_SIZE, 511 utiles) y
    %  encadena los P sin frenar (look-ahead). La trayectoria puede tener MAS de
    %  512 puntos: se van mandando a medida que la cola se vacia.
    %  Control de flujo: como mucho CONT_VENTANA lineas sin "ok" y como mucho
    %  CONT_PROF_MAX tramos en la cola del STM32 (asi un X nunca queda trabado
    %  detras de un P esperando lugar en la cola).
    %
    %  Cada punto tiene un estado (Cont.Estado):
    %     0 ok | 1 fuera de limites articulares | 2 fuera del alcance | 3 salto grande
    %  Los puntos con problemas se marcan en los graficos 2D y en la vista 3D y
    %  la trayectoria no se puede ejecutar hasta corregirlos.
    %% ---------------------------------------------------------------
    methods (Access = private)
        function tf = ocupado(app)
            tf = app.Ejec.Activa || app.Cont.Activa || app.Ens.Activo;
        end

        % ---------- plano del circulo ----------
        function pl = cont_plano(app)
            pl = string(app.DdContPlano.Value);
        end

        function d = cont_dir_brazo(app, q1)
            % vector horizontal (x,y) del plano del brazo para un q1 dado [deg]
            o1 = 0;
            try, o1 = app.Robot.links(1).offset; catch, end
            d = [cos(deg2rad(q1) + o1) sin(deg2rad(q1) + o1)];
        end

        function P = cont_plano_a_xyz(app, pl, fijo, u, v)
            % u, v [mm] en el plano -> P [m] (Nx3, sistema 0)
            u = u(:); v = v(:);
            switch pl
                case "brazo"
                    d = app.cont_dir_brazo(fijo);          % fijo = q1 [deg]
                    P = [u*d(1), u*d(2), v] / 1000;
                case "xy"
                    P = [u, v, fijo*ones(size(u))] / 1000;
                case "xz"
                    P = [u, fijo*ones(size(u)), v] / 1000;
                otherwise   % "yz"
                    P = [fijo*ones(size(u)), u, v] / 1000;
            end
        end

        function [u, v, fijo] = cont_xyz_a_plano(app, pl, p, q1)
            % p [m] (1x3) -> coordenadas del plano [mm]. Para "brazo" fijo = q1 [deg]
            p = p(:)' * 1000;
            switch pl
                case "brazo"
                    d = app.cont_dir_brazo(q1);
                    u = p(1)*d(1) + p(2)*d(2);  v = p(3);  fijo = q1;
                case "xy", u = p(1); v = p(2); fijo = p(3);
                case "xz", u = p(1); v = p(3); fijo = p(2);
                otherwise, u = p(2); v = p(3); fijo = p(1);
            end
        end

        function q = cont_q_referencia(app)
            % q "actual" para tomar el TCP: la informada si es valida, si no el objetivo
            if app.QValida, q = app.QActual; else, q = app.qObjetivo(); end
        end

        function p = cont_tcp_actual_mm(app)
            p = [];
            if isempty(app.Robot), return; end
            try
                p = app.tcp(app.cont_q_referencia())' * 1000;
            catch
            end
        end

        function cb_cont_plano(app, tomar)
            % nombres de los campos del circulo segun el plano. Con tomar = true
            % (cambio de plano desde la lista) se toma el TCP actual como centro
            switch app.cont_plano()
                case "brazo", nu = 'r radial'; nv = 'z';  nf = 'q1 del plano [deg]';
                case "xy",    nu = 'x';        nv = 'y';  nf = 'z fija [mm]';
                case "xz",    nu = 'x';        nv = 'z';  nf = 'y fija [mm]';
                otherwise,    nu = 'y';        nv = 'z';  nf = 'x fija [mm]';
            end
            app.CircULabel.Text = ['Centro ' nu ' [mm]'];
            app.CircVLabel.Text = ['Centro ' nv ' [mm]'];
            app.ContFijoLabel.Text = nf;
            if tomar, app.cb_cont_tomar("circ"); end
        end

        function cb_cont_tomar(app, que)
            % que = "circ" centro del circulo (y coordenada fija), "p1" / "p2" puntos de la recta
            p = app.cont_tcp_actual_mm();
            if isempty(p), return; end
            switch que
                case "p1"
                    app.R1x.Value = round(p(1),1); app.R1y.Value = round(p(2),1); app.R1z.Value = round(p(3),1);
                case "p2"
                    app.R2x.Value = round(p(1),1); app.R2y.Value = round(p(2),1); app.R2z.Value = round(p(3),1);
                otherwise
                    q = app.cont_q_referencia();
                    pl = app.cont_plano();
                    [u, v, f] = app.cont_xyz_a_plano(pl, p/1000, q(1));
                    app.ContFijoEdit.Value = round(f, 2);
                    app.CircU.Value = round(u, 1);  app.CircV.Value = round(v, 1);
            end
            app.avisar(sprintf('TCP actual = [%.1f  %.1f  %.1f] mm.', p), 'info');
        end

        function cont_defaults(app)
            % valores iniciales: TCP en q = [0 30 60] (con el cero vertical: brazo inclinado 30 deg
            % hacia adelante y antebrazo horizontal, punta en ~[259 0 219] mm)
            % recta vertical de 40 mm hacia abajo y circulo de 30 mm en el plano del brazo
            p = [259 0 219];
            if ~isempty(app.Robot)
                try, p = app.tcp([0 30 60])' * 1000; catch, end
            end
            p = round(p);
            app.R1x.Value = p(1); app.R1y.Value = p(2); app.R1z.Value = p(3);
            app.R2x.Value = p(1); app.R2y.Value = p(2); app.R2z.Value = p(3) - 40;
            app.ContFijoEdit.Value = 0;
            app.CircU.Value = round(hypot(p(1), p(2)));  app.CircV.Value = p(3);
        end

        % ---------- cinematica inversa de toda la trayectoria ----------
        function [q, ok, motivo, qsin] = ik_cerca(app, P, qref, q1fijo)
            % q: solucion de cin_inv_3GDL valida (alcanza P y respeta limites) mas cercana a qref.
            % qsin: la mas cercana que alcanza P SIN mirar limites (para mostrar cuanto se pasa).
            % Con q1fijo (deg) solo acepta soluciones con ese q1 (plano del brazo).
            q = []; ok = false; motivo = "fuera del espacio de trabajo"; qsin = nan(1,3);
            P = P(:);
            T = [eye(3) P; 0 0 0 1];
            Qall = cin_inv_3GDL(app.Robot, T, deg2rad(qref), false);
            lim = deg2rad(app.q_limites);
            tolL = 1e-6;
            mejor = inf; mejorSin = inf;
            for i = 1:size(Qall, 2)
                qi = Qall(:, i)';
                for j = 1:3
                    lo = lim(j,1) - tolL;
                    qi(j) = lo + mod(qi(j) - lo, 2*pi);
                end
                if ~isnan(q1fijo)
                    dq1 = mod(rad2deg(qi(1)) - q1fijo + 180, 360) - 180;
                    if abs(dq1) > 0.05, continue; end
                    qi(1) = deg2rad(q1fijo);
                end
                p = double(app.Robot.fkine(qi));
                if norm(p(1:3,4) - P) > 1e-4, continue; end
                d = norm(qi - deg2rad(qref));
                if d < mejorSin
                    mejorSin = d;  qsin = rad2deg(qi);
                end
                motivo = "fuera de los limites articulares";
                if any(qi' < lim(:,1) - tolL) || any(qi' > lim(:,2) + tolL), continue; end
                if d < mejor
                    mejor = d;
                    q = rad2deg(qi);
                end
            end
            ok = ~isinf(mejor);
            if ok, qsin = q; end
        end

        function [Q, est] = cont_ik(app, P, q1fijo)
            % P [m] Nx3 -> Q [deg] Nx3 (NaN si no alcanza) y estado de cada punto
            N = size(P, 1);
            Q = nan(N, 3); est = zeros(N, 1);
            if isempty(app.Robot)
                est(:) = 2;
                app.alerta('Sin modelo', 'Robot no cargado: ejecuta robot.m y reabri la GUI.');
                return;
            end
            qref = app.cont_q_referencia();
            if ~isnan(q1fijo), qref(1) = q1fijo; end
            dlg = [];
            if N > 300
                try
                    dlg = uiprogressdlg(app.UIFigure, 'Title', 'Cinematica inversa', ...
                        'Message', sprintf('Calculando %d puntos...', N));
                catch
                end
            end
            qant = [];
            try
                for k = 1:N
                    [qk, okk, ~, qs] = app.ik_cerca(P(k,:), qref, q1fijo);
                    if okk
                        Q(k,:) = qk;
                        if ~isempty(qant) && max(abs(qk - qant)) > app.CONT_SALTO_MAX, est(k) = 3; end
                    elseif all(isfinite(qs))
                        Q(k,:) = qs;  est(k) = 1;
                    else
                        est(k) = 2;
                    end
                    if all(isfinite(Q(k,:)))
                        qref = Q(k,:);  qant = Q(k,:);
                    end
                    if ~isempty(dlg) && mod(k, 50) == 0, dlg.Value = k/N; end
                end
            catch ME
                app.alerta('Error en la CI', ME.message);
                est(isnan(Q(:,1))) = 2;
            end
            if ~isempty(dlg), close(dlg); end
        end

        function cont_cargar_cartesiano(app, P, q1fijo, desc, crit, stop)
            if nargin < 5, crit = []; end
            if nargin < 6, stop = []; end
            [Q, est] = app.cont_ik(P, q1fijo);
            app.cont_set(Q, P, desc, est, crit, stop);
        end

        function cont_set(app, Q, P, desc, est, crit, stop)
            % crit = puntos de PASO: el brazo pasa si o si por ahi (no se mueven al suavizar)
            % stop = puntos de PARADA: ademas de no moverse, el robot frena y se queda quieto ahi
            if nargin < 5 || isempty(est), est = zeros(size(Q,1), 1); end
            if nargin < 6, crit = []; end
            if nargin < 7, stop = []; end
            crit = unique(round(crit(:)'));
            crit = crit(crit >= 1 & crit <= size(Q,1));
            stop = unique(round(stop(:)'));
            stop = stop(stop >= 1 & stop <= size(Q,1));
            crit = setdiff(crit, stop);              % si un punto es de parada no hace falta que tambien sea de paso
            app.Cont.Crit = crit;
            app.Cont.Stop = stop;
            app.SuavCrit.Value = app.crit_texto(crit);
            app.SuavStop.Value = app.crit_texto(stop);
            app.Cont.Q = Q;
            app.Cont.P = P;
            app.Cont.Estado = est(:);
            app.Cont.Desc = string(desc);
            app.cont_preview();
            app.cont_info();
            app.actualizar_botones();
            n1 = nnz(est == 1); n2 = nnz(est == 2); n3 = nnz(est == 3);
            if n1 + n2 + n3 == 0
                app.avisar(sprintf('Trayectoria lista: %s (%d puntos). No se envio nada.', desc, size(Q,1)), 'ok');
            else
                app.avisar(sprintf(['%s: %d fuera de limites articulares (naranja), %d fuera del alcance (rojo), ' ...
                    '%d con saltos grandes (violeta). Mira los graficos y ajusta los parametros.'], ...
                    desc, n1, n2, n3), 'error');
            end
        end

        % ---------- generadores ----------
        function cb_rect3d_generar(app)
            if app.ocupado(), return; end
            P1 = [app.R1x.Value app.R1y.Value app.R1z.Value];
            P2 = [app.R2x.Value app.R2y.Value app.R2z.Value];
            L = norm(P2 - P1);
            if L < 0.5
                app.avisar('P1 y P2 estan a menos de 0.5 mm: no hay recta.', 'aviso'); return;
            end
            n = max(2, ceil(L / app.RectSep.Value));
            if n > app.CONT_PUNTOS_MAX
                app.alerta('Demasiados puntos', sprintf('Salen %d puntos (maximo %d): agranda la separacion.', n, app.CONT_PUNTOS_MAX));
                return;
            end
            s = linspace(0, 1, n+1)';
            P = (P1 + s .* (P2 - P1)) / 1000;
            desc = sprintf('recta [%.0f %.0f %.0f] -> [%.0f %.0f %.0f] mm (%.1f mm)', P1, P2, L);
            app.cont_cargar_cartesiano(P, NaN, desc);
        end

        function cb_circ_generar(app)
            if app.ocupado(), return; end
            pl = app.cont_plano();
            R = app.CircR.Value;  sep = app.CircSep.Value;  nv = app.CircVueltas.Value;
            sgn = 1;
            if startsWith(string(app.CircSentido.Value), "Horario"), sgn = -1; end
            n = max(12, ceil(2*pi*R*nv / sep));
            if n > app.CONT_PUNTOS_MAX
                app.alerta('Demasiados puntos', sprintf('Salen %d puntos (maximo %d): agranda la separacion.', n, app.CONT_PUNTOS_MAX));
                return;
            end
            t = deg2rad(app.CircAng0.Value) + sgn * linspace(0, 2*pi*nv, n+1)';
            u = app.CircU.Value + R*cos(t);
            v = app.CircV.Value + R*sin(t);
            P = app.cont_plano_a_xyz(pl, app.ContFijoEdit.Value, u, v);
            q1f = NaN;
            if pl == "brazo", q1f = app.ContFijoEdit.Value; end
            desc = sprintf('circulo R=%g mm en plano %s (%g vueltas)', R, pl, nv);
            app.cont_cargar_cartesiano(P, q1f, desc);
        end

        % ---------- archivos .mat ----------
        function cb_cont_cargar(app)
            if app.ocupado(), return; end
            [f, p] = uigetfile('*.mat', 'Trayectoria de puntos densos', app.carpeta_defecto());
            figure(app.UIFigure);
            if isequal(f, 0), return; end
            archivo = fullfile(p, f);
            try
                S = load(archivo);
            catch ME
                app.alerta('Error al abrir', ME.message); return;
            end
            if isfield(S, 'biblioteca')
                app.alerta('Archivo de biblioteca', ...
                    'Ese .mat es una biblioteca de puntos sueltos: abrila en la pestana Trayectorias.');
                return;
            end
            % variable a usar: por nombre y si no la primera matriz numerica de 2 o 3 columnas
            nombre = "";
            for c = ["Q","q","qs","tray","trayectoria","puntos","xyz","P"]
                c = char(c);
                if isfield(S, c) && isnumeric(S.(c)) && ismatrix(S.(c)) && numel(S.(c)) >= 4
                    nombre = string(c); break;
                end
            end
            if nombre == ""
                fn = fieldnames(S);
                for k = 1:numel(fn)
                    x = S.(fn{k});
                    if isnumeric(x) && ismatrix(x) && any(size(x) == 2 | size(x) == 3) && numel(x) >= 4
                        nombre = string(fn{k}); break;
                    end
                end
            end
            if nombre == ""
                app.alerta('Formato no reconocido', ...
                    'No hay ninguna matriz Nx3 (q1 q2 q3 o x y z) ni Nx2 (q2 q3) en el archivo.');
                return;
            end
            M = double(S.(char(nombre)));
            if ~any(size(M,2) == [2 3]) && any(size(M,1) == [2 3]), M = M.'; end
            if ~any(size(M,2) == [2 3]) || size(M,1) < 2 || any(~isfinite(M(:)))
                app.alerta('Formato no reconocido', sprintf('La variable "%s" tiene que ser Nx3 o Nx2 sin NaN (es %dx%d).', ...
                    nombre, size(M,1), size(M,2)));
                return;
            end
            if size(M,1) > app.CONT_PUNTOS_MAX
                app.alerta('Demasiados puntos', sprintf('%d puntos (maximo %d).', size(M,1), app.CONT_PUNTOS_MAX));
                return;
            end

            cont = string(app.DdContContenido.Value);
            if nombre == "Q" && isfield(S, 'descripcion'), cont = "deg"; end    % guardado por esta GUI
            crit = [];  stop = [];
            if isfield(S, 'criticos') && isnumeric(S.criticos), crit = double(S.criticos(:)'); end
            if isfield(S, 'paradas') && isnumeric(S.paradas), stop = double(S.paradas(:)'); end
            desc = sprintf('%s (variable %s, %d puntos)', f, nombre, size(M,1));
            if any(cont == ["deg","rad"])
                if cont == "rad", M = rad2deg(M); end
                if size(M,2) == 2
                    q1 = app.cont_q_referencia(); q1 = q1(1);
                    M = [q1*ones(size(M,1),1) M];
                    desc = sprintf('%s, q1 fijo = %.1f', desc, q1);
                end
                fuera = any(M < app.q_limites(:,1)' - 1e-6 | M > app.q_limites(:,2)' + 1e-6, 2);
                est = double(fuera);                       % 1 = fuera de limites
                salto = max(max(abs(diff(M)), [], 2));
                P = nan(size(M,1), 3);
                if ~isempty(app.Robot)
                    for k = 1:size(M,1), P(k,:) = app.fk_xyz(M(k,:)); end
                end
                app.cont_set(M, P, desc, est, crit, stop);
                if salto > app.CONT_SALTO_MAX && ~any(fuera)
                    app.avisar(sprintf(['Ojo: hay saltos de hasta %.1f deg entre puntos seguidos. Entre esos puntos ' ...
                        'el robot va en linea recta en q (no en el espacio).'], salto), 'aviso');
                end
            else
                % XYZ: Nx3 = puntos en el sistema 0, Nx2 = (u,v) en el plano del circulo
                esc = 1;
                if cont == "mm", esc = 1/1000; end
                q1f = NaN;
                if size(M,2) == 3
                    P = M * esc;
                else
                    pl = app.cont_plano();
                    P = app.cont_plano_a_xyz(pl, app.ContFijoEdit.Value, M(:,1)*esc*1000, M(:,2)*esc*1000);
                    if pl == "brazo", q1f = app.ContFijoEdit.Value; end
                    desc = sprintf('%s en plano %s', desc, pl);
                end
                app.cont_cargar_cartesiano(P, q1f, desc, crit, stop);
            end
            app.LblContArchivo.Text = ['Archivo: ' archivo];
        end

        function cb_cont_guardar(app)
            if isempty(app.Cont.Q)
                app.avisar('No hay trayectoria continua para guardar.', 'aviso'); return;
            end
            [f, p] = uiputfile('*.mat', 'Guardar trayectoria continua', ...
                fullfile(app.carpeta_defecto(), 'trayectoria_continua.mat'));
            figure(app.UIFigure);
            if isequal(f, 0), return; end
            Q = app.Cont.Q;                         %#ok<NASGU>  q1 q2 q3 en grados (DH)
            xyz = app.Cont.P;                       %#ok<NASGU>  TCP en metros
            descripcion = char(app.Cont.Desc);      %#ok<NASGU>
            criticos = app.Cont.Crit;               %#ok<NASGU>  puntos de paso (el brazo pasa si o si por ahi)
            paradas = app.Cont.Stop;                %#ok<NASGU>  puntos de parada (frena y espera)
            try
                save(fullfile(p, f), 'Q', 'xyz', 'descripcion', 'criticos', 'paradas');
                app.LblContArchivo.Text = ['Archivo: ' fullfile(p, f)];
                app.avisar(['Guardado ' fullfile(p, f) ' (variable Q en grados).'], 'ok');
            catch ME
                app.alerta('Error al guardar', ME.message);
            end
        end

        function cb_cont_limpiar(app)
            if app.ocupado(), return; end
            app.Cont.Q = zeros(0,3);
            app.Cont.P = zeros(0,3);
            app.Cont.Estado = zeros(0,1);
            app.Cont.Crit = zeros(1,0);
            app.Cont.Stop = zeros(1,0);
            app.SuavCrit.Value = '';
            app.SuavStop.Value = '';
            app.Cont.Desc = "";
            app.cont_preview();
            app.cont_info();
            app.actualizar_botones();
        end

        % ---------- graficos 2D: espacio de trabajo, limites y trayectoria ----------
        function cont_ws_dibujar(app)
            % Dibuja una vez: zona alcanzable (con los limites de q2 y q3), limites
            % articulares y los objetos de la trayectoria (vacios)
            H = struct();
            H.qt = gobjects(1,3);  H.qtLim = gobjects(1,3);
            lim = app.q_limites;
            nar = [0.93 0.55 0.10]; roj = [0.85 0.10 0.10]; vio = [0.55 0.20 0.75];
            azu = [0.00 0.45 0.74]; ver = [0.20 0.62 0.25];

            % --- plano XZ: CORTE del espacio de trabajo en y = y0 (respeta los limites de q1, q2 y q3) ---
            ax = app.AxLat;  cla(ax);  hold(ax, 'on');  grid(ax, 'on');  axis(ax, 'equal');
            vws = [0.70 0.88 0.70];
            H.latWs = plot(ax, nan, nan, '.', 'Color', vws, 'MarkerSize', 4);
            plot(ax, [0 0], [-250 420], ':', 'Color', [0.5 0.5 0.5]);   % eje de la base
            H.latRob = plot(ax, nan, nan, '-o', 'Color', [0.6 0.6 0.6], 'LineWidth', 2, 'MarkerSize', 4);
            H.latPath = plot(ax, nan, nan, '-', 'Color', azu, 'LineWidth', 1.5);
            H.latB1 = plot(ax, nan, nan, 'o', 'Color', nar, 'MarkerSize', 5, 'LineWidth', 1.2);
            H.latB2 = plot(ax, nan, nan, 'x', 'Color', roj, 'MarkerSize', 7, 'LineWidth', 1.5);
            H.latB3 = plot(ax, nan, nan, 's', 'Color', vio, 'MarkerSize', 6, 'LineWidth', 1.2);
            H.latIni = plot(ax, nan, nan, 'o', 'Color', ver, 'MarkerFaceColor', ver, 'MarkerSize', 6);
            H.latCrit = plot(ax, nan, nan, 'd', 'Color', [0 0 0], 'MarkerFaceColor', [1 0.85 0], 'MarkerSize', 8);
            H.latStop = plot(ax, nan, nan, 's', 'Color', [0 0 0], 'MarkerFaceColor', [0.90 0.15 0.15], 'MarkerSize', 9);
            title(ax, 'Plano XZ'); xlabel(ax, 'x [mm]'); ylabel(ax, 'z [mm]');

            % --- plano XY: CORTE del espacio de trabajo en z = z0 ---
            ax = app.AxSup;  cla(ax);  hold(ax, 'on');  grid(ax, 'on');  axis(ax, 'equal');
            H.supWs = plot(ax, nan, nan, '.', 'Color', vws, 'MarkerSize', 4);
            plot(ax, 0, 0, 'k+');
            H.supRob = plot(ax, nan, nan, '-o', 'Color', [0.6 0.6 0.6], 'LineWidth', 2, 'MarkerSize', 4);
            H.supPath = plot(ax, nan, nan, '-', 'Color', azu, 'LineWidth', 1.5);
            H.supB1 = plot(ax, nan, nan, 'o', 'Color', nar, 'MarkerSize', 5, 'LineWidth', 1.2);
            H.supB2 = plot(ax, nan, nan, 'x', 'Color', roj, 'MarkerSize', 7, 'LineWidth', 1.5);
            H.supB3 = plot(ax, nan, nan, 's', 'Color', vio, 'MarkerSize', 6, 'LineWidth', 1.2);
            H.supIni = plot(ax, nan, nan, 'o', 'Color', ver, 'MarkerFaceColor', ver, 'MarkerSize', 6);
            H.supCrit = plot(ax, nan, nan, 'd', 'Color', [0 0 0], 'MarkerFaceColor', [1 0.85 0], 'MarkerSize', 8);
            H.supStop = plot(ax, nan, nan, 's', 'Color', [0 0 0], 'MarkerFaceColor', [0.90 0.15 0.15], 'MarkerSize', 9);
            title(ax, 'Plano XY'); xlabel(ax, 'x [mm]'); ylabel(ax, 'y [mm]');

            % --- espacio articular q2-q3 ---
            ax = app.AxQ23;  cla(ax);  hold(ax, 'on');  grid(ax, 'on');
            patch(ax, lim(2,[1 2 2 1]), lim(3,[1 1 2 2]), [0.88 0.95 0.88], 'EdgeColor', [0.3 0.6 0.3]);
            H.q23Path = plot(ax, nan, nan, '-', 'Color', azu, 'LineWidth', 1.5);
            H.q23B1 = plot(ax, nan, nan, 'o', 'Color', nar, 'MarkerSize', 5, 'LineWidth', 1.2);
            H.q23B3 = plot(ax, nan, nan, 's', 'Color', vio, 'MarkerSize', 6, 'LineWidth', 1.2);
            H.q23Ini = plot(ax, nan, nan, 'o', 'Color', ver, 'MarkerFaceColor', ver, 'MarkerSize', 6);
            H.q23Crit = plot(ax, nan, nan, 'd', 'Color', [0 0 0], 'MarkerFaceColor', [1 0.85 0], 'MarkerSize', 8);
            H.q23Stop = plot(ax, nan, nan, 's', 'Color', [0 0 0], 'MarkerFaceColor', [0.90 0.15 0.15], 'MarkerSize', 9);
            H.q23Rob = plot(ax, nan, nan, 's', 'Color', [0.5 0.5 0.5], 'MarkerFaceColor', [0.6 0.6 0.6], 'MarkerSize', 7);
            m = 0.15 * [diff(lim(2,:)) diff(lim(3,:))];
            xlim(ax, lim(2,:) + [-1 1]*m(1));  ylim(ax, lim(3,:) + [-1 1]*m(2));
            title(ax, 'Articular q2 - q3 (verde = limites)'); xlabel(ax, 'q2 [deg]'); ylabel(ax, 'q3 [deg]');

            % --- q1, q2, q3 a lo largo de la trayectoria ---
            ax = app.AxQt;  cla(ax);  hold(ax, 'on');  grid(ax, 'on');
            col = [0.00 0.45 0.74; 0.85 0.33 0.10; 0.47 0.67 0.19];
            for j = 1:3
                H.qt(j) = plot(ax, nan, nan, '-', 'Color', col(j,:), 'LineWidth', 1.3);
            end
            for j = 1:3
                H.qtLim(j) = plot(ax, nan, nan, '--', 'Color', col(j,:));
            end
            H.qtB = plot(ax, nan, nan, 'x', 'Color', roj, 'MarkerSize', 6, 'LineWidth', 1.3);
            H.qtCrit = plot(ax, nan, nan, 'd', 'Color', [0 0 0], 'MarkerFaceColor', [1 0.85 0], 'MarkerSize', 6);
            H.qtStop = plot(ax, nan, nan, 's', 'Color', [0 0 0], 'MarkerFaceColor', [0.90 0.15 0.15], 'MarkerSize', 7);
            legend(ax, H.qt, {'q1','q2','q3'}, 'Location', 'best', 'FontSize', 7);
            title(ax, 'q vs punto (-- limites)'); xlabel(ax, 'punto'); ylabel(ax, '[deg]');
            H.corteY = NaN;  H.corteZ = NaN;
            app.H2 = H;
            app.cont_preview();
        end

        function [r, z] = cont_ws_muestras(app, n)
            % zona alcanzable en el plano del brazo (q1 = 0) barriendo q2 y q3 dentro de los limites [mm]
            % r = distancia horizontal al eje de la base a lo largo del brazo (con signo), z = altura
            if nargin < 2, n = 70; end
            r = []; z = [];
            if isempty(app.Robot), return; end
            L = app.Robot.links;
            o = zeros(1,3);
            for j = 1:3, try, o(j) = L(j).offset; catch, end, end
            [Q2, Q3] = meshgrid(deg2rad(linspace(app.q_limites(2,1), app.q_limites(2,2), n)), ...
                                deg2rad(linspace(app.q_limites(3,1), app.q_limites(3,2), n)));
            t2 = Q2 + o(2);  t23 = t2 + Q3 + o(3);
            X1 = L(2).a*cos(t2) + L(3).a*cos(t23);
            Y1 = L(2).a*sin(t2) + L(3).a*sin(t23);
            r = (L(1).a + X1(:)) * 1000;
            z = (L(1).d + sin(L(1).alpha)*Y1(:)) * 1000;
        end

        function ok = cont_q1_ok(app, x, y, r)
            % el punto (x,y) con el brazo a distancia r (con signo) necesita q1 = atan2(y,x)
            % (o el opuesto si r < 0). true si ese q1 entra en los limites de q1
            o1 = 0;
            try, o1 = app.Robot.links(1).offset; catch, end
            a = atan2(y, x);
            a(r < 0) = atan2(-y(r < 0), -x(r < 0));
            q1 = rad2deg(a - o1);
            lo = app.q_limites(1,1);  hi = app.q_limites(1,2);
            q1 = lo + mod(q1 - lo, 360);
            ok = q1 <= hi + 1e-6;
        end

        function [xs, zs] = cont_ws_corte_xz(app, y0)
            % puntos (x,z) alcanzables con y = y0 [mm] respetando los limites de q1, q2 y q3
            [r, z] = app.cont_ws_muestras(130);
            xs = []; zs = [];
            k = abs(r) >= abs(y0);
            r = r(k); z = z(k);
            x1 = sqrt(r.^2 - y0^2);
            X = [x1; -x1]; Z = [z; z]; Rr = [r; r];
            ok = app.cont_q1_ok(X, y0*ones(size(X)), Rr);
            xs = X(ok); zs = Z(ok);
        end

        function [xs, ys] = cont_ws_corte_xy(app, z0)
            % puntos (x,y) alcanzables con z = z0 [mm] respetando los limites de q1, q2 y q3
            [r, z] = app.cont_ws_muestras(160);
            xs = []; ys = [];
            ru = unique(round(r(abs(z - z0) < 2) / 2) * 2);      % radios que llegan a esa altura
            if isempty(ru), return; end
            o1 = 0;
            try, o1 = app.Robot.links(1).offset; catch, end
            a = deg2rad(linspace(app.q_limites(1,1), app.q_limites(1,2), 120)) + o1;
            X = ru * cos(a);  Y = ru * sin(a);
            xs = X(:); ys = Y(:);
        end

        function cont_ws_cortes(app, y0, z0)
            % recalcula los cortes de XZ (en y0) y XY (en z0) solo si cambiaron
            H = app.H2;
            if isempty(fieldnames(H)) || isempty(app.Robot), return; end
            y0 = round(y0); z0 = round(z0);
            if ~isequal(H.corteY, y0)
                [xs, zs] = app.cont_ws_corte_xz(y0);
                set(H.latWs, 'XData', xs, 'YData', zs);
                title(app.AxLat, sprintf('Plano XZ (corte en y = %d mm)', y0));
                H.corteY = y0;
            end
            if ~isequal(H.corteZ, z0)
                [xs, ys] = app.cont_ws_corte_xy(z0);
                set(H.supWs, 'XData', xs, 'YData', ys);
                title(app.AxSup, sprintf('Plano XY (corte en z = %d mm)', z0));
                H.corteZ = z0;
            end
            app.H2 = H;
        end

        function cont_preview(app)
            % trayectoria y violaciones en la vista 3D y en los 4 graficos 2D
            Q = app.Cont.Q;  P = app.Cont.P;  est = app.Cont.Estado;
            hay = ~isempty(Q);
            if isfield(app.H3, 'cont')
                if hay
                    app.set3(app.H3.cont, P);
                    app.set3(app.H3.contIni, P(1,:));
                    if any(est ~= 0), app.set3(app.H3.contBad, P(est ~= 0, :));
                    else, app.set3(app.H3.contBad, nan(1,3)); end
                    if ~isempty(app.Cont.Crit), app.set3(app.H3.contCrit, P(app.Cont.Crit, :));
                    else, app.set3(app.H3.contCrit, nan(1,3)); end
                    if ~isempty(app.Cont.Stop), app.set3(app.H3.contStop, P(app.Cont.Stop, :));
                    else, app.set3(app.H3.contStop, nan(1,3)); end
                else
                    app.set3(app.H3.cont, nan(1,3));
                    app.set3(app.H3.contIni, nan(1,3));
                    app.set3(app.H3.contBad, nan(1,3));
                    app.set3(app.H3.contCrit, nan(1,3));
                    app.set3(app.H3.contStop, nan(1,3));
                end
            end
            H = app.H2;
            if isempty(fieldnames(H)), return; end
            nn = @(h) set(h, 'XData', nan, 'YData', nan);
            sx = @(h, x, y) set(h, 'XData', x, 'YData', y);
            if ~hay
                p0 = app.cont_tcp_actual_mm();
                if isempty(p0), p0 = [0 0 0]; end
                app.cont_ws_cortes(p0(2), p0(3));       % sin trayectoria: cortes por el TCP actual
                for h = [H.latPath H.latB1 H.latB2 H.latB3 H.latIni H.supPath H.supB1 H.supB2 H.supB3 ...
                         H.supIni H.q23Path H.q23B1 H.q23B3 H.q23Ini H.qt H.qtLim H.qtB ...
                         H.latCrit H.supCrit H.q23Crit H.qtCrit H.latStop H.supStop H.q23Stop H.qtStop]
                    nn(h);
                end
                app.cont_preview_robot();
                return;
            end
            Pm = P * 1000;
            ok = all(isfinite(Pm), 2);
            if any(ok)
                % cortes por la trayectoria (y medio para XZ, z medio para XY)
                app.cont_ws_cortes(mean(Pm(ok,2)), mean(Pm(ok,3)));
            end
            b1 = est == 1; b2 = est == 2; b3 = est == 3;
            sx(H.latPath, Pm(:,1), Pm(:,3));  sx(H.latIni, Pm(1,1), Pm(1,3));
            sx(H.latB1, Pm(b1,1), Pm(b1,3)); sx(H.latB2, Pm(b2,1), Pm(b2,3)); sx(H.latB3, Pm(b3,1), Pm(b3,3));
            sx(H.supPath, Pm(:,1), Pm(:,2));  sx(H.supIni, Pm(1,1), Pm(1,2));
            sx(H.supB1, Pm(b1,1), Pm(b1,2)); sx(H.supB2, Pm(b2,1), Pm(b2,2)); sx(H.supB3, Pm(b3,1), Pm(b3,2));
            sx(H.q23Path, Q(:,2), Q(:,3));  sx(H.q23Ini, Q(1,2), Q(1,3));
            sx(H.q23B1, Q(b1,2), Q(b1,3)); sx(H.q23B3, Q(b3,2), Q(b3,3));
            c = app.Cont.Crit;
            sx(H.latCrit, Pm(c,1), Pm(c,3));  sx(H.supCrit, Pm(c,1), Pm(c,2));
            sx(H.q23Crit, Q(c,2), Q(c,3));
            sx(H.qtCrit, [c c c], [Q(c,1)' Q(c,2)' Q(c,3)']);
            c = app.Cont.Stop;
            sx(H.latStop, Pm(c,1), Pm(c,3));  sx(H.supStop, Pm(c,1), Pm(c,2));
            sx(H.q23Stop, Q(c,2), Q(c,3));
            sx(H.qtStop, [c c c], [Q(c,1)' Q(c,2)' Q(c,3)']);
            k = (1:size(Q,1))';
            for j = 1:3
                sx(H.qt(j), k, Q(:,j));
                sx(H.qtLim(j), [1 size(Q,1) NaN 1 size(Q,1)], ...
                    [app.q_limites(j,1) app.q_limites(j,1) NaN app.q_limites(j,2) app.q_limites(j,2)]);
            end
            bad = find(est ~= 0);
            if isempty(bad)
                nn(H.qtB);
            else
                % marca en el grafico q vs punto el q de cada articulacion que se pasa
                xb = []; yb = [];
                for j = 1:3
                    f = bad(Q(bad,j) < app.q_limites(j,1) - 1e-6 | Q(bad,j) > app.q_limites(j,2) + 1e-6);
                    xb = [xb; f]; yb = [yb; Q(f,j)]; %#ok<AGROW>
                end
                sx(H.qtB, xb, yb);
            end
            app.cont_preview_robot();
        end

        function cont_preview_robot(app)
            % pose actual del robot (gris) en los planos XZ, XY y q2-q3
            H = app.H2;
            if isempty(fieldnames(H)) || isempty(app.Robot), return; end
            try
                q = app.cont_q_referencia();
                V = app.vertices_robot(q) * 1000;
                set(H.latRob, 'XData', V(:,1), 'YData', V(:,3));
                set(H.supRob, 'XData', V(:,1), 'YData', V(:,2));
                set(H.q23Rob, 'XData', q(2), 'YData', q(3));
            catch
            end
        end

        % ---------- info y secuencia a ejecutar ----------
        function [Qe, Pe, Me] = cont_secuencia(app)
            % lo que realmente se manda: ida (y vuelta) repetido N veces.
            % Me marca los puntos criticos dentro de esa secuencia (en la vuelta tambien)
            Q = app.Cont.Q;  P = app.Cont.P;
            M = false(size(Q,1), 1);  M(app.Cont.Stop) = true;        % donde frena
            Qe = Q;  Pe = P;  Me = M;
            if isempty(Q), return; end
            if app.ChkContIdaVuelta.Value
                Qe = [Q; flipud(Q(1:end-1,:))];
                Pe = [P; flipud(P(1:end-1,:))];
                Me = [M; flipud(M(1:end-1))];
            end
            Q1 = Qe;  P1 = Pe;  M1 = Me;
            for k = 2:round(app.ContRep.Value)
                if app.ChkContIdaVuelta.Value
                    Qe = [Qe; Q1(2:end,:)]; Pe = [Pe; P1(2:end,:)]; Me = [Me; M1(2:end)]; %#ok<AGROW>
                else
                    Qe = [Qe; Q1]; Pe = [Pe; P1]; Me = [Me; M1];   %#ok<AGROW>  vuelve del final al inicio en linea recta en q
                end
            end
        end

        function k = cont_paradas(app, Me)
            % numeros de punto (de la secuencia que se manda) donde el robot frena
            k = zeros(1, 0);
            if ~app.ChkFrenarCrit.Value || isempty(Me), return; end
            k = find(Me(:))';
            k = k(k > 1 & k < numel(Me));        % en el inicio y el final ya frena solo
        end

        function [v, T, Tbloque] = cont_velocidades(app, Q, P)
            % velocidad articular (deg/s) de cada tramo: constante, o calculada para que el
            % TCP vaya a velocidad constante. T = tiempo estimado, Tbloque = tramo mas lento
            N = size(Q,1);
            dq = sqrt(sum(diff(Q).^2, 2));
            if startsWith(string(app.DdContVelModo.Value), "TCP") && all(isfinite(P(:)))
                dp = sqrt(sum(diff(P).^2, 2)) * 1000;     % mm
                v = app.ContVel.Value * dq ./ max(dp, 1e-6);
                v(dp < 1e-6) = app.ContVel.Value;
                v = min(max(v, 0.2), app.CONT_V_ART_MAX);
            else
                v = app.ContVel.Value * ones(max(N-1,0), 1);
            end
            T = sum(dq ./ v);
            Tbloque = max([dq ./ v; 0]);
        end

        function cont_info(app)
            Q = app.Cont.Q;
            if isempty(Q)
                app.LblContInfo.Text = 'Sin trayectoria: genera una recta / circulo o carga un .mat.';
                app.LblContInfo.FontColor = [0.35 0.35 0.35];
                return;
            end
            est = app.Cont.Estado;
            if any(est ~= 0)
                app.LblContInfo.Text = sprintf(['%s\nNO SE PUEDE EJECUTAR: %d fuera de limites (o naranja), %d fuera del alcance (x roja), ' ...
                    '%d saltos grandes (cuadrado violeta).'], app.Cont.Desc, nnz(est==1), nnz(est==2), nnz(est==3));
                app.LblContInfo.FontColor = [0.8 0 0];
                return;
            end
            [Qe, Pe] = app.cont_secuencia();
            [~, T] = app.cont_velocidades(Qe, Pe);
            Lp = sum(sqrt(sum(diff(app.Cont.P).^2, 2))) * 1000;
            txt = sprintf(['%s\n%d puntos (%d con ida/vuelta y repeticiones) | q1 [%.1f, %.1f] q2 [%.1f, %.1f] q3 [%.1f, %.1f] deg\n' ...
                'TCP %.0f mm por pasada | tiempo aprox. %.1f s (%.0f puntos/s)'], ...
                app.Cont.Desc, size(Q,1), size(Qe,1), min(Q(:,1)), max(Q(:,1)), min(Q(:,2)), max(Q(:,2)), ...
                min(Q(:,3)), max(Q(:,3)), Lp, T, size(Qe,1)/max(T, 1e-3));
            if size(Qe,1)/max(T,1e-3) > app.CONT_PPS_MAX
                txt = [txt sprintf(' | OJO: mas de %d puntos/s, el serie no llega: agranda la separacion o baja la velocidad', app.CONT_PPS_MAX)];
            end
            if ~isempty(app.Cont.Crit)
                txt = [txt sprintf(' | %d de paso (rombos amarillos)', numel(app.Cont.Crit))];
            end
            if ~isempty(app.Cont.Stop)
                txt = [txt sprintf(' | %d de parada (cuadrados rojos)', numel(app.Cont.Stop))];
                [~, ~, Me] = app.cont_secuencia();
                np = numel(app.cont_paradas(Me));
                if np > 0
                    txt = [txt sprintf(', frena %d veces (pausa %.1f s c/u)', np, app.ContPausa.Value)];
                end
            end
            app.LblContInfo.Text = txt;
            app.LblContInfo.FontColor = [0 0.35 0];
        end

        % ---------- ejecucion ----------
        function cb_cont_ejecutar(app, soloIr)
            if app.ocupado(), return; end
            if ~app.conectado()
                app.alerta('Sin conexion', 'No hay conexion con el STM32.'); return;
            end
            if isempty(app.Cont.Q)
                app.avisar('No hay trayectoria continua: genera o carga una.', 'aviso'); return;
            end
            if any(app.Cont.Estado ~= 0) || any(~isfinite(app.Cont.Q(:)))
                app.alerta('Trayectoria no valida', ['Hay puntos fuera de limites, fuera del alcance o con saltos ' ...
                    '(marcados en los graficos). Ajusta los parametros y volve a generar.']);
                return;
            end
            [Q, P, Me] = app.cont_secuencia();
            app.Cont.Qe = Q;  app.Cont.Pe = P;      % secuencia enviada (para el registro)
            paradas = app.cont_paradas(Me);
            app.Cont.ParadasIdx = paradas;          % para el registro / ventana de torques
            fuera = Q < app.q_limites(:,1)' - 1e-6 | Q > app.q_limites(:,2)' + 1e-6;
            if any(fuera(:))
                app.alerta('Fuera de limites', 'La trayectoria tiene puntos fuera de los limites articulares de la GUI.'); return;
            end
            if ~app.robot_listo_para_mover(), return; end

            % lineas del stream: S (y S nuevos si cambia la velocidad), P..., E
            % si hay que frenar en un punto critico: ... P(critico), E, #PAUSA, S, P ...
            % el E hace que el firmware frene justo en ese punto (rampa de bajada), #PAUSA no se
            % manda: la GUI espera que quede quieto, espera la pausa y arranca otro tramo con S
            % (rampa de subida desde cero)
            [v, T, Tb] = app.cont_velocidades(Q, P);
            acc = app.ContAcc.Value;
            N = size(Q,1);
            corte = false(1, N);  corte(paradas) = true;

            % frenado en las paradas: en los tramos que quedan dentro de la distancia de frenado
            % se usa la aceleracion a = v / t, asi de velocidad constante a 0 tarda t segundos
            % (el firmware arma el trapecio con la aceleracion de cada tramo)
            tf = app.ContTFreno.Value;
            accK = acc * ones(1, N);
            if tf > 0 && ~isempty(paradas)
                tfe = max(tf - app.SCURVE_S, 0.05);             % el perfil en S suma ~50 ms a la frenada
                dq = [0; sqrt(sum(diff(Q).^2, 2))];             % largo del tramo que llega a cada punto
                for s0 = paradas
                    kk = find(corte(1:s0-1), 1, 'last');        % parada anterior (o inicio)
                    if isempty(kk), kk = 1; end
                    dist = 0;                                   % distancia (en q) hasta la parada
                    for k = s0:-1:kk+1
                        vk = v(k-1);
                        dfreno = vk * tfe / 2;                  % lo que recorre frenando de vk a 0 en tfe
                        if dist >= dfreno, break; end
                        accK(k) = vk / tfe;
                        dist = dist + dq(k);
                    end
                end
            end

            nmax = 2*N + 3*numel(paradas) + 2;
            lin = cell(1, nmax);  esP = false(1, nmax);  n = 0;
            vact = NaN;  aact = NaN;
            for k = 1:N
                vk = v(max(k-1, 1));
                ak = accK(k);
                if isnan(vact) || abs(vk - vact) > 0.03*vact || abs(ak - aact) > 0.03*aact
                    n = n + 1; lin{n} = sprintf('S,%.2f,%.1f', vk, ak);
                    vact = vk;  aact = ak;
                end
                n = n + 1; lin{n} = sprintf('P,%.3f,%.3f,%.3f', Q(k,:)); esP(n) = true;
                if corte(k)
                    n = n + 1; lin{n} = 'E';
                    n = n + 1; lin{n} = '#PAUSA';
                    vact = NaN;  aact = NaN;         % el tramo siguiente arranca con su propio S
                end
            end
            n = n + 1; lin{n} = 'E';
            lin = lin(1:n);  esP = esP(1:n);
            T = T + numel(paradas) * (app.ContPausa.Value + 1);   % cada parada: frenar, pausa, arrancar

            app.Cont.Activa = true;
            app.Cont.SoloIr = soloIr;
            app.Cont.Lineas = lin;
            app.Cont.EsP = esP;
            app.Cont.NPuntos = N;
            app.Cont.TEst = T;
            app.Cont.TRx = app.TIMEOUT_RX + 2*Tb;
            app.Cont.NParadas = numel(paradas);
            app.Cont.ParadaAct = 0;
            app.actualizar_botones();

            if norm(app.QActual - Q(1,:)) > 0.2 || soloIr
                % primero un G al punto inicial (con la velocidad P2P del control manual)
                vg = app.VelEditField.Value;
                app.Cont.Fase = "ir_inicio";
                app.Cont.Ack = false;
                app.Cont.T0 = tic;
                app.Cont.TMax = 10 + 3*norm(app.QActual - Q(1,:))/max(vg, 0.1);
                app.cont_ui();
                if ~app.enviar(sprintf("G,%.3f,%.3f,%.3f,%.2f", Q(1,:), vg)), return; end
                app.avisar('Yendo al punto inicial de la trayectoria continua...', 'info');
            else
                app.cont_iniciar_stream();
            end
        end

        function cont_iniciar_stream(app)
            app.Cont.Fase = "stream";
            app.Cont.Idx = 1;
            app.Cont.Inflight = 0;
            app.Cont.Recibidos = 0;
            app.Cont.AckP = 0;
            app.Cont.Libres = NaN;
            app.Cont.LibresMax = NaN;
            app.Cont.TOk = tic;
            app.Cont.T0 = tic;
            app.Cont.TInicio = tic;
            app.Cont.Sent = false(1, 0);            % que lineas salieron (true = P), para contar los ok
            app.Cont.EsperaIdle = false;
            app.Cont.Parado = false;
            app.Cont.TPausa = [];
            app.reg_iniciar();                      % registro de referencia y encoders
            app.avisar(sprintf('Trayectoria continua en ejecucion (%d puntos, ~%.0f s).', app.Cont.NPuntos, app.Cont.TEst), 'info');
            app.cont_bombear();
            app.cont_ui();
        end

        function cont_bombear(app)
            % manda lineas mientras haya lugar en la ventana y en la cola del STM32
            if ~app.Cont.Activa || app.Cont.Fase ~= "stream", return; end
            C = app.Cont;
            while C.Idx <= numel(C.Lineas) && C.Inflight < app.CONT_VENTANA
                if strcmp(C.Lineas{C.Idx}, '#PAUSA')
                    % parada en un punto critico: primero que confirme todo (tambien el E)
                    if C.Inflight > 0, break; end
                    if ~C.Parado
                        if ~C.EsperaIdle
                            C.EsperaIdle = true;  C.NStPausa = app.NSt;  C.TPausa = [];
                            C.ParadaAct = C.ParadaAct + 1;
                        end
                        break;                          % cont_tick avisa cuando quedo quieto y paso la pausa
                    end
                    C.Idx = C.Idx + 1;  C.Parado = false;  C.EsperaIdle = false;
                    continue;
                end
                if C.Idx > 1 && isnan(C.LibresMax), break; end      % esperar el ok del primer S (trae la cola libre)
                if C.EsP(C.Idx) && ~isnan(C.LibresMax) && ...
                        (C.LibresMax - C.Libres) + C.Inflight >= app.CONT_PROF_MAX
                    break;                                          % la cola ya tiene suficiente
                end
                l = app.con_cs(C.Lineas{C.Idx});
                % si no habia nada esperando confirmacion (por ej. despues de una parada), el
                % tiempo de espera del "ok" arranca ahora y no desde el ultimo ok (que puede ser
                % de hace varios segundos, antes de frenar y de la pausa)
                if C.Inflight == 0, C.TOk = tic; end
                try
                    writeline(app.Puerto, l);
                catch ME
                    app.Cont = C;
                    app.cont_finalizar("Error serie durante la trayectoria continua: " + string(ME.message), 'error', false);
                    return;
                end
                app.UltimoTx = string(l);
                C.Sent(end+1) = C.EsP(C.Idx);
                C.Idx = C.Idx + 1;
                C.Inflight = C.Inflight + 1;
            end
            if C.Idx > numel(C.Lineas)
                C.Fase = "fin";          % ya salio el E: esperar los ok y que termine de moverse
                C.Ack = false;
                C.T0 = tic;
                C.TMax = 20 + 3*C.TEst;
            end
            app.Cont = C;
            app.NaNLabel_2.Text = char("TX: " + app.UltimoTx);
        end

        function cont_ok(app, linea)
            % llega un "ok" mientras hay trayectoria continua
            C = app.Cont;
            switch C.Fase
                case "ir_inicio"
                    if ~C.Ack
                        C.Ack = true;  C.NStAck = app.NSt;
                    end
                    app.Cont = C;
                case {"stream", "fin"}
                    C.Inflight = max(C.Inflight - 1, 0);
                    C.Recibidos = C.Recibidos + 1;
                    if C.Recibidos <= numel(C.Sent) && C.Sent(C.Recibidos), C.AckP = C.AckP + 1; end
                    p = split(linea, ",");
                    if numel(p) >= 2
                        lb = str2double(p(2));
                        if ~isnan(lb)
                            C.Libres = lb;
                            if isnan(C.LibresMax), C.LibresMax = lb; end   % ok del primer S: cola vacia
                        end
                    end
                    C.TOk = tic;
                    if C.Fase == "fin" && C.Inflight == 0 && ~C.Ack
                        C.Ack = true;  C.NStAck = app.NSt;                % ya contesto el E
                    end
                    app.Cont = C;
                    if C.Fase == "stream", app.cont_bombear(); end
            end
        end

        function cont_st(app, libres)
            % llega un "st" mientras hay trayectoria continua: actualiza la cola libre y sigue mandando
            % el st sale en orden con los ok: refleja la cola sin los que todavia estan en vuelo
            if app.Cont.Fase == "stream" && ~isnan(app.Cont.LibresMax) && ~isnan(libres)
                app.Cont.Libres = libres;
            end
            if app.Cont.Fase == "stream", app.cont_bombear(); end
        end

        function cont_tick(app)
            C = app.Cont;
            if ~C.Activa, return; end
            tRx = app.TIMEOUT_RX;
            if C.Fase ~= "ir_inicio", tRx = C.TRx; end
            if ~isempty(app.UltimoRx) && toc(app.UltimoRx) > tRx
                app.cont_finalizar(sprintf("El STM32 no responde (sin datos hace mas de %.0f s).", tRx), 'error', true);
                return;
            end
            est = string(app.EstadoRobot);
            if any(est == ["FAULT","NOREF"])
                app.cont_finalizar("El robot informo estado " + est + ": trayectoria abortada.", 'error', false);
                return;
            end
            switch C.Fase
                case "ir_inicio"
                    llego = norm(app.QActual - C.Q(1,:)) <= app.TOL_Q;
                    if C.Ack && app.NSt > C.NStAck && est == "IDLE" && llego
                        if C.SoloIr
                            app.cont_finalizar("El robot esta en el punto inicial de la trayectoria.", 'ok', false);
                        else
                            app.cont_iniciar_stream();
                        end
                    elseif toc(C.T0) > C.TMax
                        app.cont_finalizar(sprintf("No se llego al punto inicial en %.0f s.", C.TMax), 'error', true);
                    end
                case "stream"
                    if C.EsperaIdle && ~C.Parado && app.NSt > C.NStPausa && est == "IDLE"
                        % quieto en el punto critico: cuento la pausa y despues sigue
                        if isempty(C.TPausa), C.TPausa = tic; end
                        if toc(C.TPausa) >= app.ContPausa.Value
                            C.Parado = true;  C.TPausa = [];
                        end
                        app.Cont = C;
                    end
                    app.cont_bombear();
                    if app.Cont.Activa && app.Cont.Inflight > 0 && toc(app.Cont.TOk) > tRx
                        app.cont_finalizar("El STM32 dejo de confirmar los puntos.", 'error', true);
                        return;
                    end
                    app.cont_ui();
                case "fin"
                    if C.Ack && app.NSt > C.NStAck && est == "IDLE"
                        app.cont_finalizar(sprintf("Trayectoria continua completada (%d puntos, %.1f s).", ...
                            C.NPuntos, toc(C.TInicio)), 'ok', false);
                    elseif toc(C.T0) > C.TMax
                        app.cont_finalizar("La trayectoria no termino en el tiempo esperado.", 'error', true);
                    else
                        app.cont_ui();
                    end
            end
        end

        function cb_cont_detener(app)
            if app.Cont.Activa
                app.cont_finalizar("Trayectoria continua detenida por el usuario (se envio X).", 'aviso', true);
            end
        end

        function cont_finalizar(app, msg, tipo, enviarStop)
            if enviarStop
                try
                    writeline(app.Puerto, app.con_cs("X"));
                    app.NaNLabel_2.Text = 'TX: X';
                catch
                end
            end
            app.Cont.Activa = false;
            app.Cont.Fase = "";
            app.Cont.Lineas = {};
            app.reg_fin_trayectoria();
            app.SincPendiente = true;            % al quedar IDLE las barras toman la posicion final
            app.cont_ui();
            app.actualizar_botones();
            if tipo == "error"
                app.alerta('Trayectoria continua', msg);
            else
                app.avisar(msg, tipo);
            end
        end

        function cont_ui(app)
            C = app.Cont;
            if ~C.Activa
                app.LblContEstado.Text = 'Inactiva';
                app.LblContEstado.FontColor = [0 0 0];
                app.LblContProg.Text = '';
                return;
            end
            app.LblContEstado.FontColor = [0 0.45 0];
            switch C.Fase
                case "ir_inicio"
                    app.LblContEstado.Text = 'Yendo al inicio (G)';
                    app.LblContProg.Text = sprintf('q inicial = [%.1f  %.1f  %.1f]', C.Q(1,:));
                otherwise
                    if C.Fase == "stream" && C.EsperaIdle
                        if isempty(C.TPausa)
                            app.LblContEstado.Text = sprintf('Frenando en la parada %d de %d', C.ParadaAct, C.NParadas);
                        else
                            app.LblContEstado.Text = sprintf('Parado en la parada %d de %d (pausa %.1f s)', ...
                                C.ParadaAct, C.NParadas, max(app.ContPausa.Value - toc(C.TPausa), 0));
                        end
                    elseif C.Fase == "stream", app.LblContEstado.Text = 'Ejecutando (stream)';
                    else, app.LblContEstado.Text = 'Terminando (E enviado)'; end
                    enCola = 0;
                    if ~isnan(C.LibresMax) && ~isnan(C.Libres), enCola = C.LibresMax - C.Libres; end
                    hecho = max(C.AckP - enCola, 0);
                    app.LblContProg.Text = sprintf('Confirmados %d/%d puntos  |  en la cola del STM32 ~%d  |  hechos ~%d (%.0f%%)', ...
                        C.AckP, C.NPuntos, enCola, hecho, 100*hecho/max(C.NPuntos,1));
            end
        end
    end

    %% ---------------------------------------------------------------
    %  Registro de la trayectoria continua
    %  Mientras corre el stream se manda Q mas seguido (CONT_POLL) y se guarda cada
    %  "st": qref = referencia que interpola el firmware entre los puntos que mandamos,
    %  qenc = lo que leen los encoders (en el eje 1, sin encoder, es la referencia).
    %  El tiempo es el de llegada a MATLAB (no hay marca de tiempo del STM32).
    %  Despues de terminar se sigue registrando REG_COLA segundos (ver como se asienta).
    %% ---------------------------------------------------------------
    methods (Access = private)
        function reg_iniciar(app, Qdes, Pdes, Desc, Paradas)
            % Sin argumentos: registro de la trayectoria continua (usa app.Cont).
            % Con argumentos: registro de una trayectoria por puntos.
            if nargin < 2
                Qdes = app.Cont.Qe;  Pdes = app.Cont.Pe;  Desc = app.Cont.Desc;
                Paradas = zeros(1,0);
                if isfield(app.Cont, 'ParadasIdx'), Paradas = app.Cont.ParadasIdx; end
            end
            app.reg_borrar_timer();
            app.Reg = struct('t', zeros(0,1), 'est', strings(0,1), 'qref', zeros(0,3), 'qenc', zeros(0,3), ...
                'Qdes', Qdes, 'Pdes', Pdes, 'Desc', Desc, ...
                'Activo', true, 'T0', tic, 'TStop', inf, 'Fecha', string(datetime('now')), 'Paradas', Paradas);
            app.UltUI = [];
            app.TimerLog = timer("ExecutionMode", "fixedSpacing", "Period", app.CONT_POLL, ...
                "BusyMode", "drop", "TimerFcn", @(~,~) app.reg_tick());
            start(app.TimerLog);
        end

        function reg_tick(app)
            if ~isvalid(app) || ~app.Reg.Activo, return; end
            if toc(app.Reg.T0) > app.Reg.TStop
                app.reg_cerrar();
                return;
            end
            try
                writeline(app.Puerto, app.con_cs("Q"));
            catch
                app.reg_cerrar();
            end
        end

        function reg_muestra(app, estado, qref, qenc)
            k = numel(app.Reg.t) + 1;
            app.Reg.t(k,1) = toc(app.Reg.T0);
            app.Reg.est(k,1) = estado;
            app.Reg.qref(k,:) = qref;
            app.Reg.qenc(k,:) = qenc;
        end

        function reg_fin_trayectoria(app)
            % la trayectoria termino (o se aborto): registrar REG_COLA s mas y cortar
            if app.Reg.Activo
                app.Reg.TStop = toc(app.Reg.T0) + app.REG_COLA;
            end
        end

        function reg_cerrar(app)
            app.Reg.Activo = false;
            try, stop(app.TimerLog); catch, end
            app.actualizar_botones();
            if ~isempty(app.Reg.t)
                app.avisar(sprintf('Registro listo: %d muestras en %.1f s (%.0f muestras/s). Usa "Ver registro".', ...
                    numel(app.Reg.t), app.Reg.t(end), numel(app.Reg.t)/max(app.Reg.t(end),1e-3)), 'ok');
            end
        end

        function reg_borrar_timer(app)
            try, stop(app.TimerLog); catch, end
            try, delete(app.TimerLog); catch, end
            app.TimerLog = [];
        end

        function cb_reg_ver(app)
            R = app.Reg;
            if isempty(R.t)
                app.avisar('No hay registro: ejecuta una trayectoria continua.', 'aviso'); return;
            end
            f = figure('Name', 'Registro trayectoria continua', 'NumberTitle', 'off', 'Color', 'w', ...
                'Position', [80 80 1250 680]);
            tl = tiledlayout(f, 2, 3, 'TileSpacing', 'compact', 'Padding', 'compact');
            title(tl, sprintf('%s  |  %s', R.Desc, R.Fecha), 'Interpreter', 'none', 'FontSize', 10);

            % tramo en movimiento (primer y ultimo RUN) para marcar arranque y final
            run = find(R.est == "RUN");
            tRun = [];
            if ~isempty(run), tRun = R.t([run(1) run(end)]); end

            nom = {'q1 (sin encoder: enc = ref)', 'q2', 'q3'};
            for j = 1:3
                ax = nexttile(tl, j); hold(ax, 'on'); grid(ax, 'on');
                plot(ax, R.t, R.qref(:,j), '-', 'Color', [0 0.45 0.74], 'LineWidth', 1.2);
                plot(ax, R.t, R.qenc(:,j), '-', 'Color', [0.85 0.2 0.1]);
                yline(ax, app.q_limites(j,1), ':k'); yline(ax, app.q_limites(j,2), ':k');
                app.reg_marcas(ax, tRun);
                title(ax, nom{j}); xlabel(ax, 't [s]'); ylabel(ax, '[deg]');
                if j == 1, legend(ax, {'referencia (firmware)', 'encoder'}, 'Location', 'best', 'FontSize', 7); end
            end

            % error de seguimiento referencia - encoder
            ax = nexttile(tl, 4); hold(ax, 'on'); grid(ax, 'on');
            e = R.qref - R.qenc;
            col = [0.00 0.45 0.74; 0.85 0.33 0.10; 0.47 0.67 0.19];
            for j = 2:3, plot(ax, R.t, e(:,j), '-', 'Color', col(j,:)); end
            app.reg_marcas(ax, tRun);
            enMov = R.est == "RUN";
            if ~any(enMov), enMov = true(size(R.t)); end
            title(ax, sprintf('Error ref - enc  (en RUN: q2 max %.2f / rms %.2f, q3 max %.2f / rms %.2f deg)', ...
                max(abs(e(enMov,2))), sqrt(mean(e(enMov,2).^2)), max(abs(e(enMov,3))), sqrt(mean(e(enMov,3).^2))), 'FontSize', 8);
            legend(ax, {'q2', 'q3'}, 'Location', 'best', 'FontSize', 7);
            xlabel(ax, 't [s]'); ylabel(ax, '[deg]');

            % espacio articular q2 - q3: deseado vs referencia vs encoder
            ax = nexttile(tl, 5); hold(ax, 'on'); grid(ax, 'on');
            lim = app.q_limites;
            patch(ax, lim(2,[1 2 2 1]), lim(3,[1 1 2 2]), [0.92 0.97 0.92], 'EdgeColor', [0.3 0.6 0.3]);
            hd = plot(ax, R.Qdes(:,2), R.Qdes(:,3), '.', 'Color', [0.2 0.2 0.2], 'MarkerSize', 6);
            hr = plot(ax, R.qref(:,2), R.qref(:,3), '-', 'Color', [0 0.45 0.74], 'LineWidth', 1.2);
            he = plot(ax, R.qenc(:,2), R.qenc(:,3), '-', 'Color', [0.85 0.2 0.1]);
            plot(ax, R.Qdes(1,2), R.Qdes(1,3), 'o', 'Color', [0.2 0.6 0.2], 'MarkerFaceColor', [0.2 0.6 0.2]);
            legend(ax, [hd hr he], {'deseado (puntos enviados)', 'referencia', 'encoder'}, 'Location', 'best', 'FontSize', 7);
            title(ax, 'Articular q2 - q3'); xlabel(ax, 'q2 [deg]'); ylabel(ax, 'q3 [deg]');

            % TCP en el plano XZ: deseado vs FK(referencia) vs FK(encoder)
            ax = nexttile(tl, 6); hold(ax, 'on'); grid(ax, 'on'); axis(ax, 'equal');
            if ~isempty(app.Robot)
                Pr = app.reg_fk(R.qref) * 1000;  Pe = app.reg_fk(R.qenc) * 1000;
                Pd = R.Pdes * 1000;
                hd = plot(ax, Pd(:,1), Pd(:,3), '.', 'Color', [0.2 0.2 0.2], 'MarkerSize', 6);
                hr = plot(ax, Pr(:,1), Pr(:,3), '-', 'Color', [0 0.45 0.74], 'LineWidth', 1.2);
                he = plot(ax, Pe(:,1), Pe(:,3), '-', 'Color', [0.85 0.2 0.1]);
                % error de camino: distancia de cada TCP medido (en RUN) al punto deseado mas cercano
                d = app.reg_dist_camino(Pe(enMov,:), Pd);
                legend(ax, [hd hr he], {'deseado', 'FK referencia', 'FK encoder'}, 'Location', 'best', 'FontSize', 7);
                title(ax, sprintf('TCP plano XZ  (error de camino max %.2f / rms %.2f mm)', max(d), sqrt(mean(d.^2))), 'FontSize', 8);
            else
                title(ax, 'TCP: sin modelo (ejecuta robot.m)');
            end
            xlabel(ax, 'x [mm]'); ylabel(ax, 'z [mm]');
        end

        function reg_marcas(~, ax, tRun)
            % lineas verticales en el arranque y el final del movimiento
            for k = 1:numel(tRun)
                try, xline(ax, tRun(k), '--', 'Color', [0.5 0.5 0.5]); catch, end
            end
        end

        function P = reg_fk(app, Q)
            P = nan(size(Q,1), 3);
            for k = 1:size(Q,1), P(k,:) = app.fk_xyz(Q(k,:)); end
        end

        function d = reg_dist_camino(~, Pm, Pd)
            d = zeros(size(Pm,1), 1);
            for k = 1:size(Pm,1)
                d(k) = sqrt(min(sum((Pd - Pm(k,:)).^2, 2)));
            end
            if isempty(d), d = NaN; end
        end

        function cb_reg_guardar(app)
            R = app.Reg;
            if isempty(R.t)
                app.avisar('No hay registro para guardar.', 'aviso'); return;
            end
            [f, p] = uiputfile('*.mat', 'Guardar registro', fullfile(app.carpeta_defecto(), 'registro_trayectoria.mat'));
            figure(app.UIFigure);
            if isequal(f, 0), return; end
            t = R.t;                         %#ok<NASGU>  [s] llegada de cada muestra a MATLAB
            estado = cellstr(R.est);         %#ok<NASGU>  NOREF / IDLE / RUN / HOLD / FAULT
            qref = R.qref;                   %#ok<NASGU>  [deg] referencia del firmware (interpolada)
            qenc = R.qenc;                   %#ok<NASGU>  [deg] encoders (eje 1 = referencia, no tiene encoder)
            Q_deseada = R.Qdes;              %#ok<NASGU>  [deg] puntos enviados (con ida/vuelta y repeticiones)
            xyz_deseada = R.Pdes;            %#ok<NASGU>  [m]
            xyz_ref = app.reg_fk(R.qref);    %#ok<NASGU>  [m]
            xyz_enc = app.reg_fk(R.qenc);    %#ok<NASGU>  [m]
            descripcion = char(R.Desc);      %#ok<NASGU>
            try
                save(fullfile(p, f), 't', 'estado', 'qref', 'qenc', 'Q_deseada', 'xyz_deseada', ...
                    'xyz_ref', 'xyz_enc', 'descripcion');
                app.avisar(['Registro guardado en ' fullfile(p, f)], 'ok');
            catch ME
                app.alerta('Error al guardar', ME.message);
            end
        end
    end

    %% ---------------------------------------------------------------
    %  Grabar trayectoria a mano (teach)
    %  M,0 suelta los motores (el firmware sigue leyendo los encoders y queda
    %  referenciado), se mueve el brazo con la mano y se guarda la q de los
    %  encoders cada vez que se movio mas de "Guardar cada" grados. Al parar,
    %  lo grabado queda como trayectoria continua: se ve en los graficos, se
    %  guarda con "Guardar .mat" y se ejecuta con "Ejecutar".
    %  Al volver a habilitar (M,1, o el G / S de la ejecucion) el firmware
    %  sincroniza la referencia con los encoders, asi que no pega un salto.
    %% ---------------------------------------------------------------
    methods (Access = private)
        function cb_ens_grabar(app)
            if app.ocupado(), return; end
            if ~app.conectado()
                app.alerta('Sin conexion', 'No hay conexion con el STM32.'); return;
            end
            est = string(app.EstadoRobot);
            if est == "NOREF"
                app.alerta('Robot sin referenciar', ['Para leer los angulos el robot tiene que estar referenciado: ' ...
                    'usa Referenciar (R) o Calibrar (C).']); return;
            elseif est == "FAULT"
                app.alerta('Falla activa', 'Usa "Detener (X)" para borrar la falla.'); return;
            elseif any(est == ["RUN","HOLD"])
                app.alerta('Robot ocupado', 'Espera a que el robot termine de moverse.'); return;
            end
            txt = ['Se van a SOLTAR los motores (M,0) y se graban los encoders mientras moves el brazo con la mano.' newline newline ...
                '- SOSTENE EL BRAZO: sin torque puede caerse por su propio peso.' newline ...
                '- Si los ENA de los drivers no estan conectados, los motores NO se sueltan: apaga la fuente ' ...
                'de los motores despues de apretar Grabar (los encoders se alimentan de la Nucleo y siguen leyendo).' newline ...
                '- No te pases de los limites articulares (esos puntos quedan marcados y no se puede ejecutar).'];
            op = uiconfirm(app.UIFigure, txt, 'Grabar a mano', ...
                'Options', {'Soltar y grabar', 'Cancelar'}, 'DefaultOption', 2, 'CancelOption', 2, 'Icon', 'warning');
            if op ~= "Soltar y grabar", return; end
            if ~app.enviar("M,0"), return; end

            app.ens_borrar_timer();
            app.Ens = struct('Activo', true, 'Q', zeros(0,3), 'T0', tic, 'NLect', 0, 'Crit', zeros(1,0), 'Stop', zeros(1,0));
            if app.QValida, app.ens_muestra(app.QActual); end
            app.UltUI = [];
            app.TimerEns = timer("ExecutionMode", "fixedSpacing", "Period", app.CONT_POLL, ...
                "BusyMode", "drop", "TimerFcn", @(~,~) app.ens_tick());
            start(app.TimerEns);
            app.actualizar_botones();
            app.avisar('Grabando: mueve el brazo con la mano. Apreta "Parar grabacion" al terminar.', 'aviso');
        end

        function cb_ens_marcar(app, tipo)
            % marca la posicion actual: tipo = "paso" (pasa si o si por ahi) o "parada" (frena ahi)
            if ~app.Ens.Activo || ~app.QValida, return; end
            q = app.QActual;
            if isempty(app.Ens.Q) || max(abs(q - app.Ens.Q(end,:))) > 1e-6
                app.Ens.Q(end+1, :) = q;          % se guarda aunque no se haya movido "Guardar cada"
            end
            k = size(app.Ens.Q, 1);
            if tipo == "parada"
                app.Ens.Stop = unique([app.Ens.Stop k]);
                app.Ens.Crit = setdiff(app.Ens.Crit, k);
                app.avisar(sprintf('Parada %d marcada (punto %d): q = [%.1f %.1f %.1f]', ...
                    numel(app.Ens.Stop), k, q), 'ok');
            else
                if ~any(app.Ens.Stop == k), app.Ens.Crit = unique([app.Ens.Crit k]); end
                app.avisar(sprintf('Punto de paso %d marcado (punto %d): q = [%.1f %.1f %.1f]', ...
                    numel(app.Ens.Crit), k, q), 'ok');
            end
        end

        function ens_tick(app)
            if ~isvalid(app) || ~app.Ens.Activo, return; end
            try
                writeline(app.Puerto, app.con_cs("Q"));
            catch
                app.cb_ens_parar();
                return;
            end
            app.LblEnsInfo.Text = sprintf('GRABANDO: %d puntos, %d de paso, %d de parada (%.0f s)', ...
                size(app.Ens.Q, 1), numel(app.Ens.Crit), numel(app.Ens.Stop), toc(app.Ens.T0));
        end

        function ens_muestra(app, q)
            % guarda la q del encoder si se movio al menos "Guardar cada" desde el ultimo punto
            app.Ens.NLect = app.Ens.NLect + 1;
            if isempty(app.Ens.Q) || max(abs(q - app.Ens.Q(end,:))) >= app.EnsUmbral.Value
                app.Ens.Q(end+1, :) = q;
            end
        end

        function cb_ens_parar(app)
            if ~app.Ens.Activo, return; end
            app.Ens.Activo = false;
            app.ens_borrar_timer();
            Q = app.Ens.Q;
            dur = toc(app.Ens.T0);
            app.SincPendiente = true;
            app.actualizar_botones();
            if size(Q, 1) < 2
                app.LblEnsInfo.Text = 'No se grabo movimiento (menos de 2 puntos).';
                app.avisar('Grabacion terminada sin movimiento. Los motores siguen sueltos (usa Motores ON).', 'aviso');
                return;
            end
            crit = app.Ens.Crit;  stop = app.Ens.Stop;
            if app.ChkEnsSuavizar.Value && size(Q, 1) >= 5
                % saca el temblequeo de la mano por tramos: inicio, fin, puntos de paso y de parada quedan exactos
                K = unique([1, crit, stop, size(Q,1)]);
                for j = 1:numel(K) - 1
                    Q(K(j):K(j+1), :) = app.suav_tramo(app.Ens.Q(K(j):K(j+1), :), 5, 'movmean');
                end
            end
            fuera = any(Q < app.q_limites(:,1)' - 1e-6 | Q > app.q_limites(:,2)' + 1e-6, 2);
            P = nan(size(Q,1), 3);
            if ~isempty(app.Robot)
                for k = 1:size(Q,1), P(k,:) = app.fk_xyz(Q(k,:)); end
            end
            desc = sprintf('grabada a mano (%d puntos, %.0f s)', size(Q,1), dur);
            app.cont_set(Q, P, desc, double(fuera), crit, stop);
            app.LblEnsInfo.Text = sprintf(['Listo: %d puntos. Guardala con "Guardar .mat". Antes de ejecutar ' ...
                'devolve la tension a los motores (fuente y/o Motores ON).'], size(Q,1));
            if ~any(fuera)
                app.avisar(sprintf('Grabacion terminada: %d puntos. Los motores siguen sueltos.', size(Q,1)), 'ok');
            end
        end

        function cb_ens_motores_on(app)
            if app.ocupado(), return; end
            if app.enviar("M,1")
                app.SincPendiente = true;
                app.avisar('Motores habilitados (M,1): el firmware toma la posicion de los encoders, no salta.', 'ok');
            end
        end

        function ens_borrar_timer(app)
            try, stop(app.TimerEns); catch, end
            try, delete(app.TimerEns); catch, end
            app.TimerEns = [];
        end
    end

    %% ---------------------------------------------------------------
    %  Suavizar y remuestrear la trayectoria continua cargada
    %  1) promedio gaussiano de "Ventana" puntos sobre q (saca el ruido y los
    %     quiebres chicos, sobre todo de lo grabado a mano o de un .mat ruidoso)
    %  2) remuestreo a separacion constante en el espacio articular (asi cada
    %     tramo del firmware tiene el mismo largo y la velocidad sale pareja),
    %     con interpolacion pchip (curva suave que no se pasa de los puntos).
    %  Se trabaja en q, que es lo que ejecuta el firmware. El inicio y el fin no
    %  se mueven. Se muestra cuanto se aparta la punta del camino original.
    %% ---------------------------------------------------------------
    methods (Access = private)
        function cb_cont_suavizar(app)
            if app.ocupado(), return; end
            Q = app.Cont.Q;  P0 = app.Cont.P;
            if size(Q, 1) < 3 || any(~isfinite(Q(:)))
                app.avisar('Hace falta una trayectoria cargada, sin puntos fuera de alcance, para suavizar.', 'aviso');
                return;
            end
            w = max(1, round(app.SuavVentana.Value));
            sep = app.SuavSep.Value;
            crit = app.Cont.Crit;  stop = app.Cont.Stop;
            N = size(Q, 1);

            % se corta en tramos en el inicio, el fin y cada punto de paso o de parada: cada
            % tramo se suaviza y se remuestrea solo, con sus dos extremos exactos
            K = unique([1, crit, stop, N]);
            cerrada = isempty(crit) && isempty(stop) && norm(Q(1,:) - Q(end,:)) < 1e-3 && N > 2*w + 2;
            Qn = zeros(0, 3);  critN = zeros(1, 0);  stopN = zeros(1, 0);
            for j = 1:numel(K) - 1
                seg = Q(K(j):K(j+1), :);
                if cerrada
                    segS = app.suav_cerrada(seg, w);
                else
                    segS = app.suav_tramo(seg, w, 'gaussian');
                end
                segN = app.remuestrear(segS, sep);
                if isempty(Qn), Qn = segN; else, Qn = [Qn; segN(2:end,:)]; end %#ok<AGROW>
                if any(crit == K(j+1)), critN(end+1) = size(Qn, 1); end        %#ok<AGROW>
                if any(stop == K(j+1)), stopN(end+1) = size(Qn, 1); end        %#ok<AGROW>
            end
            if any(crit == 1), critN = [1 critN]; end
            if any(stop == 1), stopN = [1 stopN]; end
            if size(Qn,1) < 2 || size(Qn,1) > app.CONT_PUNTOS_MAX
                app.alerta('Remuestreo', sprintf('Saldrian %d puntos: cambia la separacion.', size(Qn,1))); return;
            end

            % cinematica directa, limites y cuanto se aparta del camino original
            Pn = nan(size(Qn,1), 3);
            if ~isempty(app.Robot)
                for k = 1:size(Qn,1), Pn(k,:) = app.fk_xyz(Qn(k,:)); end
            end
            est = double(any(Qn < app.q_limites(:,1)' - 1e-6 | Qn > app.q_limites(:,2)' + 1e-6, 2));
            dev = NaN;
            if all(isfinite(Pn(:))) && all(isfinite(P0(:)))
                dev = max(app.reg_dist_camino(Pn * 1000, P0 * 1000));
            end

            app.Cont.Ant = struct('Q', app.Cont.Q, 'P', app.Cont.P, 'Estado', app.Cont.Estado, 'Desc', app.Cont.Desc, ...
                'Crit', app.Cont.Crit, 'Stop', app.Cont.Stop);
            base = char(app.Cont.Desc);
            k = strfind(base, ' | suavizada');
            if ~isempty(k), base = base(1:k(1)-1); end      % si ya estaba suavizada no se repite el texto
            desc = sprintf('%s | suavizada (ventana %d, cada %.2f deg)', base, w, sep);
            app.cont_set(Qn, Pn, desc, est, critN, stopN);
            app.LblSuavInfo.Text = sprintf(['%d -> %d puntos (%d criticos sin mover). La punta se aparta como maximo ' ...
                '%.2f mm del camino original. "Deshacer" vuelve a la anterior.'], size(Q,1), size(Qn,1), numel(critN) + numel(stopN), dev);
            app.actualizar_botones();
        end

        function S = suav_tramo(~, seg, w, metodo)
            % suaviza un tramo dejando sus dos extremos exactos (correccion lineal chiquita)
            S = seg;
            n = size(seg, 1);
            if w <= 1 || n < 3, return; end
            w = min(w, n);
            if metodo == "movmean"
                S = movmean(seg, w, 1);
            else
                S = smoothdata(seg, 1, 'gaussian', w);
            end
            t = linspace(0, 1, n)';
            S = S + (1 - t) .* (seg(1,:) - S(1,:)) + t .* (seg(end,:) - S(end,:));
        end

        function S = suav_cerrada(~, Q, w)
            % trayectoria cerrada (ej. circulo sin puntos criticos): se suaviza dando la vuelta
            if w <= 1, S = Q; return; end
            Qp = [Q(end-w:end-1,:); Q; Q(2:w+1,:)];
            Qp = smoothdata(Qp, 1, 'gaussian', w);
            S = Qp(w+1:end-w, :);
            S(1,:) = Q(1,:);  S(end,:) = Q(end,:);
        end

        function Qn = remuestrear(~, Q, sep)
            % puntos equidistantes en q (cada sep grados) con interpolacion pchip; extremos exactos
            d = sqrt(sum(diff(Q).^2, 2));
            s = [0; cumsum(d)];
            ok = [true; d > 1e-9];
            s = s(ok);  Qs = Q(ok, :);
            L = s(end);
            if L < 1e-6 || size(Qs,1) < 2
                Qn = Q([1 end], :);              % tramo quieto
                return;
            end
            nn = max(1, ceil(L / sep));
            Qn = interp1(s, Qs, linspace(0, L, nn + 1)', 'pchip');
            Qn(1,:) = Q(1,:);  Qn(end,:) = Q(end,:);
        end

        function t = crit_texto(~, crit)
            if isempty(crit), t = ''; else, t = strjoin(compose('%d', crit), ', '); end
        end

        function cb_crit_editar(app)
            % se escriben a mano los numeros de punto de paso y de parada
            if isempty(app.Cont.Q), app.SuavCrit.Value = ''; app.SuavStop.Value = ''; return; end
            N = size(app.Cont.Q,1);
            leer = @(t) unique(round(str2double(regexp(t, '[\d\.]+', 'match'))));
            v = leer(app.SuavCrit.Value);  v = v(isfinite(v));
            st = leer(app.SuavStop.Value); st = st(isfinite(st));
            malos = [v(v < 1 | v > N), st(st < 1 | st > N)];
            v = v(v >= 1 & v <= N);  st = st(st >= 1 & st <= N);
            v = setdiff(v, st);
            app.Cont.Crit = v;  app.Cont.Stop = st;
            app.SuavCrit.Value = app.crit_texto(v);
            app.SuavStop.Value = app.crit_texto(st);
            app.cont_preview();
            app.cont_info();
            if ~isempty(malos)
                app.avisar(sprintf('Se ignoraron numeros fuera de 1..%d.', size(app.Cont.Q,1)), 'aviso');
            end
        end

        function cb_cont_deshacer(app)
            if app.ocupado() || ~isfield(app.Cont, 'Ant') || isempty(app.Cont.Ant), return; end
            A = app.Cont.Ant;
            app.Cont.Ant = [];
            app.cont_set(A.Q, A.P, A.Desc, A.Estado, A.Crit, A.Stop);
            app.LblSuavInfo.Text = 'Se volvio a la trayectoria anterior.';
            app.actualizar_botones();
        end
    end

    %% ---------------------------------------------------------------
    %  Inicio y construccion de la interfaz
    %% ---------------------------------------------------------------
    methods (Access = private)
        function startupFcn(app)
            app.q_limites = [app.q1_limites; app.q2_limites; app.q3_limites];
            app.config_limits_art();                      % primero los limites de sliders/campos
            try
                app.obtener_robot();
            catch ME
                app.Robot = [];
                uialert(app.UIFigure, ME.message, 'Robot no disponible');
            end
            app.UIFigure.CloseRequestFcn = @(~,~) app.cerrar();   % libera el COM y el timer al cerrar

            % biblioteca de trayectorias (persistente en trayectorias.mat junto a este archivo)
            app.Lib = Trayectorias;
            f = fullfile(app.carpeta_defecto(), app.ARCHIVO_DEFECTO);
            if isfile(f)
                try
                    app.Lib.cargar(f);
                    app.TrayIdx = min(1, app.Lib.cantidad());
                catch ME
                    app.avisar("No se pudo cargar " + string(app.ARCHIVO_DEFECTO) + ": " + string(ME.message), 'aviso');
                end
            end

            app.crear_3d();
            app.cont_defaults();
            app.cb_cont_plano(false);
            app.cont_ws_dibujar();
            app.cont_info();
            app.refrescar_todo();
            app.actualizar_objetivo();
            app.ejec_ui();
            app.is_connected();
        end

        function createComponents(app)
            app.UIFigure = uifigure('Visible', 'off');
            app.UIFigure.Position = [60 40 1280 700];
            app.UIFigure.Name = 'Supervisor Brazo 3 GDL';

            app.TabGroup = uitabgroup(app.UIFigure);
            app.TabGroup.Position = [10 40 860 650];
            app.TabControl = uitab(app.TabGroup, 'Title', 'Control manual');
            app.TabTray = uitab(app.TabGroup, 'Title', 'Trayectorias');
            app.TabCont = uitab(app.TabGroup, 'Title', 'Trayectoria continua');
            app.TabTorque = uitab(app.TabGroup, 'Title', 'Torques estimados');

            app.LblBarra = uilabel(app.UIFigure);
            app.LblBarra.Position = [10 10 1260 22];
            app.LblBarra.Text = 'Listo.';
            app.LblBarra.FontColor = [0.35 0.35 0.35];

            app.crearTabControl();
            app.crearTabTrayectorias();
            app.crearTabContinua();
            app.crearTabTorques();
            app.crear3DPanel();

            app.BtnsConexion = {app.EnviarButton, app.MoverP2PButton, app.cmdEditField, ...
                app.BtnMotOn, app.BtnMotOff, app.BtnRef, app.BtnCal, app.BtnLim, app.BtnEnvPunto};
            app.CtrlsManual = {app.q1Slider, app.q2Slider, app.q3Slider, ...
                app.q1EditField, app.q2EditField, app.q3EditField, ...
                app.xEditField, app.yEditField, app.zEditField, ...
                app.CalcularCIButton, app.RestablecerButton, app.VelEditField};
            app.BtnsEdicion = {app.ListaTray, app.BtnNuevaTray, app.BtnRenTray, app.BtnDupTray, ...
                app.BtnDelTray, app.BtnAbrir, app.BtnGuardar, app.BtnGuardarComo, app.BtnImportar, ...
                app.BtnExportar, app.TablaPuntos, app.BtnAddPunto, app.BtnDupPunto, app.BtnDelPunto, ...
                app.BtnActPunto, app.BtnSubir, app.BtnBajar, app.BtnIrPunto, app.DdModo, app.TMinEdit};

            app.UIFigure.Visible = 'on';
        end

        %% ---------------- Pestana: control manual ----------------
        function crearTabControl(app)
            T = app.TabControl;

            % ---------- Panel articular ----------
            app.Panel = uipanel(T);
            app.Panel.Title = 'Espacio articular (DH) [deg]  -  posicion objetivo';
            app.Panel.Position = [20 395 363 198];

            ys = [148 105 62];
            for k = 1:3
                lb = uilabel(app.Panel);
                lb.HorizontalAlignment = 'right';
                lb.Position = [21 ys(k) 25 22];
                lb.Text = sprintf('q%d', k);

                sl = uislider(app.Panel);
                sl.Position = [67 ys(k)+9 150 3];
                sl.Limits = [-180 180];
                sl.ValueChangingFcn = @(~,ev) app.cb_slider(k, ev.Value);
                sl.ValueChangedFcn  = @(~,ev) app.cb_slider(k, ev.Value);

                ef = uieditfield(app.Panel, 'numeric');
                ef.Position = [242 ys(k)-10 100 22];
                ef.ValueDisplayFormat = '%.2f';
                ef.ValueChangedFcn = @(~,~) app.cb_edit(k);

                switch k
                    case 1, app.q1SliderLabel = lb; app.q1Slider = sl; app.q1EditField = ef;
                    case 2, app.q2SliderLabel = lb; app.q2Slider = sl; app.q2EditField = ef;
                    case 3, app.q3SliderLabel = lb; app.q3Slider = sl; app.q3EditField = ef;
                end
            end

            app.EnviarButton = uibutton(app.Panel, 'push');
            app.EnviarButton.ButtonPushedFcn = @(~,~) app.enviar_move();
            app.EnviarButton.Position = [242 18 100 23];
            app.EnviarButton.Text = 'Enviar';
            app.EnviarButton.Tooltip = 'Envia el P2P (G) con la posicion objetivo actual y la velocidad indicada';

            app.cmdEditFieldLabel = uilabel(app.Panel);
            app.cmdEditFieldLabel.HorizontalAlignment = 'right';
            app.cmdEditFieldLabel.Position = [72 10 28 22];
            app.cmdEditFieldLabel.Text = 'cmd';

            app.cmdEditField = uieditfield(app.Panel, 'text');
            app.cmdEditField.ValueChangedFcn = @(~,~) app.cb_cmd();
            app.cmdEditField.Position = [115 10 100 22];

            % ---------- Estado informado por el STM32 ----------
            app.ESTADOLabel = uilabel(T);
            app.ESTADOLabel.Position = [20 365 57 22];
            app.ESTADOLabel.Text = 'ESTADO:';

            app.NaNLabel = uilabel(T);
            app.NaNLabel.Position = [77 365 306 22];
            app.NaNLabel.Text = 'NaN';

            app.NaNLabel_2 = uilabel(T);
            app.NaNLabel_2.Position = [77 343 306 22];
            app.NaNLabel_2.Text = 'NaN';

            app.NaNLabel_3 = uilabel(T);
            app.NaNLabel_3.Position = [77 321 306 22];
            app.NaNLabel_3.Text = 'NaN';

            app.XYZLabel = uilabel(T);
            app.XYZLabel.Position = [20 292 363 22];
            app.XYZLabel.Text = 'TCP informado = [-  -  -] m';

            app.XYZObjLabel = uilabel(T);
            app.XYZObjLabel.Position = [20 270 363 22];
            app.XYZObjLabel.Text = 'TCP objetivo (FK) = [-  -  -] m';

            app.IKLabel = uilabel(T);
            app.IKLabel.Position = [20 205 363 60];
            app.IKLabel.VerticalAlignment = 'top';
            app.IKLabel.WordWrap = 'on';
            app.IKLabel.Text = '';

            app.RestablecerButton = app.mkBtn(T, 'Copiar pose informada', @() app.cb_restablecer(), ...
                'Carga en los controles la ultima q informada por el STM32 (no envia nada)');
            app.RestablecerButton.Position = [20 165 180 28];
            app.BtnStop = app.mkBtn(T, 'Detener (X)', @() app.cb_stop(), ...
                'X: frena siguiendo la trayectoria y borra una falla. NO es una parada de emergencia: no reemplaza al corte fisico de potencia', true);
            app.BtnStop.Position = [215 165 168 28];

            % ---------- Comandos del robot (comandos crudos ya existentes) ----------
            app.PanelCmd = uipanel(T);
            app.PanelCmd.Title = 'Comandos del robot';
            app.PanelCmd.Position = [20 20 363 130];
            xs = [10 127 244];
            app.BtnMotOn  = app.mkBtn(app.PanelCmd, 'Motores ON', @() app.cb_comando("M,1"), ...
                'M,1: habilita los motores (con torque). Si ya esta referenciado, sincroniza con los encoders');
            app.BtnMotOff = app.mkBtn(app.PanelCmd, 'Motores OFF', @() app.cb_comando("M,0"), ...
                'M,0: suelta los motores (sin torque). Cuidado: el brazo puede caer por gravedad');
            app.BtnRef    = app.mkBtn(app.PanelCmd, 'Referenciar (R)', @() app.cb_comando("R"), ...
                'R: referencia con la calibracion guardada (el robot tiene que estar cerca de la pose de calibracion)');
            app.BtnCal    = app.mkBtn(app.PanelCmd, 'Calibrar (C)', @() app.cb_calibrar(), ...
                'C: le dice al firmware en que pose esta FISICAMENTE el robot ahora (graba en flash)');
            app.BtnLim    = app.mkBtn(app.PanelCmd, 'Limites -> STM32', @() app.cb_limites(), ...
                'L: envia los limites articulares de esta GUI al firmware (graba en flash)');
            app.BtnMotOn.Position  = [xs(1) 62 108 28];
            app.BtnMotOff.Position = [xs(2) 62 108 28];
            app.BtnRef.Position    = [xs(3) 62 108 28];
            app.BtnCal.Position    = [xs(1) 20 108 28];
            app.BtnLim.Position    = [xs(2) 20 108 28];

            app.LblVel = uilabel(app.PanelCmd);
            app.LblVel.Text = 'v P2P [deg/s]';
            app.LblVel.Position = [xs(3) 44 108 18];
            app.LblVel.HorizontalAlignment = 'center';
            app.VelEditField = uieditfield(app.PanelCmd, 'numeric');
            app.VelEditField.Position = [xs(3)+14 20 80 22];
            app.VelEditField.Limits = [0.5 200];   % el firmware igual recorta a JOINT_VMAX de cada eje
            app.VelEditField.Value = app.V_P2P_DEFECTO;
            app.VelEditField.ValueDisplayFormat = '%.1f';
            app.VelEditField.Tooltip = 'Velocidad sobre la trayectoria articular. El firmware la reduce si algun eje superaria su velocidad maxima';

            % ---------- P2P cartesiano (CI) ----------
            app.PanelXYZ = uipanel(T);
            app.PanelXYZ.Title = 'P2P cartesiano (CI)';
            app.PanelXYZ.Position = [400 330 210 240];

            nombres = {'x [m]', 'y [m]', 'z [m]'};
            valores = [0.04 0.02 0];
            ysx = [175 140 105];
            for k = 1:3
                lb = uilabel(app.PanelXYZ);
                lb.HorizontalAlignment = 'right';
                lb.Position = [8 ysx(k) 45 22];
                lb.Text = nombres{k};
                ef = uieditfield(app.PanelXYZ, 'numeric');
                ef.ValueDisplayFormat = '%.4f';
                ef.Position = [65 ysx(k) 125 22];
                ef.Value = valores(k);
                switch k
                    case 1, app.xEditFieldLabel = lb; app.xEditField = ef;
                    case 2, app.yEditFieldLabel = lb; app.yEditField = ef;
                    case 3, app.zEditFieldLabel = lb; app.zEditField = ef;
                end
            end

            app.CalcularCIButton = uibutton(app.PanelXYZ, 'push');
            app.CalcularCIButton.ButtonPushedFcn = @(~,~) app.resolver_y_mostrar(false);
            app.CalcularCIButton.Position = [20 60 170 28];
            app.CalcularCIButton.Text = 'Calcular CI';
            app.CalcularCIButton.Tooltip = 'Resuelve la CI y carga q1..q3 en los controles (no envia)';

            app.MoverP2PButton = uibutton(app.PanelXYZ, 'push');
            app.MoverP2PButton.ButtonPushedFcn = @(~,~) app.resolver_y_mostrar(true);
            app.MoverP2PButton.Position = [20 22 170 28];
            app.MoverP2PButton.Text = 'Mover P2P';
            app.MoverP2PButton.Tooltip = 'Resuelve la CI y envia el P2P (G)';

            % ---------- Conexion ----------
            app.ConnectLampLabel = uilabel(T);
            app.ConnectLampLabel.HorizontalAlignment = 'right';
            app.ConnectLampLabel.Position = [500 590 100 22];
            app.ConnectLampLabel.Text = 'Connect';

            app.ConnectLamp = uilamp(T);
            app.ConnectLamp.Position = [615 591 20 20];
            app.ConnectLamp.Color = [0.8392 0.4863 0.4863];
        end

        %% ---------------- Panel 3D (siempre visible) ----------------
        function crear3DPanel(app)
            app.Panel3D = uipanel(app.UIFigure);
            app.Panel3D.Title = 'Vista 3D del brazo';
            app.Panel3D.Position = [880 40 390 650];

            app.Ax3D = uiaxes(app.Panel3D);
            app.Ax3D.Position = [10 110 370 510];

            app.ChkRecorrido = uicheckbox(app.Panel3D);
            app.ChkRecorrido.Text = 'Mostrar recorrido de la trayectoria';
            app.ChkRecorrido.Value = true;
            app.ChkRecorrido.Position = [10 80 250 22];
            app.ChkRecorrido.ValueChangedFcn = @(~,~) app.actualizar_3d_recorrido();

            app.BtnVista = uibutton(app.Panel3D, 'push');
            app.BtnVista.Text = 'Vista inicial';
            app.BtnVista.Position = [275 80 105 24];
            app.BtnVista.ButtonPushedFcn = @(~,~) app.cb_vista();

            app.LblLeyenda = uilabel(app.Panel3D);
            app.LblLeyenda.Position = [10 5 370 68];
            app.LblLeyenda.VerticalAlignment = 'top';
            app.LblLeyenda.WordWrap = 'on';
            app.LblLeyenda.Text = ['Azul: posicion OBJETIVO (MATLAB).  Gris: q INFORMADA por el STM32 ' ...
                '(consigna interna, no medicion).  Naranja: punto seleccionado.  Rojo: recorrido ' ...
                'previsto del TCP.  Mouse: rotar / zoom.'];
        end

        %% ---------------- Pestana: trayectorias ----------------
        function crearTabTrayectorias(app)
            G = uigridlayout(app.TabTray, [2 2]);
            G.RowHeight = {'1x', 170};
            G.ColumnWidth = {250, '1x'};
            G.Padding = [10 10 10 10];

            % Lista de trayectorias y archivo
            app.PanelTrayLista = uipanel(G, 'Title', 'Trayectorias');
            app.PanelTrayLista.Layout.Row = [1 2]; app.PanelTrayLista.Layout.Column = 1;
            g = uigridlayout(app.PanelTrayLista, [7 2]);
            g.RowHeight = {'1x', 28, 28, 28, 28, 28, 50};
            g.ColumnWidth = {'1x', '1x'};

            app.ListaTray = uilistbox(g);
            app.ListaTray.Layout.Row = 1; app.ListaTray.Layout.Column = [1 2];
            app.ListaTray.Items = {};
            app.ListaTray.ValueChangedFcn = @(~,~) app.cb_lista();

            app.BtnNuevaTray = app.mkBtn(g, 'Nueva trayectoria', @() app.cb_nueva_tray());
            app.BtnRenTray   = app.mkBtn(g, 'Renombrar', @() app.cb_ren_tray());
            app.BtnDupTray   = app.mkBtn(g, 'Duplicar', @() app.cb_dup_tray());
            app.BtnDelTray   = app.mkBtn(g, 'Eliminar', @() app.cb_del_tray());
            app.BtnAbrir     = app.mkBtn(g, 'Abrir...', @() app.cb_abrir(), 'Reemplaza la biblioteca por el contenido de un archivo .mat');
            app.BtnGuardar   = app.mkBtn(g, 'Guardar', @() app.cb_guardar(), 'Guarda toda la biblioteca en el archivo actual');
            app.BtnGuardarComo = app.mkBtn(g, 'Guardar como...', @() app.cb_guardar_como());
            app.BtnImportar  = app.mkBtn(g, 'Importar...', @() app.cb_importar(), 'Agrega las trayectorias de otro archivo');
            app.BtnExportar  = app.mkBtn(g, 'Exportar seleccionada...', @() app.cb_exportar(), 'Guarda solo la trayectoria seleccionada');
            pos = {app.BtnNuevaTray, 2, 1; app.BtnRenTray, 2, 2; app.BtnDupTray, 3, 1; ...
                   app.BtnDelTray, 3, 2; app.BtnAbrir, 4, 1; app.BtnGuardar, 4, 2; ...
                   app.BtnGuardarComo, 5, 1; app.BtnImportar, 5, 2};
            for k = 1:size(pos,1)
                pos{k,1}.Layout.Row = pos{k,2}; pos{k,1}.Layout.Column = pos{k,3};
            end
            app.BtnExportar.Layout.Row = 6; app.BtnExportar.Layout.Column = [1 2];

            app.LblArchivo = uilabel(g);
            app.LblArchivo.Layout.Row = 7; app.LblArchivo.Layout.Column = [1 2];
            app.LblArchivo.WordWrap = 'on';
            app.LblArchivo.VerticalAlignment = 'top';
            app.LblArchivo.Text = '';

            % Puntos de la trayectoria seleccionada
            app.PanelPuntos = uipanel(G, 'Title', 'Puntos');
            app.PanelPuntos.Layout.Row = 1; app.PanelPuntos.Layout.Column = 2;
            g2 = uigridlayout(app.PanelPuntos, [5 1]);
            g2.RowHeight = {22, '1x', 36, 22, 66};

            app.LblTrayNombre = uilabel(g2);
            app.LblTrayNombre.Layout.Row = 1;
            app.LblTrayNombre.FontWeight = 'bold';
            app.LblTrayNombre.Text = 'Sin trayectoria seleccionada';

            app.TablaPuntos = uitable(g2);
            app.TablaPuntos.Layout.Row = 2;
            app.TablaPuntos.ColumnName = {'#', 'Nombre', 'q1 [deg]', 'q2 [deg]', 'q3 [deg]', 'X [m]', 'Y [m]', 'Z [m]', 'Iman'};
            app.TablaPuntos.ColumnWidth = {34, 'auto', 70, 70, 70, 70, 70, 70, 50};
            app.TablaPuntos.ColumnEditable = [false true true true true false false false false];
            app.TablaPuntos.ColumnFormat = {'numeric','char','numeric','numeric','numeric','numeric','numeric','numeric','char'};
            app.TablaPuntos.RowName = {};
            app.TablaPuntos.Data = cell(0, 9);
            try
                app.TablaPuntos.SelectionType = 'row';
                app.TablaPuntos.Multiselect = 'off';
            catch
            end
            app.TablaPuntos.CellSelectionCallback = @(~,ev) app.cb_seleccion_celda(ev);
            app.TablaPuntos.CellEditCallback = @(~,ev) app.cb_edicion_celda(ev);

            app.LblSecuencia = uilabel(g2);
            app.LblSecuencia.Layout.Row = 3;
            app.LblSecuencia.WordWrap = 'on';
            app.LblSecuencia.VerticalAlignment = 'top';
            app.LblSecuencia.Text = '';

            app.LblPuntoInfo = uilabel(g2);
            app.LblPuntoInfo.Layout.Row = 4;
            app.LblPuntoInfo.Text = '';

            tb = uigridlayout(g2, [2 4]);
            tb.Layout.Row = 5;
            tb.Padding = [0 0 0 0]; tb.RowSpacing = 6; tb.ColumnSpacing = 8;
            tb.RowHeight = {28, 28};
            app.BtnAddPunto = app.mkBtn(tb, '+ Anadir punto actual', @() app.cb_add_punto(), ...
                'Guarda la posicion objetivo de los controles (con su XYZ por cinematica directa) y el estado actual del iman');
            app.BtnDupPunto = app.mkBtn(tb, 'Duplicar punto', @() app.cb_dup_punto());
            app.BtnDelPunto = app.mkBtn(tb, 'Eliminar punto', @() app.cb_del_punto());
            app.BtnActPunto = app.mkBtn(tb, 'Actualizar con objetivo', @() app.cb_act_punto(), ...
                'Reemplaza las coordenadas y el estado del iman del punto seleccionado por los actuales');
            app.BtnSubir = app.mkBtn(tb, [app.S_UP ' Subir'], @() app.mover_punto(-1), 'Mueve el punto antes en la secuencia');
            app.BtnBajar = app.mkBtn(tb, [app.S_DOWN ' Bajar'], @() app.mover_punto(+1), 'Mueve el punto despues en la secuencia');
            app.BtnIrPunto = app.mkBtn(tb, 'Cargar como objetivo', @() app.cb_ir_punto(), ...
                'Carga el punto en los controles. No envia nada al robot');
            app.BtnEnvPunto = app.mkBtn(tb, 'Enviar punto al robot', @() app.cb_env_punto(), ...
                'Carga el punto y envia el P2P (G) (accion explicita)');
            btn = {app.BtnAddPunto, app.BtnDupPunto, app.BtnDelPunto, app.BtnActPunto, ...
                   app.BtnSubir, app.BtnBajar, app.BtnIrPunto, app.BtnEnvPunto};
            for k = 1:8
                btn{k}.Layout.Row = ceil(k/4); btn{k}.Layout.Column = mod(k-1,4) + 1;
            end

            % Ejecucion
            app.PanelEjec = uipanel(G, 'Title', 'Ejecucion');
            app.PanelEjec.Layout.Row = 2; app.PanelEjec.Layout.Column = 2;
            g3 = uigridlayout(app.PanelEjec, [3 5]);
            g3.RowHeight = {28, 32, '1x'};
            g3.ColumnWidth = {50, 170, 150, 70, '1x'};

            l = uilabel(g3); l.Text = 'Modo'; l.Layout.Row = 1; l.Layout.Column = 1;
            app.DdModo = uidropdown(g3);
            app.DdModo.Layout.Row = 1; app.DdModo.Layout.Column = 2;
            app.DdModo.Items = {'Una vez', 'Bucle (hasta detener)'};
            app.DdModo.Value = 'Una vez';

            l = uilabel(g3); l.Text = 'Espera min. por punto [s]'; l.Layout.Row = 1; l.Layout.Column = 3;
            l.HorizontalAlignment = 'right';
            app.TMinEdit = uieditfield(g3, 'numeric');
            app.TMinEdit.Layout.Row = 1; app.TMinEdit.Layout.Column = 4;
            app.TMinEdit.Limits = [0.5 60];
            app.TMinEdit.Value = app.TMIN_DEFECTO;
            app.TMinEdit.Tooltip = ['El firmware no confirma "llegada". El punto se da por terminado tras esta espera, ' ...
                'con estado IDLE y q informada cerca del objetivo. Ajustala al tiempo real del movimiento.'];

            app.LblEjecEstado = uilabel(g3);
            app.LblEjecEstado.Layout.Row = 1; app.LblEjecEstado.Layout.Column = 5;
            app.LblEjecEstado.FontWeight = 'bold';
            app.LblEjecEstado.Text = 'Inactiva';

            app.BtnEjecutar = app.mkBtn(g3, 'Ejecutar', @() app.cb_ejecutar(), ...
                'Ejecuta la trayectoria seleccionada punto por punto (un G a la vez)');
            app.BtnEjecutar.Layout.Row = 2; app.BtnEjecutar.Layout.Column = [1 2];
            app.BtnPausa = app.mkBtn(g3, 'Pausar', @() app.cb_pausa(), ...
                'Pausa al terminar el punto en curso (el firmware no tiene pausa)');
            app.BtnPausa.Layout.Row = 2; app.BtnPausa.Layout.Column = 3;
            app.BtnDetener = app.mkBtn(g3, 'Detener', @() app.cb_detener(), ...
                'Envia STOP y termina la ejecucion. No es una parada de emergencia', true);
            app.BtnDetener.Layout.Row = 2; app.BtnDetener.Layout.Column = [4 5];

            app.LblProgreso = uilabel(g3);
            app.LblProgreso.Layout.Row = 3; app.LblProgreso.Layout.Column = [1 5];
            app.LblProgreso.Text = '';
        end

        %% ---------------- Pestana: trayectoria continua ----------------
        function crearTabTorques(app)
            % Pestana 'Torques estimados': graficos de tau_j y tau_m de la trayectoria en curso
            try
                app.PanelTorque = panel_torques(app.TabTorque, @() app.Reg);
            catch ME
                uilabel(app.TabTorque, 'Text', ['No se pudo crear la pestana: ' ME.message], 'Position', [20 600 800 22]);
            end
        end

        function crearTabContinua(app)
            G = uigridlayout(app.TabCont, [2 2]);
            G.RowHeight = {300, '1x'};
            G.ColumnWidth = {330, '1x'};
            G.Padding = [8 8 8 8];
            G.RowSpacing = 8; G.ColumnSpacing = 8;

            % ================= generadores (pestanas chicas) =================
            tg = uitabgroup(G);
            tg.Layout.Row = 1; tg.Layout.Column = 1;
            tR = uitab(tg, 'Title', 'Recta 3D');
            tC = uitab(tg, 'Title', 'Circulo');
            tM = uitab(tg, 'Title', 'Archivo .mat');
            tE = uitab(tg, 'Title', 'Grabar a mano');
            tS = uitab(tg, 'Title', 'Suavizar');

            % ---------- recta en el espacio ----------
            g = uigridlayout(tR, [7 4]);
            g.RowHeight = {18, 24, 24, 24, 24, 30, '1x'};
            g.ColumnWidth = {34, '1x', '1x', '1x'};
            g.RowSpacing = 5; g.Padding = [8 8 8 8];
            hdr = {'', 'x [mm]', 'y [mm]', 'z [mm]'};
            for c = 2:4
                l = uilabel(g, 'Text', hdr{c}, 'HorizontalAlignment', 'center', 'FontWeight', 'bold');
                l.Layout.Row = 1; l.Layout.Column = c;
            end
            l = uilabel(g, 'Text', 'P1'); l.Layout.Row = 2; l.Layout.Column = 1;
            app.R1x = uieditfield(g, 'numeric', 'Limits', [-1000 1000]); app.R1x.Layout.Row = 2; app.R1x.Layout.Column = 2;
            app.R1y = uieditfield(g, 'numeric', 'Limits', [-1000 1000]); app.R1y.Layout.Row = 2; app.R1y.Layout.Column = 3;
            app.R1z = uieditfield(g, 'numeric', 'Limits', [-1000 1000]); app.R1z.Layout.Row = 2; app.R1z.Layout.Column = 4;
            l = uilabel(g, 'Text', 'P2'); l.Layout.Row = 3; l.Layout.Column = 1;
            app.R2x = uieditfield(g, 'numeric', 'Limits', [-1000 1000]); app.R2x.Layout.Row = 3; app.R2x.Layout.Column = 2;
            app.R2y = uieditfield(g, 'numeric', 'Limits', [-1000 1000]); app.R2y.Layout.Row = 3; app.R2y.Layout.Column = 3;
            app.R2z = uieditfield(g, 'numeric', 'Limits', [-1000 1000]); app.R2z.Layout.Row = 3; app.R2z.Layout.Column = 4;
            app.BtnR1Tcp = app.mkBtn(g, 'P1 = TCP actual', @() app.cb_cont_tomar("p1"));
            app.BtnR1Tcp.Layout.Row = 4; app.BtnR1Tcp.Layout.Column = [1 2];
            app.BtnR2Tcp = app.mkBtn(g, 'P2 = TCP actual', @() app.cb_cont_tomar("p2"));
            app.BtnR2Tcp.Layout.Row = 4; app.BtnR2Tcp.Layout.Column = [3 4];
            l = uilabel(g, 'Text', 'Separacion entre puntos [mm]'); l.Layout.Row = 5; l.Layout.Column = [1 3];
            app.RectSep = uieditfield(g, 'numeric', 'Limits', [0.1 50], 'Value', 1);
            app.RectSep.Layout.Row = 5; app.RectSep.Layout.Column = 4;
            app.BtnRectGen = app.mkBtn(g, 'Generar recta', @() app.cb_rect3d_generar(), ...
                'Calcula los puntos y la CI y los dibuja. No envia nada');
            app.BtnRectGen.Layout.Row = 6; app.BtnRectGen.Layout.Column = [1 4];
            app.BtnRectGen.FontWeight = 'bold';
            l = uilabel(g, 'Text', ['Recta vertical: misma x e y en P1 y P2 (q1 no se mueve, solo q2 y q3). ' ...
                'Ida y vuelta, repeticiones y velocidad se eligen abajo en Ejecucion.']);
            l.Layout.Row = 7; l.Layout.Column = [1 4];
            l.WordWrap = 'on'; l.VerticalAlignment = 'top'; l.FontSize = 10; l.FontColor = [0.4 0.4 0.4];

            % ---------- circulo ----------
            g = uigridlayout(tC, [9 2]);
            g.RowHeight = repmat({22}, 1, 9);
            g.ColumnWidth = {'1x', 110};
            g.RowSpacing = 4; g.Padding = [8 6 8 6];
            l = uilabel(g, 'Text', 'Plano'); l.Layout.Row = 1; l.Layout.Column = 1;
            app.DdContPlano = uidropdown(g);
            app.DdContPlano.Layout.Row = 1; app.DdContPlano.Layout.Column = 2;
            app.DdContPlano.Items = {'Brazo (q2,q3)', 'XY (z fija)', 'XZ (y fija)', 'YZ (x fija)'};
            app.DdContPlano.ItemsData = {'brazo', 'xy', 'xz', 'yz'};
            app.DdContPlano.Value = 'brazo';
            app.DdContPlano.Tooltip = 'Brazo: plano vertical del brazo, q1 queda fijo y solo se mueven q2 y q3';
            app.DdContPlano.ValueChangedFcn = @(~,~) app.cb_cont_plano(true);
            [app.ContFijoLabel, app.ContFijoEdit] = app.mkCampo(g, 2, 'q1 del plano [deg]', 0, [-1000 1000]);
            [app.CircULabel, app.CircU] = app.mkCampo(g, 3, 'Centro r radial [mm]', 0, [-1000 1000]);
            [app.CircVLabel, app.CircV] = app.mkCampo(g, 4, 'Centro z [mm]', 0, [-1000 1000]);
            [~, app.CircR]       = app.mkCampo(g, 5, 'Radio [mm]', 30, [0.5 300]);
            [~, app.CircAng0]    = app.mkCampo(g, 6, 'Angulo inicial [deg]', 0, [-360 360]);
            [~, app.CircVueltas] = app.mkCampo(g, 7, 'Vueltas', 1, [0.05 50]);
            app.CircSentido = uidropdown(g, 'Items', {'Antihorario', 'Horario'});
            app.CircSentido.Layout.Row = 8; app.CircSentido.Layout.Column = 1;
            app.CircSep = uieditfield(g, 'numeric', 'Limits', [0.1 50], 'Value', 1);
            app.CircSep.Layout.Row = 8; app.CircSep.Layout.Column = 2;
            app.CircSep.Tooltip = 'Separacion entre puntos [mm]';
            app.BtnCircCentro = app.mkBtn(g, 'Centro = TCP actual', @() app.cb_cont_tomar("circ"));
            app.BtnCircCentro.Layout.Row = 9; app.BtnCircCentro.Layout.Column = 1;
            app.BtnCircGen = app.mkBtn(g, 'Generar circulo', @() app.cb_circ_generar(), ...
                'Calcula los puntos y la CI y los dibuja. No envia nada');
            app.BtnCircGen.Layout.Row = 9; app.BtnCircGen.Layout.Column = 2;
            app.BtnCircGen.FontWeight = 'bold';

            % ---------- archivo .mat ----------
            g = uigridlayout(tM, [5 3]);
            g.RowHeight = {22, 24, 30, 30, '1x'};
            g.ColumnWidth = {'1x', '1x', '1x'};
            g.RowSpacing = 6; g.Padding = [8 8 8 8];
            l = uilabel(g, 'Text', 'Contenido del .mat'); l.Layout.Row = 1; l.Layout.Column = [1 3];
            app.DdContContenido = uidropdown(g);
            app.DdContContenido.Layout.Row = 2; app.DdContContenido.Layout.Column = [1 3];
            app.DdContContenido.Items = {'q [grados]', 'q [radianes]', 'XYZ [m] (usa CI)', 'XYZ [mm] (usa CI)'};
            app.DdContContenido.ItemsData = {'deg', 'rad', 'm', 'mm'};
            app.DdContContenido.Value = 'deg';
            app.BtnContCargar  = app.mkBtn(g, 'Cargar .mat...', @() app.cb_cont_cargar());
            app.BtnContCargar.Layout.Row = 3; app.BtnContCargar.Layout.Column = [1 3];
            app.BtnContCargar.FontWeight = 'bold';
            app.BtnContGuardar = app.mkBtn(g, 'Guardar .mat...', @() app.cb_cont_guardar(), ...
                'Guarda Q (Nx3, grados), xyz (Nx3, m) y descripcion');
            app.BtnContGuardar.Layout.Row = 4; app.BtnContGuardar.Layout.Column = [1 2];
            app.BtnContLimpiar = app.mkBtn(g, 'Limpiar', @() app.cb_cont_limpiar());
            app.BtnContLimpiar.Layout.Row = 4; app.BtnContLimpiar.Layout.Column = 3;
            app.LblContArchivo = uilabel(g, 'Text', ['Variable Q / q / xyz / P o la primera matriz Nx3 (q1 q2 q3 o x y z) ' ...
                'o Nx2 (q2 q3 con q1 fijo; en XYZ = u v en el plano del circulo).']);
            app.LblContArchivo.Layout.Row = 5; app.LblContArchivo.Layout.Column = [1 3];
            app.LblContArchivo.WordWrap = 'on'; app.LblContArchivo.VerticalAlignment = 'top';
            app.LblContArchivo.FontSize = 10; app.LblContArchivo.FontColor = [0.4 0.4 0.4];

            % ---------- grabar a mano ----------
            g = uigridlayout(tE, [7 2]);
            g.RowHeight = {48, 22, 22, 30, 30, 30, '1x'};
            g.ColumnWidth = {'1x', '1x'};
            g.RowSpacing = 5; g.Padding = [8 8 8 8];
            l = uilabel(g, 'Text', ['Suelta los motores (M,0), moves el brazo con la mano y se guardan los angulos de los ' ...
                'encoders. Al parar queda como trayectoria continua: se dibuja, se guarda en .mat y se ejecuta con la ' ...
                'velocidad / ida y vuelta de abajo.']);
            l.Layout.Row = 1; l.Layout.Column = [1 2];
            l.WordWrap = 'on'; l.VerticalAlignment = 'top'; l.FontSize = 10; l.FontColor = [0.4 0.4 0.4];
            l = uilabel(g, 'Text', 'Guardar cada [deg]'); l.Layout.Row = 2; l.Layout.Column = 1;
            app.EnsUmbral = uieditfield(g, 'numeric', 'Limits', [0.05 10], 'Value', 0.5);
            app.EnsUmbral.Layout.Row = 2; app.EnsUmbral.Layout.Column = 2;
            app.EnsUmbral.Tooltip = 'Se guarda un punto nuevo cuando alguna articulacion se movio esto desde el ultimo';
            app.ChkEnsSuavizar = uicheckbox(g, 'Text', 'Suavizar al terminar (promedio de 5)', 'Value', true);
            app.ChkEnsSuavizar.Layout.Row = 3; app.ChkEnsSuavizar.Layout.Column = [1 2];
            app.BtnEnsGrabar = app.mkBtn(g, 'Soltar y grabar', @() app.cb_ens_grabar(), ...
                'Manda M,0 y empieza a guardar los encoders', true);
            app.BtnEnsGrabar.Layout.Row = 4; app.BtnEnsGrabar.Layout.Column = 1;
            app.BtnEnsParar = app.mkBtn(g, 'Parar grabacion', @() app.cb_ens_parar());
            app.BtnEnsParar.Layout.Row = 4; app.BtnEnsParar.Layout.Column = 2;
            app.BtnEnsParar.FontWeight = 'bold';
            app.BtnEnsMotOn = app.mkBtn(g, 'Motores ON (M,1)', @() app.cb_ens_motores_on(), ...
                'Vuelve a dar torque. El firmware toma la posicion de los encoders (no salta)');
            app.BtnEnsMarcar = app.mkBtn(g, 'Marcar punto de paso', @() app.cb_ens_marcar("paso"), ...
                'Guarda la posicion actual como punto critico: no se mueve al suavizar');
            app.BtnEnsMarcar.Layout.Row = 5; app.BtnEnsMarcar.Layout.Column = 1;
            app.BtnEnsMarcar.BackgroundColor = [1 0.92 0.55];
            app.BtnEnsMarcar.Tooltip = 'El brazo pasa si o si por esta posicion, sin frenar (no se mueve al suavizar)';
            app.BtnEnsParada = app.mkBtn(g, 'Marcar parada', @() app.cb_ens_marcar("parada"), ...
                'El robot frena hasta quedar quieto en esta posicion, espera la pausa y sigue (tomar / soltar)');
            app.BtnEnsParada.Layout.Row = 5; app.BtnEnsParada.Layout.Column = 2;
            app.BtnEnsParada.BackgroundColor = [1 0.70 0.65];
            app.BtnEnsMotOn.Layout.Row = 6; app.BtnEnsMotOn.Layout.Column = 1;
            app.BtnEnsGuardar = app.mkBtn(g, 'Guardar .mat...', @() app.cb_cont_guardar(), ...
                'Guarda la trayectoria grabada (Q en grados, xyz, descripcion)');
            app.BtnEnsGuardar.Layout.Row = 6; app.BtnEnsGuardar.Layout.Column = 2;
            app.LblEnsInfo = uilabel(g, 'Text', 'Sin grabar.');
            app.LblEnsInfo.Layout.Row = 7; app.LblEnsInfo.Layout.Column = [1 2];
            app.LblEnsInfo.WordWrap = 'on'; app.LblEnsInfo.VerticalAlignment = 'top';
            app.LblEnsInfo.FontWeight = 'bold';

            % ---------- suavizar ----------
            g = uigridlayout(tS, [8 2]);
            g.RowHeight = {30, 22, 22, 22, 22, 28, 28, '1x'};
            g.ColumnWidth = {'1x', 110};
            g.RowSpacing = 5; g.Padding = [8 8 8 8];
            l = uilabel(g, 'Text', ['Suaviza la trayectoria cargada y la vuelve a muestrear con puntos equidistantes ' ...
                'en q. El inicio, el fin y los puntos de paso y de parada no se mueven.']);
            l.Layout.Row = 1; l.Layout.Column = [1 2];
            l.WordWrap = 'on'; l.VerticalAlignment = 'top'; l.FontSize = 10; l.FontColor = [0.4 0.4 0.4];
            [~, app.SuavVentana] = app.mkCampo(g, 2, 'Ventana de suavizado [puntos] (1 = no)', 9, [1 201]);
            app.SuavVentana.RoundFractionalValues = 'on';
            app.SuavVentana.Tooltip = 'Mas grande = mas suave, pero se aparta mas del camino original';
            [~, app.SuavSep] = app.mkCampo(g, 3, 'Separacion nueva [deg en q]', 0.5, [0.02 10]);
            app.SuavSep.Tooltip = ['Distancia entre puntos en el espacio articular. Ojo con los puntos/s: ' ...
                'a 100 deg/s con 0.5 deg salen 200 puntos/s (el serie llega a ~150)'];
            l = uilabel(g, 'Text', 'Puntos de paso (n de punto)'); l.Layout.Row = 4; l.Layout.Column = 1;
            app.SuavCrit = uieditfield(g, 'text');
            app.SuavCrit.Layout.Row = 4; app.SuavCrit.Layout.Column = 2;
            app.SuavCrit.Tooltip = ['Puntos por donde el brazo pasa si o si, SIN frenar (no se mueven al suavizar). ' ...
                'Separados por coma. Rombos amarillos. El n de punto es el eje x del grafico q vs punto'];
            app.SuavCrit.ValueChangedFcn = @(~,~) app.cb_crit_editar();
            l = uilabel(g, 'Text', 'Puntos de parada (n de punto)'); l.Layout.Row = 5; l.Layout.Column = 1;
            app.SuavStop = uieditfield(g, 'text');
            app.SuavStop.Layout.Row = 5; app.SuavStop.Layout.Column = 2;
            app.SuavStop.Tooltip = ['Puntos donde el robot FRENA, se queda quieto la pausa y sigue (tampoco se ' ...
                'mueven al suavizar). Separados por coma. Cuadrados rojos'];
            app.SuavStop.ValueChangedFcn = @(~,~) app.cb_crit_editar();
            app.BtnSuavizar = app.mkBtn(g, 'Suavizar y remuestrear', @() app.cb_cont_suavizar());
            app.BtnSuavizar.Layout.Row = 6; app.BtnSuavizar.Layout.Column = [1 2];
            app.BtnSuavizar.FontWeight = 'bold';
            app.BtnDeshacer = app.mkBtn(g, 'Deshacer', @() app.cb_cont_deshacer());
            app.BtnDeshacer.Layout.Row = 7; app.BtnDeshacer.Layout.Column = [1 2];
            app.LblSuavInfo = uilabel(g, 'Text', '');
            app.LblSuavInfo.Layout.Row = 8; app.LblSuavInfo.Layout.Column = [1 2];
            app.LblSuavInfo.WordWrap = 'on'; app.LblSuavInfo.VerticalAlignment = 'top';

            % ================= ejecucion =================
            pe = uipanel(G, 'Title', 'Ejecucion continua (S / P / E)');
            pe.Layout.Row = 2; pe.Layout.Column = 1;
            g = uigridlayout(pe, [9 4]);
            g.RowHeight = {23, 23, 23, 23, 23, 30, 26, 20, '1x'};
            g.ColumnWidth = {'1x', 62, 70, 58};
            g.RowSpacing = 5; g.Padding = [8 8 8 8];
            app.DdContVelModo = uidropdown(g);
            app.DdContVelModo.Layout.Row = 1; app.DdContVelModo.Layout.Column = [1 3];
            app.DdContVelModo.Items = {'Velocidad articular [deg/s]', 'Velocidad TCP constante [mm/s]'};
            app.DdContVelModo.Tooltip = ['Articular: velocidad constante sobre la trayectoria en q (el TCP varia). ' ...
                'TCP: se manda un S nuevo cuando cambia la velocidad articular necesaria'];
            app.DdContVelModo.ValueChangedFcn = @(~,~) app.cont_info();
            app.ContVel = uieditfield(g, 'numeric', 'Limits', [0.1 200], 'Value', 15);
            app.ContVel.Layout.Row = 1; app.ContVel.Layout.Column = 4;
            app.ContVel.ValueChangedFcn = @(~,~) app.cont_info();
            l = uilabel(g, 'Text', 'Aceleracion [deg/s2]'); l.Layout.Row = 2; l.Layout.Column = [1 3];
            app.ContAcc = uieditfield(g, 'numeric', 'Limits', [1 1000], 'Value', 60);
            app.ContAcc.Layout.Row = 2; app.ContAcc.Layout.Column = 4;
            app.ChkContIdaVuelta = uicheckbox(g, 'Text', 'Ida y vuelta', 'Value', true);
            app.ChkContIdaVuelta.Layout.Row = 3; app.ChkContIdaVuelta.Layout.Column = 1;
            app.ChkContIdaVuelta.ValueChangedFcn = @(~,~) app.cont_info();
            l = uilabel(g, 'Text', 'Repetir', 'HorizontalAlignment', 'right'); l.Layout.Row = 3; l.Layout.Column = [2 3];
            app.ContRep = uieditfield(g, 'numeric', 'Limits', [1 100], 'Value', 1, 'RoundFractionalValues', 'on');
            app.ContRep.Layout.Row = 3; app.ContRep.Layout.Column = 4;
            app.ContRep.Tooltip = 'Cantidad de pasadas. Sin ida y vuelta, entre pasadas vuelve del final al inicio';
            app.ContRep.ValueChangedFcn = @(~,~) app.cont_info();
            app.ChkFrenarCrit = uicheckbox(g, 'Text', 'Frenar en las paradas', 'Value', true);
            app.ChkFrenarCrit.Layout.Row = 4; app.ChkFrenarCrit.Layout.Column = [1 2];
            app.ChkFrenarCrit.Tooltip = ['El robot frena hasta quedar quieto en cada punto de parada (cuadrado rojo), ' ...
                'espera la pausa y arranca de nuevo con su rampa de aceleracion'];
            app.ChkFrenarCrit.ValueChangedFcn = @(~,~) app.cont_info();
            l = uilabel(g, 'Text', 'Pausa [s]', 'HorizontalAlignment', 'right'); l.Layout.Row = 4; l.Layout.Column = 3;
            app.ContPausa = uieditfield(g, 'numeric', 'Limits', [0 60], 'Value', 0.5);
            app.ContPausa.Layout.Row = 4; app.ContPausa.Layout.Column = 4;
            app.ContPausa.Tooltip = 'Tiempo que se queda quieto en cada punto critico (para tomar o soltar)';
            app.ContPausa.ValueChangedFcn = @(~,~) app.cont_info();
            l = uilabel(g, 'Text', 'Frenado en paradas: de v constante a 0 en [s]');
            l.Layout.Row = 5; l.Layout.Column = [1 3];
            app.ContTFreno = uieditfield(g, 'numeric', 'Limits', [0 10], 'Value', 0.6);
            app.ContTFreno.Layout.Row = 5; app.ContTFreno.Layout.Column = 4;
            app.ContTFreno.Tooltip = ['Tiempo que tarda en frenar de la velocidad de crucero a 0 al llegar a cada parada. ' ...
                '0 = frena con la aceleracion de arriba. El firmware igual limita a JOINT_AMAX de cada eje'];

            app.BtnContEjecutar = app.mkBtn(g, [app.S_PLAY ' Ejecutar'], @() app.cb_cont_ejecutar(false), ...
                'G al primer punto (velocidad P2P del control manual) y despues el stream S/P/E sin frenar');
            app.BtnContEjecutar.Layout.Row = 6; app.BtnContEjecutar.Layout.Column = 1;
            app.BtnContEjecutar.FontWeight = 'bold';
            app.BtnContIrInicio = app.mkBtn(g, 'Ir al inicio', @() app.cb_cont_ejecutar(true), ...
                'Solo lleva el robot al primer punto (G)');
            app.BtnContIrInicio.Layout.Row = 6; app.BtnContIrInicio.Layout.Column = [2 3];
            app.BtnContDetener = app.mkBtn(g, 'Detener', @() app.cb_cont_detener(), ...
                'Envia X: frena siguiendo la trayectoria. No es una parada de emergencia', true);
            app.BtnContDetener.Layout.Row = 6; app.BtnContDetener.Layout.Column = 4;
            app.LblContEstado = uilabel(g, 'Text', 'Inactiva', 'FontWeight', 'bold');
            app.LblContEstado.Layout.Row = 8; app.LblContEstado.Layout.Column = [1 4];
            app.LblContProg = uilabel(g, 'Text', '');
            app.LblContProg.Layout.Row = 9; app.LblContProg.Layout.Column = [1 4];
            app.BtnRegVer = app.mkBtn(g, 'Ver registro', @() app.cb_reg_ver(), ...
                'Graficos de referencia del firmware y encoders contra la trayectoria deseada');
            app.BtnRegVer.Layout.Row = 7; app.BtnRegVer.Layout.Column = 1;
            app.BtnRegGuardar = app.mkBtn(g, 'Guardar registro...', @() app.cb_reg_guardar(), ...
                'Guarda t, qref, qenc, Q_deseada, xyz_* en un .mat');
            app.BtnRegGuardar.Layout.Row = 7; app.BtnRegGuardar.Layout.Column = [2 4];
            app.LblContProg.WordWrap = 'on'; app.LblContProg.VerticalAlignment = 'top';

            % ================= graficos 2D =================
            pg = uipanel(G, 'Title', ['Espacio de trabajo y limites   (verde = alcanzable / limites,  ' ...
                'o naranja = fuera de limites,  x roja = fuera de alcance,  cuadrado violeta = salto)']);
            % XZ y XY son CORTES (y fija / z fija) del espacio de trabajo, no proyecciones
            tip = ['Zona verde de XZ = lo alcanzable (con los limites de q1, q2 y q3) en la y media de la trayectoria. ' ...
                'XY = lo alcanzable en la z media. Si la trayectoria cambia de y o de z, las marcas (CI exacta) mandan.'];
            try
                pg.Tooltip = tip;
            catch
            end
            pg.Layout.Row = [1 2]; pg.Layout.Column = 2;
            g = uigridlayout(pg, [3 2]);
            g.RowHeight = {'1x', '1x', 64};
            g.ColumnWidth = {'1x', '1x'};
            g.Padding = [4 4 4 4]; g.RowSpacing = 4; g.ColumnSpacing = 4;
            app.AxLat = uiaxes(g); app.AxLat.Layout.Row = 1; app.AxLat.Layout.Column = 1;
            app.AxSup = uiaxes(g); app.AxSup.Layout.Row = 1; app.AxSup.Layout.Column = 2;
            app.AxQ23 = uiaxes(g); app.AxQ23.Layout.Row = 2; app.AxQ23.Layout.Column = 1;
            app.AxQt  = uiaxes(g); app.AxQt.Layout.Row = 2;  app.AxQt.Layout.Column = 2;
            for ax = [app.AxLat app.AxSup app.AxQ23 app.AxQt]
                ax.FontSize = 8;
                try, ax.Toolbar.Visible = 'on'; catch, end
            end
            app.LblContInfo = uilabel(g, 'Text', '');
            app.LblContInfo.Layout.Row = 3; app.LblContInfo.Layout.Column = [1 2];
            app.LblContInfo.WordWrap = 'on';
            app.LblContInfo.VerticalAlignment = 'top';
            app.LblContInfo.FontSize = 11;

            app.CtrlsCont = {app.R1x, app.R1y, app.R1z, app.R2x, app.R2y, app.R2z, app.BtnR1Tcp, app.BtnR2Tcp, ...
                app.RectSep, app.BtnRectGen, app.DdContPlano, app.ContFijoEdit, app.CircU, app.CircV, ...
                app.CircR, app.CircAng0, app.CircVueltas, app.CircSentido, app.CircSep, app.BtnCircCentro, ...
                app.BtnCircGen, app.DdContContenido, app.BtnContCargar, app.BtnContLimpiar, ...
                app.DdContVelModo, app.ContVel, app.ContAcc, app.ChkContIdaVuelta, app.ContRep, ...
                app.EnsUmbral, app.ChkEnsSuavizar, app.SuavVentana, app.SuavSep, app.SuavCrit, ...
                app.ChkFrenarCrit, app.ContPausa, app.SuavStop, app.ContTFreno};
        end

        function [lb, ed] = mkCampo(~, g, fila, txt, val, lim)
            lb = uilabel(g, 'Text', txt);
            lb.Layout.Row = fila; lb.Layout.Column = 1;
            ed = uieditfield(g, 'numeric', 'Limits', lim, 'Value', val);
            ed.Layout.Row = fila; ed.Layout.Column = 2;
        end
    end

    %% ---------------------------------------------------------------
    %  Creacion y eliminacion de la app
    %% ---------------------------------------------------------------
    methods (Access = public)
        function app = GUI_control
            createComponents(app)
            registerApp(app, app.UIFigure)
            runStartupFcn(app, @startupFcn)

            if nargout == 0
                clear app
            end
        end

        function delete(app)
            delete(app.UIFigure)
        end
    end
end
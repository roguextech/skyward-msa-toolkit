function [data_para, Tf, Yf, bound_value] = descent_parachute6dof(Ta, Ya, settings, uw, vw, ww, uncert)
% This function computes the descent phase of a rocket connected to 2
% parachute: a drogue and a main one. Due to model complexity, the problem
% is divided into 3 phases:
%    - PHASE 1: drogue extraction and first desccent phase
%    - PHASE 2: drogue pulls out the main parachute till main shock cord
%               has reached its nominal length
%    - PHASE 3: final descent phase with both main and drogue.
%
% [data_para] = descent_parachute6dof(Ta, Ya, settings)
%
% -------------------------------------------------------------------------
% INPUT PARAMETERS:
% -  Ta            [nx1]          integration time from ascent          [s]
% -  Ya            [nx20]         state vector of from ascent           [-]
% -  settings      [1x1 struct]   settings struct                       [-]
% -  [uw,vw,ww]    [1x3]          NED wind velocity vector            [m/s]    
%
% -------------------------------------------------------------------------
% OUTPUT PARAMETERS:
% -  data_para     [2x1 struct]  descent phases data                    [-]
%
% -------------------------------------------------------------------------

    %% PHASE 0 : Set up data
    % drogue parameters
    drg.S     = settings.para(1).S;               % [m^2]  Drogue surface
    drg.mass  = settings.para(1).mass;            % [kg]   Drogue mass
    drg.CD    = settings.para(1).CD;              % [-]    Drogue drag coefficient
    drg.CX    = settings.para(1).CX;              % [-]    Drogue opening force coefficient
    drg.m     = settings.para(1).m;               % [-]    Exponent of the time surface model
    drg.nf    = settings.para(1).nf;              % [-]    Adimensional opening time coefficient
    drg.z_cut = settings.para(1).z_cut;           % [m]    Final altitude of drogue
    drg.L     = settings.para(1).ShockCord_L;     % [m]    Drogue shock cord length
    drg.K     = settings.para(1).ShockCord_k;     % [N/m]  Shock Cord elastic constant
    drg.C     = settings.para(1).ShockCord_c;     % [Ns/m] Shock Cord damping coefficient
    drg.Vexit = settings.para(1).Vexit;           % [m/s]  Speed of nosecone extraction

    % main parameters
    main.S     = settings.para(2).S;               % [m^2]  Main surface
    main.mass  = settings.para(2).mass;            % [kg]   Main mass
    main.CD    = settings.para(2).CD;              % [-]    Main drag coefficient
    main.CX    = settings.para(2).CX;              % [-]    Main opening force coefficient
    main.m     = settings.para(2).m;               % [-]    Exponent of the time surface model
    main.nf    = settings.para(2).nf;              % [-]    Adimensional opening time coefficient
    main.z_cut = settings.para(2).z_cut;           % [m]    Final altitude of main
    main.L     = settings.para(2).ShockCord_L;     % [m]    Main shock cord length
    main.K     = settings.para(2).ShockCord_k;     % [N/m]  Shock Cord elastic constant
    main.C     = settings.para(2).ShockCord_c;     % [Ns/m] Shock Cord damping coefficient

    % Creating the output struct
    data_para = cell(settings.Npara, 1);

    %% PHASE 1 : Drogue descent
    % Initial conditions
    posPara0 = quatrotate(quatconj(Ya(end,10:13)),...                         % (NED) position of the point from where the parachute will be deployed
        [(settings.xcg(2)-settings.Lnc) 0 0]) + Ya(end,1:3);
    velPara0 = quatrotate(quatconj(Ya(end,10:13)),Ya(end,4:6)+...             % (NED) Initial parachute position
        [drg.Vexit 0 0]) + quatrotate(quatconj(Ya(end,10:13)),...
        cross(Ya(end,7:9),[(settings.xcg(2)-settings.Lnc) 0 0]));
    Y0p = [Ya(end,1:13) posPara0 velPara0 Ya(end,18:20)];                     % Initializing starting state vector

    t0p = Ta(end);
    tf  = settings.ode.final_time;
    
    Yf = Ya(:,1:13);
    Tf = Ta;

    % ODE
    options1 = odeset('Events', @event_paraCut);
    [Tp1, Yp1] = ode113(@drogue_descent, [t0p, tf], Y0p,...
        options1, settings, uw, vw, ww, drg, t0p, uncert);

    % Saving additional data
    [data_para{1}] = RecallOdeFcn(@drogue_descent, Tp1, Yp1, settings, uw, vw, ww, drg, t0p, uncert);
    data_para{1}.state.Y = Yp1;
    data_para{1}.state.T = Tp1;
    
    Yf = [Yf; Yp1(:,1:13)];
    Tf = [Tf; Tp1];
    
    % TIME, POSITION AND VELOCITY AT PARACHUTES DEPLOYMENT
    % Usefull values for the plots
    bound_value = struct;
    bound_value(1).t = Ta(end);
    bound_value(1).X = [Ya(end, 2), Ya(end, 1), -Ya(end, 3)];
    bound_value(1).V = quatrotate(quatconj(Ya(end, 10:13)), Ya(end, 4:6));

    %% PHASE 2 : Main Extraction
    % Initial conditions
    posDrg0 = Yp1(end,14:16);
    posMain0 = quatrotate(quatconj(Yp1(end,10:13)),...  
        [(settings.xcg(2)-settings.Lnc) 0 0]) + Yp1(end,1:3);
    velDrg0 = Yp1(end,17:19);
    velMain0 = quatrotate(quatconj(Yp1(end,10:13)),Yp1(end,4:6)) +...
        quatrotate(quatconj(Yp1(end,10:13)),cross(Yp1(end,7:9),...
        [(settings.xcg(2)-settings.Lnc) 0 0]));

    Y0p = [Yp1(end,1:13) posDrg0 velDrg0 posMain0 velMain0 Yp1(end,20:22)];
    t0p = Tp1(end);

    % ODE
    options2 = odeset('Events', @event_mainExt);
    [Tp2, Yp2] = ode113(@main_extraction, [t0p, tf], Y0p,...
        options2, settings, uw, vw, ww, drg, main, t0p, uncert);

    % Saving additional data
    [data_paraf1, data_para{2}] = RecallOdeFcn(@main_extraction, Tp2, Yp2, settings, uw, vw, ww, drg, main, t0p, uncert);
    
    Yf = [Yf; Yp2(:,1:13)];
    Tf = [Tf; Tp2];

    % Filling with old data
    % Drogue
    data_para{1}.state.Y        = [data_para{1}.state.Y; Yp2(:,1:19) Yp2(:,26:28)];
    data_para{1}.state.T        = [data_para{1}.state.T; Tp2];
    data_para{1}.integration.t  = [data_para{1}.integration.t, data_paraf1.integration.t];
    data_para{1}.interp.alt     = [data_para{1}.interp.alt, data_paraf1.interp.alt];
    data_para{1}.wind.body_wind = [data_para{1}.wind.body_wind, data_paraf1.wind.body_wind];
    data_para{1}.wind.NED_wind  = [data_para{1}.wind.NED_wind, data_paraf1.wind.NED_wind];
    data_para{1}.velocities     = [data_para{1}.velocities; data_paraf1.velocities];
    data_para{1}.air.rho        = [data_para{1}.air.rho, data_paraf1.air.rho];
    data_para{1}.air.P          = [data_para{1}.air.P, data_paraf1.air.P];
    data_para{1}.accelerations.body_acc = [data_para{1}.accelerations.body_acc; data_paraf1.accelerations.body_acc];
    data_para{1}.interp.M       = [data_para{1}.interp.M, data_paraf1.interp.M];
    data_para{1}.interp.alpha   = [data_para{1}.interp.alpha, data_paraf1.interp.alpha];
    data_para{1}.interp.beta    = [data_para{1}.interp.beta, data_paraf1.interp.beta];
    data_para{1}.forces.T       = [data_para{1}.forces.T, data_paraf1.forces.T];
    data_para{1}.forces.T_chord = [data_para{1}.forces.T_chord, data_paraf1.forces.T_chord];
    data_para{1}.SCD            = [data_para{1}.SCD, data_paraf1.SCD];
    data_para{1}.accelerations.ang_acc = [data_para{1}.accelerations.ang_acc; data_paraf1.accelerations.ang_acc];

    % Main
    data_para{2}.state.Y        = [Yp2(:,1:13) Yp2(:,20:28)];
    data_para{2}.state.T        = Tp2; 
    
    bound_value(2).t = Tp1(end);
    bound_value(2).X = [Yp1(end, 2), Yp1(end, 1), -Yp1(end, 3)];
    bound_value(2).V = quatrotate(quatconj(Yp1(end, 10:13)), Yp1(end, 4:6));
    
    %% PHASE 3 : Main descent
    % Initial conditions
    posDrg0 = Yp2(end,14:16);
    posMain0 = Yp2(end,20:22);
    velDrg0 = Yp2(end,17:19);
    velMain0 = Yp2(end,23:25);

    Y0p = Yp2(end,1:28);
    t0p = Tp2(end);
    
    % ODE
    options3 = odeset('Events', @event_landRocket);
    [Tp3, Yp3] = ode113(@main_descent, [t0p, tf], Y0p,...
        options3, settings, uw, vw, ww, drg, main, t0p, uncert);
    
    % Saving additional data
    [data_paraf1, data_paraf2] = RecallOdeFcn(@main_descent, Tp3, Yp3, settings, uw, vw, ww, drg, main, t0p, uncert);
    
    Yf = [Yf; Yp3(:,1:13)];
    Tf = [Tf; Tp3];
    
    % Filling with old data
    % Drogue
    data_para{1}.state.Y        = [data_para{1}.state.Y; Yp3(:,1:19) Yp3(:,26:28)];
    data_para{1}.state.T        = [data_para{1}.state.T; Tp3];
    data_para{1}.integration.t  = [data_para{1}.integration.t, data_paraf1.integration.t];
    data_para{1}.interp.alt     = [data_para{1}.interp.alt, data_paraf1.interp.alt];
    data_para{1}.wind.body_wind = [data_para{1}.wind.body_wind, data_paraf1.wind.body_wind];
    data_para{1}.wind.NED_wind  = [data_para{1}.wind.NED_wind, data_paraf1.wind.NED_wind];
    data_para{1}.velocities     = [data_para{1}.velocities; data_paraf1.velocities];
    data_para{1}.air.rho        = [data_para{1}.air.rho, data_paraf1.air.rho];
    data_para{1}.air.P          = [data_para{1}.air.P, data_paraf1.air.P];
    data_para{1}.accelerations.body_acc = [data_para{1}.accelerations.body_acc; data_paraf1.accelerations.body_acc];
    data_para{1}.interp.M       = [data_para{1}.interp.M, data_paraf1.interp.M];
    data_para{1}.interp.alpha   = [data_para{1}.interp.alpha, data_paraf1.interp.alpha];
    data_para{1}.interp.beta    = [data_para{1}.interp.beta, data_paraf1.interp.beta];
    data_para{1}.forces.T       = [data_para{1}.forces.T, data_paraf1.forces.T];
    data_para{1}.forces.T_chord = [data_para{1}.forces.T_chord, data_paraf1.forces.T_chord];
    data_para{1}.SCD            = [data_para{1}.SCD, data_paraf1.SCD];
    data_para{1}.accelerations.ang_acc = [data_para{1}.accelerations.ang_acc; data_paraf1.accelerations.ang_acc];

    % Main
    data_para{2}.state.Y        = [data_para{2}.state.Y; Yp3(:,1:13) Yp3(:,20:28)];
    data_para{2}.state.T        = [data_para{2}.state.T; Tp3];
    data_para{2}.integration.t  = [data_para{2}.integration.t, data_paraf2.integration.t];
    data_para{2}.interp.alt     = [data_para{2}.interp.alt, data_paraf2.interp.alt];
    data_para{2}.wind.body_wind = [data_para{2}.wind.body_wind, data_paraf2.wind.body_wind];
    data_para{2}.wind.NED_wind  = [data_para{2}.wind.NED_wind, data_paraf2.wind.NED_wind];
    data_para{2}.velocities     = [data_para{2}.velocities; data_paraf2.velocities];
    data_para{2}.air.rho        = [data_para{2}.air.rho, data_paraf2.air.rho];
    data_para{2}.air.P          = [data_para{2}.air.P, data_paraf2.air.P];
    data_para{2}.accelerations.body_acc = [data_para{2}.accelerations.body_acc; data_paraf2.accelerations.body_acc];
    data_para{2}.interp.M       = [data_para{2}.interp.M, data_paraf2.interp.M];
    data_para{2}.interp.alpha   = [data_para{2}.interp.alpha, data_paraf2.interp.alpha];
    data_para{2}.interp.beta    = [data_para{2}.interp.beta, data_paraf2.interp.beta];
    data_para{2}.forces.T       = [data_para{2}.forces.T, data_paraf2.forces.T];
    data_para{2}.forces.T_chord = [data_para{2}.forces.T_chord, data_paraf2.forces.T_chord];
    data_para{2}.SCD            = [data_para{2}.SCD, data_paraf2.SCD];
    data_para{2}.accelerations.ang_acc = [data_para{2}.accelerations.ang_acc; data_paraf2.accelerations.ang_acc];
    
end

function [value, isterminal, direction] = event_paraCut(~, Y, settings, varargin)
    x = Y(1);
    y = Y(2);
    z = -Y(3);

    para = varargin{4};

    if settings.terrain
        zloc = -settings.funZ(x,y);
        if zloc > 859
            zloc = 859;
        end

        if zloc < -845
            zloc = -845;
        end

        value = z - zloc - para.z_cut;
    else
        value = z - para.z_cut;
    end

    isterminal = 1;
    direction = 0;
end
function [value, isterminal, direction] = event_mainExt(~, Y, settings, varargin)
    para = varargin{5};
    
    pos_para = [Y(20) Y(21) Y(22)];
    rel_pos = norm(pos_para - ([Y(1) Y(2) Y(3)] + ...
        quatrotate(quatconj([Y(10) Y(11) Y(12) Y(13)]),...
        [(settings.xcg(2)-settings.Lnc) 0 0])));
    
    value = para.L - rel_pos;
    
    isterminal = 1;
    direction = 0;
end
function [value, isterminal, direction] = event_landRocket(~, Y, settings, varargin)
    x = Y(1);
    y = Y(2);
    z = -Y(3);
    if settings.terrain
        zloc = -settings.funZ(x,y);
        if zloc > 853
            zloc = 853;
        end
        if zloc < -656
            zloc = -656;
        end
        value = z - zloc;
    else
        value = z;
    end
    isterminal = 1;
    direction = 0;
end
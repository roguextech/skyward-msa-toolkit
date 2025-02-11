function [data_para, Tp, Yp, bound_value] = descentParachute6Dof(Ta, Ya, settings)
%{

descentParachute6Dof - This function computes the descent phase of a rocket 
                       connected to 2 parachute: a drogue and a main one. 
                       Due to model complexity, the problem is divided into 
                       3 phases:
                            - PHASE 1: drogue extraction and first descent 
                                       phase.
                            - PHASE 2: drogue pulls out the main parachute 
                                       till main shock cord has reached its 
                                       nominal length
                            - PHASE 3: final descent phase with both main 
                                       and drogue.


     INPUT PARAMETERS:
     -  Ta, array, [n� variations, 1], integration time from ascent          
     -  Ya, array, [n� variations, 16], state vector from ascent              
     -  settings, struct, stores data of the rocket and of the simulation    


     OUTPUT PARAMETERS:
     -  data_para, cell, (n� parachutes, 1), descent phase data                  
     -  Tp, array, descent integration time vector       
     -  Yp, array, descent state matrix                  
     -  bound_value, struct, boundary values for the plots         

     CALLED FUNCTIONS: descentDrogue, extractionMain, descentMain, recallOdeFcn

     REVISIONS:
     - #0 16/12/2020, Release, Davide Rosato, Fiammetta Artioli
%}
    bound_value = struct;
    bound_value(1).t = Ta(end);
    bound_value(1).X = [Ya(end, 2), Ya(end, 1), -Ya(end, 3)];
    bound_value(1).V = quatrotate(quatconj(Ya(end, 10:13)), Ya(end, 4:6));
    
    % Creating the output struct
    data_para = cell(settings.Npara, 1);

    %% PHASE 1 : Drogue descent
    % Initial conditions
    posPara0 = quatrotate(quatconj(Ya(end,10:13)),...                         % (NED) Initial drogue position
        [(settings.xcg(2)-settings.Lnose) 0 0]) + Ya(end,1:3);
    
    velPara0 = quatrotate(quatconj(Ya(end,10:13)),Ya(end,4:6)+...             % (NED) Initial drogue velocity
        [settings.para(1).Vexit 0 0]) + quatrotate(quatconj(Ya(end,10:13)),...
        cross(Ya(end,7:9),[(settings.xcg(2)-settings.Lnose) 0 0]));
    
    Y0p = [Ya(end, 1:16) posPara0 velPara0];                                  % Initializing starting state vector

    t0p = Ta(end);
    tf  = settings.ode.finalTime;
    
    % ODE
    settings.paraN = 1; settings.paraNumber = 1; % drogue only
    settings.t0p = t0p;
    [Tp1, Yp1] = ode113(@descentDrogue, [t0p, tf], Y0p, settings.ode.optionsDrogue6DOF,...
        settings);

    % Saving additional data
    [data_para{1}] = recallOdeFcn(@descentDrogue, Tp1, Yp1, settings);
    data_para{1}.state.Y = Yp1;
    data_para{1}.state.T = Tp1;
    
    data_para{1}.SCD = data_para{1}.SCD(1,:);
    data_para{1}.forces.T_chord = data_para{1}.forces.T_chord(1,:); 
    data_para{1}.forces.D = data_para{1}.forces.D(1,:);
    
    Yp = [Yp1, NaN*ones(size(Yp1,1),6)];
    Tp = Tp1;
    
    %% PHASE 2 : Main Extraction
    % Initial conditions
    posMain0 = quatrotate(quatconj(Yp1(end,10:13)),...                        % (NED) Initial main position
        [(settings.xcg(2)-settings.Lnose) 0 0]) + Yp1(end,1:3);

    velMain0 = quatrotate(quatconj(Yp1(end,10:13)),Yp1(end,4:6)) +...         % (NED) INitial main velocity
        quatrotate(quatconj(Yp1(end,10:13)),cross(Yp1(end,7:9),...
        [(settings.xcg(2)-settings.Lnose) 0 0]));

    Y0p = [Yp1(end,1:22) posMain0 velMain0];                                  % Initializing starting state vector
    t0p = Ta(end);

    % ODE
    settings.paraN = [1, 2];
    settings.t0p = t0p;
    [Tp2, Yp2] = ode113(@extractionMain, [Tp1(end), tf], Y0p,...
        settings.ode.optionsMainExt6DOF, settings);
 
    % Saving additional data
    [data_paraf1] = recallOdeFcn(@extractionMain, Tp2, Yp2, settings);
    
    Yp = [Yp; Yp2];
    Tp = [Tp; Tp2];

    % Filling with old data
    % Drogue
    data_para{1}.state.Y        = [data_para{1}.state.Y; Yp2(:,1:22)];
    data_para{1}.state.T        = [data_para{1}.state.T; Tp2];
    data_para{1}.integration.t  = [data_para{1}.integration.t, data_paraf1.integration.t];
    data_para{1}.interp.alt     = [data_para{1}.interp.alt, data_paraf1.interp.alt];
    data_para{1}.wind.body_wind = [data_para{1}.wind.body_wind, data_paraf1.wind.body_wind];
    data_para{1}.wind.NED_wind  = [data_para{1}.wind.NED_wind, data_paraf1.wind.NED_wind];
    data_para{1}.velocities     = [data_para{1}.velocities, data_paraf1.velocities];
    data_para{1}.air.rho        = [data_para{1}.air.rho, data_paraf1.air.rho];
    data_para{1}.air.P          = [data_para{1}.air.P, data_paraf1.air.P];
    data_para{1}.accelerations.body_acc = [data_para{1}.accelerations.body_acc, data_paraf1.accelerations.body_acc];
    data_para{1}.interp.M       = [data_para{1}.interp.M, data_paraf1.interp.M];
    data_para{1}.forces.T_chord = [data_para{1}.forces.T_chord, data_paraf1.forces.T_chord(1,:)];
    data_para{1}.forces.D       = [data_para{1}.forces.D, data_paraf1.forces.D(1,:)];
    data_para{1}.SCD            = [data_para{1}.SCD, data_paraf1.SCD(1,:)];
    data_para{1}.accelerations.ang_acc = [data_para{1}.accelerations.ang_acc, data_paraf1.accelerations.ang_acc];

    % Main
    data_para{2}.state.Y        = [Yp2(:,1:16) Yp2(:,23:28)];
    data_para{2}.state.T        = Tp2; 
    data_para{2}.integration.t  = data_paraf1.integration.t;
    data_para{2}.interp.alt     = data_paraf1.interp.alt;
    data_para{2}.wind.body_wind = data_paraf1.wind.body_wind;
    data_para{2}.wind.NED_wind  = data_paraf1.wind.NED_wind;
    data_para{2}.velocities     = data_paraf1.velocities;
    data_para{2}.air.rho        = data_paraf1.air.rho;
    data_para{2}.air.P          = data_paraf1.air.P;
    data_para{2}.accelerations.body_acc = data_paraf1.accelerations.body_acc;
    data_para{2}.interp.M       = data_paraf1.interp.M;
    data_para{2}.forces.T_chord = data_paraf1.forces.T_chord(2,:);
    data_para{2}.forces.D       = data_paraf1.forces.D(2,:);
    data_para{2}.SCD            = data_paraf1.SCD(2,:);
    data_para{2}.accelerations.ang_acc = data_paraf1.accelerations.ang_acc;

    %% PHASE 3 : Main descent
    % Initial conditions
    Y0p = Yp2(end,1:28);
    t0p = [Ta(end), Tp2(end)];
    
    % ODE
    settings.paraN = [1, 2];
    settings.t0p = t0p;
    [Tp3, Yp3] = ode113(@descentMain, [Tp2(end), tf], Y0p,...
        settings.ode.optionsMain6DOF, settings);
    
    % Saving additional data
    [data_paraf2] = recallOdeFcn(@descentMain, Tp3, Yp3, settings);
    
    Yp = [Yp; Yp3];
    Tp = [Tp; Tp3];
    
    % Filling with old data
    % Drogue
    data_para{1}.state.Y        = [data_para{1}.state.Y; Yp3(:,1:22)];
    data_para{1}.state.T        = [data_para{1}.state.T; Tp3];
    data_para{1}.integration.t  = [data_para{1}.integration.t, data_paraf2.integration.t];
    data_para{1}.interp.alt     = [data_para{1}.interp.alt, data_paraf2.interp.alt];
    data_para{1}.wind.body_wind = [data_para{1}.wind.body_wind, data_paraf2.wind.body_wind];
    data_para{1}.wind.NED_wind  = [data_para{1}.wind.NED_wind, data_paraf2.wind.NED_wind];
    data_para{1}.velocities     = [data_para{1}.velocities, data_paraf2.velocities];
    data_para{1}.air.rho        = [data_para{1}.air.rho, data_paraf2.air.rho];
    data_para{1}.air.P          = [data_para{1}.air.P, data_paraf2.air.P];
    data_para{1}.accelerations.body_acc = [data_para{1}.accelerations.body_acc, data_paraf2.accelerations.body_acc];
    data_para{1}.interp.M       = [data_para{1}.interp.M, data_paraf2.interp.M];
    data_para{1}.forces.T_chord = [data_para{1}.forces.T_chord, data_paraf2.forces.T_chord(1,:)];
    data_para{1}.forces.D       = [data_para{1}.forces.D, data_paraf2.forces.D(1,:)];
    data_para{1}.SCD            = [data_para{1}.SCD, data_paraf2.SCD(1,:)];
    data_para{1}.accelerations.ang_acc = [data_para{1}.accelerations.ang_acc, data_paraf2.accelerations.ang_acc];

    % Main
    data_para{2}.state.Y        = [data_para{2}.state.Y; Yp3(:,1:16) Yp3(:,23:28)];
    data_para{2}.state.T        = [data_para{2}.state.T; Tp3];
    data_para{2}.integration.t  = [data_para{2}.integration.t, data_paraf2.integration.t];
    data_para{2}.interp.alt     = [data_para{2}.interp.alt, data_paraf2.interp.alt];
    data_para{2}.wind.body_wind = [data_para{2}.wind.body_wind, data_paraf2.wind.body_wind];
    data_para{2}.wind.NED_wind  = [data_para{2}.wind.NED_wind, data_paraf2.wind.NED_wind];
    data_para{2}.velocities     = [data_para{2}.velocities, data_paraf2.velocities];
    data_para{2}.air.rho        = [data_para{2}.air.rho, data_paraf2.air.rho];
    data_para{2}.air.P          = [data_para{2}.air.P, data_paraf2.air.P];
    data_para{2}.accelerations.body_acc = [data_para{2}.accelerations.body_acc, data_paraf2.accelerations.body_acc];
    data_para{2}.interp.M       = [data_para{2}.interp.M, data_paraf2.interp.M];
    data_para{2}.forces.T_chord = [data_para{2}.forces.T_chord, data_paraf2.forces.T_chord(2,:)];
    data_para{2}.forces.D       = [data_para{2}.forces.D, data_paraf2.forces.D(2,:)];
    data_para{2}.SCD            = [data_para{2}.SCD, data_paraf2.SCD(2,:)];
    data_para{2}.accelerations.ang_acc = [data_para{2}.accelerations.ang_acc, data_paraf2.accelerations.ang_acc];
    
    bound_value(2).t = data_para{2}.state.T(1);
    bound_value(2).X = [data_para{2}.state.Y(1, 2), data_para{2}.state.Y(1, 1), -data_para{2}.state.Y(1, 3)];
    bound_value(2).V = quatrotate(quatconj(data_para{2}.state.Y(1, 10:13)), data_para{2}.state.Y(1, 4:6));
    bound_value(2).V(3) = -bound_value(2).V(3);
    
    bound_value(3).t = data_para{2}.state.T(end);
    bound_value(3).X = [data_para{2}.state.Y(end, 2), data_para{2}.state.Y(end, 1), -data_para{2}.state.Y(end, 3)];
    bound_value(3).V = quatrotate(quatconj(data_para{2}.state.Y(end, 10:13)), data_para{2}.state.Y(end, 4:6));
    bound_value(3).V(3) = -bound_value(3).V(3);
end
function [Tf, Yf, Ta, Ya, bound_value] = stdRun(settings)
%{ 

stdRun - This function runs a standard (non-stochastic) simulation

INTPUTS: 
            - settings, struct, stores data of the rocket and of the simulation.

OUTPUTS:
            - Tf, array, Total integration time vector; 
            - Yf, array, Total State Matrix;
            - Ta, array, [n° variations, 1], Ascent Integration time vector;
            - Ya, array, [n° variations, 16], Ascent State Matrix;
            - bound_value, struct, Useful values for the plots;

CALLED FUNCTIONS: windConstGenerator, ascent, recallOdeFcn, descentParachute, descentParachute6Dof.

REVISIONS:
- #0, Release, Ruben Di Battista
- #1, Revision, Francesco Colombi
- #2 09/10/2019, Revision, Adriano Filippo Inno
%}

if settings.wind.model && settings.wind.input
    error('Both wind model and input wind are true, select just one of them')
end

if settings.wind.HourMin ~= settings.wind.HourMax || settings.wind.DayMin ~= settings.wind.DayMax
    error('In standard simulations with the wind model the day and the hour of launch must be unique, check config.m')
end

if settings.OMEGAmin ~= settings.OMEGAmax || settings.PHImin ~= settings.PHImax
    error('In a single simulation the launchpad configuration has to be unique, check config.m')
end

if settings.para(settings.Npara).z_cut ~= 0 
    error('The landing will be not achived, check the final altitude of the last parachute in config.m')
end

if settings.upwind
    error('Upwind is available just in stochastich simulations, check config.m');
end

if settings.wind.input && not(all(settings.wind.input_uncertainty == 0))
    error('settings.wind.input_uncertainty is available just in stochastich simulations, set it null')
end

%% STARTING CONDITIONS
%%% Launchpad
settings.OMEGA = settings.OMEGAmin;
settings.PHI = settings.PHImin;

%%% Attitude
Q0 = angleToQuat(settings.PHI, settings.OMEGA, 0*pi/180)';

%%% State
X0 = [0 0 0]';
V0 = [0 0 0]';
W0 = [0 0 0]';

Y0a = [X0; V0; W0; Q0; settings.Ixxf; settings.Iyyf; settings.Izzf];

%% WIND GENERATION
if not(settings.wind.model) && not(settings.wind.input)
    [uw, vw, ww, ~] = windConstGenerator(settings.wind);
    settings.constWind = [uw, vw, ww];
    if ww ~= 0
        warning('Pay attention using vertical wind, there might be computational errors')
    end
    
end

tf = settings.ode.final_time;

%% ASCENT
% ascent phase computation
[Ta, Ya] = ode113(@ascent, [0, tf], Y0a, settings.ode.optionsasc1, settings); % till the apogee

if settings.para(1).delay ~= 0 % checking if the actuation delay is different from zero
    [Ta2, Ya2] = ode113(@ascent, [Ta(end), Ta(end) + settings.para(1).delay],... 
        Ya(end,:), settings.ode.optionsasc2, settings); % till end of the delay
    
    Ta = [Ta; Ta2(2:end ) ];
    Ya = [Ya; Ya2(2:end, :)];
end

[data_ascent] = recallOdeFcn(@ascent, Ta, Ya, settings);
data_ascent.state.Y = Ya;
data_ascent.state.T = Ta;
save('ascent_plot.mat', 'data_ascent');

%% PARATCHUTES
% Initial Condition are the last from ascent (need to rotate because
% velocities are in body axes)
if not(settings.descent6DOF)
    Y0p = [Ya(end, 1:3) quatrotate(quatconj(Ya(end, 10:13)),Ya(end, 4:6))];
    data_para = cell(settings.Npara, 1);
    Yf = Ya(:, 1:6);
    Tf  = Ta;
    t0p = Ta(end);

    for i = 1:settings.Npara
        para = i; settings.paraNumber = para;
        [Tp, Yp] = ode113(@descentParachute, [t0p, tf], Y0p, settings.ode.optionspara, settings);

        [data_para{para}] = recallOdeFcn(@descentParachute, Tp, Yp, settings);
        data_para{para}.state.Y = Yp;
        data_para{para}.state.T = Tp;

        % total state
        Yf = [Yf; Yp];
        Tf = [Tf; Tp];

        % updating ODE starting conditions
        Y0p = Yp(end, :);
        t0p = Tp(end);
    end
    
    % Usefull values for the plots
    bound_value = struct;
    bound_value(1).t = Ta(end);
    bound_value(1).X = [Ya(end, 2), Ya(end, 1), -Ya(end, 3)];
    bound_value(1).V = quatrotate(quatconj(Ya(end, 10:13)), Ya(end, 4:6));

    for i = 1:settings.Npara
        bound_value(i+1).t = data_para{i}.state.T(end);
        bound_value(i+1).X = [data_para{i}.state.Y(end, 2), data_para{i}.state.Y(end, 1), -data_para{i}.state.Y(end, 3)];
        bound_value(i+1).V = [data_para{i}.state.Y(end, 4), data_para{i}.state.Y(end, 5), -data_para{i}.state.Y(end, 6)];
    end

else
    Yf = [Ya(:, 1:16), NaN*ones(size(Ya,1),12)];
    Tf = Ta;
    
    [data_para, Tp, Yp, bound_value] = descentParachute6Dof(Ta, Ya, settings);
    
    % total state
    Yf = [Yf; Yp];
    Tf = [Tf; Tp];
end
    
save('descent_para_plot.mat', 'data_para')

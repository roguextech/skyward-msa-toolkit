function [Tf, Yf, Ta, Ya, bound_value] = std_run(settings)
%{ 

STD_RUN - This function runs a standard (non-stochastic) simulation

INTPUTS: 
            - settings, rocket data structure;

OUTPUTS:
            - Tf, Total integration time vector; 
            - Yf, Total State Matrix;
            - Ta, Ascent Integration time vector;
            - Ya, Ascent State Matrix;
            - bound_value, Usefull values for the plots;

Author: Ruben Di Battista
Skyward Experimental Rocketry | CRD Dept | crd@skywarder.eu
email: ruben.dibattista@skywarder.eu

Author: Francesco Colombi
Skyward Experimental Rocketry | CRD Dept | crd@skywarder.eu
email: francesco.colombi@skywarder.eu

Author: Adriano Filippo Inno
Skyward Experimental Rocketry | AFD Dept
email: adriano.filippo.inno@skywarder.eu
Revision date: 09/10/2019

%}

if settings.wind.model && settings.wind.input
    error('Both wind model and input wind are true, select just one of them')
end

if settings.wind.HourMin ~= settings.wind.HourMax || settings.wind.HourMin ~= settings.wind.HourMax
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

if settings.wind.input && all(settings.wind.input_uncertainty ~= 0)
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
if settings.wind.model || settings.wind.input   % will be computed inside the integrations
    uw = 0; vw = 0; ww = 0; uncert = [0,0];
else
    [uw, vw, ww, ~] = wind_const_generator(settings.wind);
    
    if ww ~= 0
        warning('Pay attention using vertical wind, there might be computational errors')
    end
    
end

tf = settings.ode.final_time;

%% ASCENT
% ascent phase computation
[Ta, Ya] = ode113(@ascent, [0, tf], Y0a, settings.ode.optionsasc1, settings, uw, vw, ww, uncert); % till the apogee

if settings.para(1).delay ~= 0 % checking if the actuation delay is different from zero
    [Ta2, Ya2] = ode113(@ascent, [Ta(end), Ta(end) + settings.para(1).delay], Ya(end,:), settings.ode.optionsasc2,...
        settings, uw, vw, ww, uncert); % till end of the delay
    
    Ta = [Ta; Ta2(2:end ) ];
    Ya = [Ya; Ya2(2:end, :)];
end

[data_ascent] = recallOdeFcn(@ascent, Ta, Ya, settings, uw, vw, ww, uncert);
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
        para = i;
        [Tp, Yp] = ode113(@descent_parachute, [t0p, tf], Y0p, settings.ode.optionspara,...
            settings, uw, vw, ww, para, uncert);

        [data_para{para}] = recallOdeFcn(@descent_parachute, Tp, Yp, settings, uw, vw, ww, para, uncert);
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
    
    [data_para, Tp, Yp, bound_value] = descent_parachute6dof(Ta, Ya, settings, uw, vw, ww, uncert);
    
    % total state
    Yf = [Yf; Yp];
    Tf = [Tf; Tp];
end
    
save('descent_para_plot.mat', 'data_para')

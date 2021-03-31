function [LP, X, ApoTime, data_ascent, data_para] = stoch_run(settings)
%{

STOCH_RUN - This function runs a stochastic simulation (parallel)

INPUTS:     - settings, rocket data structure.

OUTPUTS:
            - LP, Landing Points matrix;
            - X, Apogee Points matrix;
            - ApoTime, Apogee time vector;
            - data_ascent, cell matrix containing fligth data of the ascent phase;
            - data_para, cell matrix containing fligth data of the descent phase.

Author: Ruben Di Battista
Skyward Experimental Rocketry | CRD Dept | crd@skywarder.eu
email: ruben.dibattista@skywarder.eu
April 2014; Last revision: 29.V.2014

Author: Adriano Filippo Inno
Skyward Experimental Rocketry | AFD Dept
email: adriano.filippo.inno@skywarder.eu
Revision date: 09/10/2019

%}

warning off

if settings.wind.model && settings.wind.input
    error('Both wind model and input wind are true, select just one of them')
end

if settings.descent6DOF
    error('You can''t use descent 6 dof simulation for stochastic purpose');
end

if settings.OMEGAmin == settings.OMEGAmax && settings.PHImin == settings.PHImax...
        && settings.PHIsigma == 0
    if not(settings.wind.model) && not(settings.wind.input)
        
        if settings.wind.MagMin == settings.wind.MagMax && settings.wind.AzMin == settings.wind.AzMax
            error('In stochastic simulations the wind must setted with the random model, check config.m')
        end
        
    elseif settings.wind.model
        
        if settings.wind.DayMin == settings.wind.DayMax && settings.wind.HourMin == settings.wind.HourMax
            error('In stochastic simulations with the wind model the day or the hour of launch must vary, check config.m')
        end
        
    end
    
    if settings.wind.input && all(settings.wind.input_uncertainty == 0)
        error('In stochastic simulations the wind input model, the uncertainty must be different to 0 check config.m')
    end
end

if settings.para(settings.Npara).z_cut ~= 0
    error('The landing will be not achived, check the final altitude of the last parachute in config.m')
end

%% STARTING CONDITIONS
% State
X0 = [0 0 0]';
V0 = [0 0 0]';
W0 = [0 0 0]';

%PreAllocation
N = settings.stoch.N;
Np = settings.Npara;
LP = zeros(N, 3);
X = zeros(N, 3);
ApoTime = zeros(N, 1);
data_para = cell(N, Np);

tf = settings.ode.final_time;


%% STOCHASTIC INPUTS

if not(settings.wind.model)
    Day = zeros(N, 1);
    Hour = zeros(N, 1); 
    uw = zeros(N, 1); vw = uw; ww = uw; Azw = uw;
    if settings.wind.input
        signn = randi([1, 4]); % 4 sign cases
        unc = settings.wind.input_uncertainty;
        
        switch signn
            case 1
                % unc = unc;
            case 2
                unc(1) = - unc(1);
            case 3
                unc(2) = - unc(2);
            case 4
                unc = - unc;
        end
        
        uncert = rand(N, 2).*unc;
    else
        for i = 1:N
            [uw(i), vw(i), ww(i), Azw(i)] = wind_const_generator(settings.wind);
        end
        uncert = zeros(N, 2);
    end
    
else
    uw = zeros(N, 1); vw = uw; ww = uw;
    uncert = zeros(N, 2);
    Day = randi([settings.wind.DayMin, settings.wind.DayMax], N, 1);
    Hour = randi([settings.wind.HourMin, settings.wind.HourMax], N, 1);
end

OMEGA = settings.OMEGAmin + rand(N, 1)*(settings.OMEGAmax - settings.OMEGAmin);

if settings.wind.input || settings.wind.model
    PHI = settings.PHImin + rand(N, 1)*(settings.PHImax - settings.PHImin);
else
    
    if settings.upwind
        PHI = mod(Azw + pi, 2*pi);
        signn = randi([1, 2]); % 4 sign cases
        
        if signn == 1
            PHIsigma = settings.PHIsigma;
        else
            PHIsigma = -settings.PHIsigma;
        end
        
        PHI = PHI + PHIsigma*rand(N, 1);
    else
        PHI = settings.PHImin + rand(N, 1)*(settings.PHImax - settings.PHImin);
    end
end

%% PARFOR LOOP
parfor_progress(N);
parpool;
parfor i = 1:N
    settingsNew = settings;
    settingsNew.stoch.OMEGA = OMEGA(i); 
    settingsNew.stoch.uncert = uncert(i, :); 
    settingsNew.stoch.Day = Day(i);
    settingsNew.stoch.Hour = Hour(i);
    settingsNew.stoch.uw = uw(i);
    settingsNew.stoch.vw = vw(i);
    settingsNew.stoch.ww = ww(i);
    
    %%% ascent
    Q0 = angleToQuat(PHI(i), OMEGA(i), 0*pi/180)';
    Y0a = [X0; V0; W0; Q0; settings.Ixxf; settings.Iyyf; settings.Izzf];
    [Ta, Ya] = ode113(@ascent, [0, tf], Y0a, settings.ode.optionsasc1, settingsNew);
    
    if settings.para(1).delay ~= 0 % checking if the actuation delay is different from zero
        [Ta2,Ya2] = ode113(@ascent, [Ta(end), Ta(end) + settings.para(1).delay], Ya(end, :),...
            settings.ode.optionsasc2, settingsNew);
        Ta = [Ta; Ta2(2:end   )];
        Ya = [Ya; Ya2(2:end, :)];
    end
    
    [data_ascent{i}] = recallOdeFcn(@ascent, Ta, Ya, settingsNew);
    data_ascent{i}.state.Y = Ya;
    data_ascent{i}.state.T = Ta;
    
    %%% descent
    % Initial Condition are the last from ascent (need to rotate because
    % velocities are in body axes)
    Y0p = [Ya(end,1:3) quatrotate(quatconj(Ya(end,10:13)), Ya(end,4:6))];
    Yf = Ya(:, 1:6);
    Tf  = Ta;
    t0p = Ta(end);

    for k = 1:Np
        settingsNew.stoch.para = k;
        [Tp, Yp] = ode113(@descent_parachute, [t0p, tf], Y0p, settings.ode.optionspara, settingsNew);
        
        ParoutDesc = recallOdeFcn(@descent_parachute, Tp, Yp, settingsNew);
        ParoutDesc.state.Y = Yp;
        ParoutDesc.state.T = Tp;
        
        % total state
        Yf = [Yf; Yp];
        Tf = [Tf; Tp];
        
        % updating ODE starting conditions
        Y0p = Yp(end, :);
        t0p = Tp(end);
        data_para{i, k} = ParoutDesc;
        
    end
    
    LP(i, :) = Yf(end,1:3);
    X(i, :) = [Ya(end,1); Ya(end,2); -Ya(end,3)];
    ApoTime(i) = Ta(end);
    parfor_progress;

end


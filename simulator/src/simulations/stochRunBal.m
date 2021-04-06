function [LP, X, ApoTime, data_ascent, data_bal] = stochRunBal(settings)
%{

stochRunBal - This function runs a stochastic simulation (parallel)

INPUTS:     
            - settings, struct, stores data of the rocket and of the simulation.

OUTPUTS:
            - LP, array, [n° simulations, 3], Landing Points matrix;
            - X, array, [n° simulations, 3], Apogee Points matrix;
            - ApoTime, array,  [n° simulations, 1], Apogee time vector;
            - data_ascent, cell, (n°simulations, 1), cell matrix containing fligth data of the ascent phase;
            - data_para, cell, (n°simulations, n° of parachutes), cell matrix containing fligth data of the descent phase.

CALLED FUNCTIONS: windConstGenerator, parfor_progress, ascent, descentBallistic, recallOdeFcn.

REVISIONS:
- #0 29.05.2014, Release, Ruben Di Battista
- #1 09/10/2019, Revision, Adriano Filippo Inno

%}

warning off 

if settings.wind.model && settings.wind.input
    error('Both wind model and input wind are true, select just one of them')
end

if settings.descent6DOF
    error('You can''t use descent 6 dof simulation for stochastic purpose')
end

if settings.OMEGAmin == settings.OMEGAmax && settings.PHImin == settings.PHImax 
    if not(settings.wind.model) && not(settings.wind.input)
        
        if settings.wind.MagMin == settings.wind.MagMax && settings.wind.ElMin == settings.wind.ElMax
            error('In stochastic simulations the random model wind must be setted with stochastic input, such as the magnitude that has to vary, check config.m')
        end
        
    elseif settings.wind.model
        
        if settings.wind.DayMin == settings.wind.DayMax && settings.wind.HourMin == settings.wind.HourMax
            error('In stochastic simulations with the wind model the day or the hour of launch must vary, check config.m')
        end
        
    end
    
    if settings.wind.input && all(settings.wind.input_uncertainty == 0)
        error('In stochastic simulations the wind input model, the uncertainty must be different to 0, check config.m')
    end
    
end

%% STARTING CONDITIONS

% State
X0 = [0 0 0]';
V0 = [0 0 0]';
W0 = [0 0 0]';

%PreAllocation
N = settings.stoch.N;
LP = zeros(N, 3);
X = zeros(N, 3);
ApoTime = zeros(N, 1);

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
            [uw(i), vw(i), ww(i), Azw(i)] = windConstGenerator(settings.wind);
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
    
    Q0 = angleToQuat(PHI(i), OMEGA(i), 0*pi/180)';
    Y0a = [X0; V0; W0; Q0; settings.Ixxf; settings.Iyyf; settings.Izzf];
    [Ta,Ya] = ode113(@ascent, [0, tf], Y0a, settings.ode.optionsasc1, settingsNew);
    [data_ascent{i}] = recallOdeFcn(@ascent, Ta, Ya, settingsNew);
    data_ascent{i}.state.Y = Ya;
    data_ascent{i}.state.T = Ta;
    
    %% DESCEND
    [Tb, Yb] = ode113(@descentBallistic, [Ta(end), tf], Ya(end, 1:13), settings.ode.optionsdesc,...
        settingsNew);
    [data_bal{i}] = recallOdeFcn(@descentBallistic, Tb, Yb, settingsNew);
    data_bal{i}.state.Y = Yb;
    data_bal{i}.state.T = Tb;
    
    %% FINAL STATE ASSEMBLING
    %Total State
    LP(i, :) = Yb(end, 1:3);
   
    X(i, :) = [Ya(end, 1); Ya(end, 2); -Ya(end, 3)]
    ApoTime(i) = Ta(end);
    
    parfor_progress;

end


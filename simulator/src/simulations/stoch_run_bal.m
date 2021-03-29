function [LP, X, ApoTime, data_ascent, data_bal] = stoch_run_bal(settings)
%{

STOCH_RUN_BAL - This function runs a stochastic simulation (parallel)

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

% PreAllocation
LP = zeros(settings.stoch.N, 3);
X = zeros(settings.stoch.N, 3);
ApoTime = zeros(settings.stoch.N, 1);

tf = settings.ode.final_time;

%% PARFOR LOOP

parfor_progress(settings.stoch.N); % initiaize parfor loop
% parpool;

parfor i = 1:settings.stoch.N
    
    %% WIND GENERATION
    
    if not(settings.wind.model)
        Day = 0; Hour = 0;
        
        if settings.wind.input
            signn = randi([1, 4]); % 4 sign cases
            unc = settings.wind.input_uncertainty;
            
            switch signn
                case 1
                    %                       unc = unc;
                case 2
                    unc(1) = - unc(1);
                case 3
                    unc(2) = - unc(2);
                case 4
                    unc = - unc;
            end
            
            uncert = rand(1, 2).*unc;
            uw = 0; vw = 0; ww = 0;
        else
            [uw, vw, ww, Azw] = wind_const_generator(settings.wind);
            uncert = [0; 0];
        end
        
    else
        Day = randi([settings.wind.DayMin, settings.wind.DayMax]);
        Hour = randi([settings.wind.HourMin, settings.wind.HourMax]);
        uw = 0; vw = 0; ww = 0; uncert = [0; 0];
    end
    
    %% ASCENT
    % ascent phase computation
    OMEGA = settings.OMEGAmin + rand*(settings.OMEGAmax - settings.OMEGAmin);
    
    % Attitude
    if settings.wind.input || settings.wind.model
        PHI = settings.PHImin + rand*(settings.PHImax - settings.PHImin);
    else
        
        if settings.upwind
            PHI = mod(Azw + pi, 2*pi);
            signn = randi([1, 2]); % 4 sign cases
            
            if signn == 1
                
                PHIsigma = settings.PHIsigma;
                
            else
                
                PHIsigma = -settings.PHIsigma;
                
            end
            
            PHI = PHI + PHIsigma*rand;
        else
            PHI = settings.PHImin + rand*(settings.PHImax - settings.PHImin);
            
        end
    end
    
    Q0 = angleToQuat(PHI, OMEGA, 0*pi/180)';
    Y0a = [X0; V0; W0; Q0; settings.Ixxf; settings.Iyyf; settings.Izzf];
    [Ta,Ya] = ode113(@ascent, [0, tf], Y0a, settings.ode.optionsasc1, settings, uw, vw, ww, uncert, Hour, Day, OMEGA);
    [data_ascent{i}] = recallOdeFcn(@ascent, Ta, Ya, settings, uw, vw, ww, uncert, Hour, Day, OMEGA);
    data_ascent{i}.state.Y = Ya;
    data_ascent{i}.state.T = Ta;
    
    %% DESCEND
    [Tb, Yb] = ode113(@descent_ballistic, [Ta(end), tf], Ya(end, 1:13), settings.ode.optionsdesc,...
        settings, uw, vw, ww, uncert, Hour, Day);
    [data_bal{i}] = recallOdeFcn(@descent_ballistic, Tb, Yb, settings, uw, vw, ww, uncert, Hour, Day);
    data_bal{i}.state.Y = Yb;
    data_bal{i}.state.T = Tb;
    
    %% FINAL STATE ASSEMBLING
    %Total State
    LP(i, :) = Yb(end, 1:3);
   
    X(i, :) = [Ya(end, 1); Ya(end, 2); -Ya(end, 3)]
    ApoTime(i) = Ta(end);
    
    parfor_progress;

end


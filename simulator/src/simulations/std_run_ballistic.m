function [Tf, Yf, Ta, Ya, bound_value] = std_run_ballistic(settings)
%{ 

STD_RUN_BALLISTIC - This function runs a standard ballistic (non-stochastic) simulation

INTPUTS: 
            - settings, rocket data structure;

OUTPUTS:
            - Tf, Total integration time vector; 
            - Yf, Total State Matrix;
            - Ta, Ascent Integration time vector; 
            - Ya, Ascent State Matrix;
            - bound_value, Usefull values for the plots.

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

if settings.wind.HourMin ~= settings.wind.HourMax || settings.wind.DayMin ~= settings.wind.DayMax
    error('In standard simulations with the wind model the day and the hour of launch must be unique, check config.m')
end

if settings.OMEGAmin ~= settings.OMEGAmax || settings.PHImin ~= settings.PHImax 
    error('In a single simulation the launchpad configuration has to be unique, check config.m')
end

if settings.ballistic && settings.descent6DOF
    error('Both ballistic and descent6DOF are true, select just one of them')
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

% Attitude
Q0 = angleToQuat(settings.PHI, settings.OMEGA, 0*pi/180)';

% State
X0 = [0 0 0]';
V0 = [0 0 0]';
W0 = [0 0 0]';

Y0a = [X0; V0; W0; Q0; settings.Ixxf; settings.Iyyf; settings.Izzf];

%% WIND GENERATION
if not(settings.wind.model) && not(settings.wind.input)
    [uw, vw, ww, ~] = wind_const_generator(settings.wind);
    settings.constWind = [uw, vw, ww];
    if ww ~= 0
        warning('Pay attention using vertical wind, there might be computational errors')
    end
    
end

tf = settings.ode.final_time;

%% ASCENT
% ascent phase computation
[Ta, Ya] = ode113(@ascent, [0, tf], Y0a, settings.ode.optionsasc1, settings);
[data_ascent] = recallOdeFcn(@ascent, Ta, Ya, settings);
data_ascent.state.Y = Ya;
data_ascent.state.T = Ta;
save('ascent_plot.mat', 'data_ascent');

%% DESCEND 
% Initial Condition are the last from ascent
[Td, Yd] = ode113(@descent_ballistic, [Ta(end), tf], Ya(end, 1:13), settings.ode.optionsdesc, settings);
[data_bal] = recallOdeFcn(@descent_ballistic, Td, Yd, settings);
data_bal.state.Y = Yd;
data_bal.state.T = Td;
save('descent_plot.mat', 'data_bal');

Yf = [Ya(:, 1:13); Yd];
Tf = [Ta; Td];

%% TIME, POSITION AND VELOCITY AT APOGEE
% Usefull values for the plots
bound_value.t = Ta(end);
bound_value.X = [Ya(end, 2), Ya(end, 1), -Ya(end, 3)];
bound_value.V = quatrotate(quatconj(Ya(end, 10:13)), Ya(end, 4:6));


function [X, ApoTime, data_ascent] = sensitivityStochAscent(settings)
%{
sensitivityStochAscent - This function runs a stochastic sensitivity analysis
of the ascent phase with stochastic variations of the parameters.

INPUTS:
- settings, struct, stores data of the rocket and of the simulation.

OUTPUTS:
- X, array [3, n° simulations], Apogee coordinates in NED
- ApoTime, array,  [n° simulations, 1], times when the apogee is reached
- data_ascent, cell, (n° simulations, 1), useful data for the plots

CALLED FUNCTIONS: windConstGenerator, parfor_progress, ascent, recallOdeFcn.

REVISIONS:
- #0 22/12/2020, Release, Luca Facchini
- #1 05/04/2021, Revision, Giulio Pacifici
  Changes: Variation of more than one parameter. Normal distribution of
  uncertainty. Uncertainty on thrust.
%}

%% STARTING CONDITIONS (EQUAL FOR ALL)

% State
X0 = [0 0 0]';
V0 = [0 0 0]';
W0 = [0 0 0]';

tf = settings.ode.finalTime;

Az = settings.wind.Az;
El = settings.wind.El;
Magw = settings.wind.Mag;

% ascent phase computation
OMEGA = settings.OMEGA;
PHI = settings.PHI;

% WIND GENERATION
[uw, vw, ww, Azw] = windConstGenerator(settings.wind);
settings.constWind = [uw, vw, ww];

% Attitude
if settings.upwind
    PHI = mod(Azw + pi, 2*pi);
end

Q0 = angle2quat(PHI, OMEGA, 0*pi/180, 'ZYX')';
Y0a = [X0; V0; W0; Q0; settings.Ixxf; settings.Iyyf; settings.Izzf];


%% PREALLOCATION
N = settings.sensitivity.N;

ApoTime = zeros(N,1);
X = zeros(3,N);
data_ascent = cell(N,1);

%% PARFOR LOOP
stdThrust = settings.sensitivity.stdStoch(1);
stdCA = settings.sensitivity.stdStoch(2);
stdMs = settings.sensitivity.stdStoch(3);

pw = PoolWaitbar(N, 'Please wait... ');
if settings.parThreads
    parpool('threads');
else
    parpool;
end
parfor i = 1:N
    
    % Modify uncertain variables contained in settings:
    newSettings = settings;
    
    % Thrust:
    if strcmp("same",settings.sensitivity.thrustUncertainty)
        newSettings.motor.expThrust = settings.motor.expThrust*(1 + randn*stdThrust);
    elseif strcmp("independent",settings.sensitivity.thrustUncertainty)
        nExpThrust = length(settings.motor.expThrust);
        newSettings.motor.expThrust =...
            settings.motor.expThrust.*(1 + randn(1,nExpThrust)*stdThrust);
    end
    
    % CA:
    newSettings.CoeffsE.CA = settings.CoeffsE.CA*(1 + randn*stdCA);
    newSettings.CoeffsF.CA = settings.CoeffsF.CA*(1 + randn*stdCA);
    
    % Structural Mass:
    newSettings.ms = settings.ms*(1 + randn*stdMs);
    
    [Ta, Ya] = ode113(@ascent, [0, tf], Y0a, settings.ode.optionsasc1,...
                      newSettings); 
    
    X(:,i) = [Ya(end,1); Ya(end,2); -Ya(end,3)];
    ApoTime(i) = Ta(end);
    
    [data_ascent{i}] = recallOdeFcn(@ascent, Ta, Ya, newSettings);
    
    data_ascent{i}.state.Y = Ya;
    data_ascent{i}.state.T = Ta;
    
    increment(pw);
end

delete(gcp('nocreate'))
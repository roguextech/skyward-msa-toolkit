function [X, ApoTime, data_ascent] = sensitivityDetAscent(settings)
%{
sensitivityDetAscent - This function runs a simulation of the ascent phase 
with determistic variations of the parameters.

INPUTS:
- settings, struct, stores data of the rocket and of the simulation.

OUTPUTS:
- X, array [3, n° parameters, n° variations], Apogee coordinates in NED
- ApoTime, array,  [n° parameters, n° variations], times when the apogee is reached
- data_ascent, cell, (n° variations, n° parameters), useful data for the plots

CALLED FUNCTIONS: windConstGenerator, parfor_progress, ascent, recallOdeFcn.

REVISIONS:
- #0 22/12/2020, Release, Luca Facchini
%}

%% STARTING CONDITIONS (EQUAL FOR ALL)

% State
X0 = [0 0 0]';
V0 = [0 0 0]';
W0 = [0 0 0]';

tf = settings.ode.finalTime;

% ascent phase computation
OMEGA = settings.OMEGA;
PHI = settings.PHI;

% WIND GENERATION
[uw, vw, ww, Azw] = windConstGenerator(settings.wind);
settings.constWind = [uw; vw; ww];

% Attitude
if settings.upwind
    PHI = mod(Azw + pi, 2*pi);
end

Q0 = angle2quat(PHI, OMEGA, 0*pi/180, 'ZYX')';
Y0a = [X0; V0; W0; Q0; settings.Ixxf; settings.Iyyf; settings.Izzf];

%% PREALLOCATION
nDelta = length(settings.sensitivity.deltaDet);
nPara = length(settings.sensitivity.para);

% Preallocation
ApoTime = zeros(nDelta,nPara);
X = zeros(3,nDelta,nPara);
deltaDet = settings.sensitivity.deltaDet;
data_ascent = cell(nDelta,nPara);

%% PARFOR LOOP
coeff_names = settings.sensitivity.para;

pw = PoolWaitbar(nPara, 'Please wait... ');
if settings.parThreads
    parpool('threads');
else
    parpool;
end
parfor j=1:nPara
    COEFF_NAME = string(coeff_names{j});
    for i = 1:nDelta
        
        newSettings = settings;
        
        if strcmp(COEFF_NAME,'ms')
            newSettings.ms = settings.ms*(1 + deltaDet(i));
            
            % ASCENT
            [Ta, Ya] = ode113(@ascent, [0, tf], Y0a, settings.ode.optionsasc1,...
                              newSettings);
        else
            % Pertubation of the proper field
            newSettings.CoeffsF.(COEFF_NAME) = ...
                settings.CoeffsF.(COEFF_NAME).*(1 + deltaDet(i));
            newSettings.CoeffsE.(COEFF_NAME) = ...
                settings.CoeffsE.(COEFF_NAME).*(1 + deltaDet(i));
            
            % ASCENT
            [Ta, Ya] = ode113(@ascent, [0, tf], Y0a, settings.ode.optionsasc1,...
                              newSettings);
        end
        
        X(:,i,j) = [Ya(end,1); Ya(end,2); -Ya(end,3)];
        ApoTime(i,j) = Ta(end);
        
        [data_ascent{i,j}] = recallOdeFcn(@ascent, Ta, Ya, newSettings);
        data_ascent{i,j}.state.Y = Ya;
        data_ascent{i,j}.state.T = Ta;
    end
    increment(pw);
end


delete(gcp('nocreate'))

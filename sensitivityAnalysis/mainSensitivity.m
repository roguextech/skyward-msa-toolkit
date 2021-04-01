%{
mainSensitivity - main script for the sensitivity analysis.

CALLED FUNCTIONS: sensitivityStochAscent, sensitivityDetAscent.

REVISIONS:
- #0 22/12/2020, Release, Luca Facchini
%}

close all
clear 
clc

filePath = fileparts(mfilename('fullpath'));
currentPath = pwd;
if not(strcmp(filePath, currentPath))
    cd (filePath);
    currentPath = filePath;
end

addpath(genpath(currentPath));


%% LOAD DATA
dataPath = '../data/';
addpath(dataPath);
commonFunctionsPath = '../commonFunctions/';
addpath(genpath(commonFunctionsPath));
simulationsData
configSensitivity

%% CHECK SETTINGS
if settings.sensitivity.stoch == 1 && settings.sensitivity.N < 2
    error('Consider at least 2 stochastic simulation, N >= 2.');
end

%% START THE CHOSEN SIMULATION
% T = vector of time used by ODE, [s] also for Tf Ta
% Y = State = ( x y z | u v w | p q r | q0 q1 q2 q3 | )

if settings.sensitivity.stoch
    tic
    [X, ApoTime, data_ascent] = sensitivityStochAscent(settings);
    toc
else
    tic
    [X, ApoTime ,data_ascent] = sensitivityDetAscent(settings);
    toc
end

%% PLOTS
plots




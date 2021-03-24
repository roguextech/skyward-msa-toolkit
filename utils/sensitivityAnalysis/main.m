%{

START_SIMULATION - this is the main script; it runs the simulation that has been chosen in config.m

%}

close all
clear 
clc

path = genpath(pwd);
addpath(path);


%% LOAD DATA
config;

%% CHECK SETTINGS
if settings.sensitivity.stoch && length(settings.sensitivity.param) > 1
    error("Stochastic variation is possible for only 1 parameter. Please change 'stoch' to false or choose only 1 paramer in 'param'");
end
if settings.sensitivity.stoch == 1 && settings.sensitivity.N < 2
    error('Consider at least 2 stochastic simulation, N >= 2.');
end

%% START THE CHOSEN SIMULATION
% T = vector of time used by ODE, [s] also for Tf Ta
% Y = State = ( x y z | u v w | p q r | q0 q1 q2 q3 | )

if settings.sensitivity.stoch
    tic
    [X, ApoTime, data_ascent] = sensitivity_stoch_ascent(settings);
    toc
else
    tic
    [X, ApoTime ,data_ascent] = sensitivity_det_ascent(settings);
    toc
end

%% PLOTS
run('plots.m');




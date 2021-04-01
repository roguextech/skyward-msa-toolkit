%{

START_SIMULATION - this is the main script; it runs the simulation that has been chosen in config.m

Author: Francesco Colombi
Skyward Experimental Rocketry | CRD Dept | crd@skywarder.eu
email: francesco.colombi@skywarder.eu
Release date: 16/04/2016

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
configSimulator;
dataPath = '../data/';
addpath(dataPath);
commonFunctionsPath = '../commonFunctions/';
addpath(genpath(commonFunctionsPath));
simulationsData;

%% START THE CHOSEN SIMULATION
% T = vector of time used by ODE, [s] also for Tf Ta
% Y = State = ( x y z | u v w | p q r | q0 q1 q2 q3 | thetax thetay thetaz | ) also for Ya,Yf corresponding to T

if not(settings.ballistic)
    settings.Npara = length(settings.para);
else
    settings.Npara = 1;
end

tic
    
% Checking if stochastic or standard simulation needed
if settings.ballistic
    if settings.stoch.N > 1
        if settings.wind.model 
            fprintf('Stochastic Ballistic Simulation With Wind Model Started...\n\n');
        elseif settings.wind.input
            fprintf('Stochastic Ballistic Simulation With Input Wind Started...\n\n');
        else
            fprintf('Stochastic Ballistic Simulation With Random Wind Started...\n\n');
        end
        [LP, X, ApoTime, data_ascent, data_bal] = stochRunBal(settings);
    else
        fprintf('Standard Ballistic Simulation Started...\n\n');
        [Tf, Yf, Ta, Ya, bound_value] = stdRunBallistic(settings);
    end
else
    if settings.stoch.N > 1
        
        if settings.wind.model 
            fprintf('Stochastic Simulation With Wind Model Started...\n\n');
        elseif settings.wind.input
            fprintf('Stochastic Simulation With Input Wind Started...\n\n');
        else
            fprintf('Stochastic Simulation With Random Wind Started...\n\n');
        end
            [LP, X, ApoTime, data_ascent, data_para] = stochRun(settings);
    else
        fprintf('Standard Simulation Started...\n\n');
        [Tf, Yf, Ta, Ya, bound_value] = stdRun(settings);
    end
end

toc

%% DATA-PRINTING
if settings.stoch.N == 1 
    
    Na = length(Ya(:, 1));
    
    za = -Ya(:, 3);
    Xa = Ya(:, 1:3);
    Va = Ya(:, 4:6);
    
    x = Yf(:, 1);
    y = Yf(:, 2);    
    z = -Yf(:, 3); 
    
    load('ascent_plot.mat');
    Aa = data_ascent.accelerations.body_acc;
    M = data_ascent.interp.M;
    V_apo = norm(data_ascent.state.Y(end, 4:6) - ...
        data_ascent.wind.body_wind(1:3, end)');
    
    abs_X = vecnorm(Xa');
    abs_V = vecnorm(Va');
    abs_A = vecnorm(Aa);
    
    [max_dist, imax_dist] = max(abs_X);
    [max_v, imax_v] = max(abs_V);
    [max_a, imax_a] = max(abs_A);
    [max_M, imax_M] = max(M);
    
    iexit = find(abs_X <= settings.lrampa);  % checking where the missile is undocked from the hook of the launch pad
    iexit = iexit(end);

    [lat, lon, ~] = ned2geodetic(x, y, 0, settings.lat0, settings.lon0, 0, wgs84Ellipsoid);
   
    % DATA RECORD (display)
    disp(' ')
    disp('DATA RECORD:')
    fprintf('apogee reached: %g [m] \n', za(end));
        
    fprintf('time: %g [sec] \n\n', Ta(end))
    
    fprintf('max speed reached: %g [m/s] \n', max_v)
    fprintf('altitude: %g [m] \n', za(imax_v))
    fprintf('Mach: %g [-] \n', M(imax_v))
    fprintf('time: %g [sec] \n\n', Ta(imax_v))
    
    fprintf('max Mach reached: %g [-] \n', max_M)
    fprintf('altitude: %g [m] \n', za(imax_M))
    fprintf('velocity: %g [m/s] \n', abs_V(imax_M))
    fprintf('time: %g [sec] \n\n', Ta(imax_M))
    
    fprintf('max acceleration reached: %g [m/s2] = %g [g] \n', max_a, max_a/9.81)
    fprintf('velocity: %g [m/s] \n', abs_V(imax_a))
    fprintf('time: %g [sec] \n\n', Ta(imax_a))
    
    fprintf('run on launch pad: %g [m] \n', abs_X(iexit))
    fprintf('speed at launch pad exit: %g [m/s] \n', abs_V(iexit))
    fprintf('time: %g [sec] \n\n', Ta(iexit))
    
    fprintf('latitude of landing point: %10.8f [deg] \n', lat(end));
    fprintf('longitude of landing point: %10.8f [deg] \n\n', lon(end));

    fprintf('apogee velocity relative to wind: %g [m/s] \n', V_apo);

else      
    % Mean Apogee Time
    ApoTimem = mean(ApoTime);
    
    % Std. Deviation Apogee Time
    ApoTimestd = std(ApoTime);
    
    % Std. Deviation Altitude
    zstd = std(X(:,3));
    
    % Mean Apogee Points
    zapom = mean(X(:,3));
    
    xm = mean(LP(:, 1));
    ym = mean(LP(:, 2));
    text = ['Mean Landing Point:X:%3.3f m,  Y:%3.3f m\n',...
        'Mean Altitude: %3.3f m || STD: %3.3f m\n',...
        'Mean Apogee Time: %3.3f s || STD: %3.3f s\n'];
    fprintf(text, xm, ym, zapom, zstd, ApoTimem, ApoTimestd);
    
    if not(settings.ballistic)
        
        [p, flag, ind_Pin, ind_Pout, LPOP] = launchProb(settings, data_ascent, data_para, LP);
        LPin = LP(ind_Pin, :);
        LPout = LP(ind_Pout, :);
        LPOPin = LPOP(ind_Pin, :);
        LPOPout = LPOP(ind_Pout, :);
        
        fprintf('The launch probability is: %.1f %% \n\n', p);
    end
    
    delete(gcp('nocreate'))
    delete('parfor_progress.txt')
    
end

%% PLOTS
if settings.plots
    run('plots.m')
end

clearvars -except T data_ascent data_para data_bal flag LP
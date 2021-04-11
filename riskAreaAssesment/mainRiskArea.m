%{

mainRiskArea - this is the main script; it runs the simulation that has been chosen in config.m

CALLED SCRIPTS: simulationsData, configSimulator, plots.

CALLED FUNCTIONS: stochRunBal, stdRunBal, stochRun, stdRun, launchProb.

REVISIONS:
- #0 11/04/2021, Release, Adriano FIlippo Inno & Jacopo Carradori

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
simulationsData;
configRiskArea;

%% LAUNCHPAD DYNAMICS
%%% Initial State
X0pad = [0; 0; 0; 0];
%%% Attitude
Q0 = angleToQuat(settings.PHImin, settings.OMEGA, 0*pi/180)';

nAlpha = size(settings.CoeffsF.CA, 1);
nBeta = size(settings.CoeffsF.CA, 3);
meanCA = settings.CoeffsF.CA(ceil(nAlpha/2), 1, ceil(nBeta/2), 1, 1);
[Tpad, Ypad] = ode113(@launchPadFreeDyn, [0, 10], X0pad, settings.ode.optionspad,...
    settings, Q0, meanCA);

velExit = Ypad(end, 4);

inertialWind = [-settings.wind.MagMax, 0, 0];
bodyWind = quatrotate(Q0', inertialWind);
bodyVelocity = [Ypad(end, 4), 0, 0];
Vr = bodyVelocity - bodyWind;
ur = Vr(1); vr = Vr(2); wr = Vr(3);
alphaExitMax = round(atand(wr/ur), 1)*pi/180;

settings.OMEGAmin = settings.OMEGA - alphaExitMax;
settings.OMEGAmax = 90*pi/180;

%% StochRunBal
settings.ballistic = true;
[LP, ~, ~, ~, ~] = stochRunBal(settings);

figure('Name', 'Landing Points', 'NumberTitle','off')
[lat_LP, lon_LP, ~] = ned2geodetic(LP(:, 1), LP(:, 2), 0, settings.lat0, settings.lon0, 0, wgs84Ellipsoid);
geoplot(lat_LP, lon_LP, '.r','MarkerSize', 5);
hold on
geoplot(settings.lat0, settings.lon0,'ro', 'MarkerSize', 16, 'MarkerFacecolor', 'r');
geobasemap('satellite');
geolimits([settings.lat0-settings.limLat, settings.lat0+settings.limLat], [settings.lon0-settings.limLon settings.lon0+settings.limLon]);

title('Landing Points in ballistic');


% [h1, h2, h3] = landingConePlot(LP(:, 1), LP(:, 2), [178, 34, 34]/255, [205, 92, 92]/255, [139, 0, 0]/255);

delete(gcp('nocreate'))

%% stochRun completa
settings.ballistic = false;
settings.Npara = length(settings.para);
[LP, X, ApoTime, data_ascent, data_para] = stochRun(settings);
% [h4, h5, h6] = landingConePlot(LP(:, 1), LP(:, 2), [], [], []);

figure('Name', 'Landing Points', 'NumberTitle','off')
[lat_LP, lon_LP, ~] = ned2geodetic(LP(:, 1), LP(:, 2), 0, settings.lat0, settings.lon0, 0, wgs84Ellipsoid);
geoplot(lat_LP, lon_LP, '.b','MarkerSize', 5);
hold on
geoplot(settings.lat0, settings.lon0,'bo', 'MarkerSize', 16, 'MarkerFacecolor', 'b');
geobasemap('satellite');
geolimits([settings.lat0-settings.limLat, settings.lat0+settings.limLat], [settings.lon0-settings.limLon settings.lon0+settings.limLon]);

title('Landing Points with 2nd drouge');

delete('parfor_progress.txt')

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

%%% compute the LP in latitude and longitude
[latLP, lonLP, ~] = ned2geodetic(LP(:, 1), LP(:, 2), 0, settings.lat0, settings.lon0, 0, wgs84Ellipsoid);

%%% max LP 
radiusLP = vecnorm(LP');
[~, iMaxRadius] = max(radiusLP);

%%% geoplot
figure('Name', 'Landing Points - ballistic', 'NumberTitle','off')
launchpadHandle = geoplot(settings.lat0, settings.lon0,'ro', 'MarkerSize', 16, 'MarkerFacecolor', 'r');
hold on
LPhandle = geoplot(latLP, lonLP, '.r','MarkerSize', 5);
geobasemap('satellite');
geolimits([settings.lat0-settings.limLat, settings.lat0+settings.limLat], [settings.lon0-settings.limLon settings.lon0+settings.limLon]);

%%% adding circle to the max LP
[latCircle, lonCircle] = scircle2(settings.lat0, settings.lon0, latLP(iMaxRadius), lonLP(iMaxRadius), wgs84Ellipsoid);
circleHandle = geoplot(latCircle, lonCircle, 'r', 'LineWidth', 2);

legend([launchpadHandle, LPhandle, circleHandle], {'Launchpad', 'Landing Points', 'Red Area'});
title('Landing Points in ballistic');

delete(gcp('nocreate'))

%% stochRun completa
settings.ballistic = false;
settings.Npara = length(settings.para);
[LP, X, ApoTime, data_ascent, data_para] = stochRun(settings);

%%% compute the LP in latitude and longitude
[latLP, lonLP, ~] = ned2geodetic(LP(:, 1), LP(:, 2), 0, settings.lat0, settings.lon0, 0, wgs84Ellipsoid);

%%% max LP 
radiusLP = vecnorm(LP');
[~, iMaxRadius] = max(radiusLP);

%%% geoplot
figure('Name', 'Landing Points - nominal', 'NumberTitle','off')
launchpadHandle = geoplot(settings.lat0, settings.lon0,'bo', 'MarkerSize', 16, 'MarkerFacecolor', 'b');
hold on
LPhandle = geoplot(latLP, lonLP, '.b','MarkerSize', 5);
geobasemap('satellite');
geolimits([settings.lat0-settings.limLat, settings.lat0+settings.limLat], [settings.lon0-settings.limLon settings.lon0+settings.limLon]);

%%% adding circle to the max LP
[latCircle, lonCircle] = scircle2(settings.lat0, settings.lon0, latLP(iMaxRadius), lonLP(iMaxRadius), wgs84Ellipsoid);
circleHandle = geoplot(latCircle, lonCircle, 'b', 'LineWidth', 2);

legend([launchpadHandle, LPhandle, circleHandle], {'Launchpad', 'Landing Points', 'Blu Area'});
title('Landing Points with 2nd drouge');


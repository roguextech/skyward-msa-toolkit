%{

MAIN - this is the main script; it runs the simulation that has been 
       chosen in config.m. NOTE: To be able to be run, the script needs
       the aerodynamic coefficients matrix of the payload in this folder.
       You can change rocket parameters in simulationsData.m and payload
       ones in config.m.

CALLED SCRIPTS: simulationsData, config.

CALLED FUNCTIONS: ascent TCT.

REVISIONS:
- #0 27/06/2021, Release, Davide Rosato


%}

clear all
close all
clc

filePath = fileparts(mfilename('fullpath'));
currentPath = pwd;
if not(strcmp(filePath, currentPath))
    cd (filePath);
    currentPath = filePath;
end

addpath(genpath(currentPath));


%% LOAD DATA
config;
dataPath = '../../data/';
addpath(dataPath);
commonFunctionsPath = '../../commonFunctions/';
addpath(genpath(commonFunctionsPath));
simulationsData;


%% STARTING CONDITIONS
%%% Launchpad
settings.OMEGA = settings.OMEGAmin;
settings.PHI = settings.PHImin;

%%% Attitude
Q0 = angleToQuat(settings.PHI, settings.OMEGA, 0*pi/180)';

%%% State
X0 = [0 0 0]';
V0 = [0 0 0]';
W0 = [0 0 0]';

Y0a = [X0; V0; W0; Q0; settings.Ixxf; settings.Iyyf; settings.Izzf];

%% WIND GENERATION
if not(settings.wind.model) && not(settings.wind.input)
    [uw, vw, ww, ~] = windConstGenerator(settings.wind);
    settings.constWind = [uw, vw, ww];
    if ww ~= 0
        warning('Pay attention using vertical wind, there might be computational errors')
    end
    
end

tf = settings.ode.finalTime;


%% ASCENT
% ascent phase computation
[Ta, Ya] = ode113(@ascentTCT, [0, tf], Y0a, settings.ode.optionsasc1, settings); % till the apogee


[data_ascent] = recallOdeFcn(@ascentTCT, Ta, Ya, settings);
data_ascent.state.Y = Ya;
data_ascent.state.T = Ta;
save('ascent_plot.mat', 'data_ascent');

%% POST-PROCESS calculations
Mtot = vecnorm(data_ascent.vincolo.M(1:3,:));

MassTest = (Mtot)/(9.81*settings.xProva);

%% PLOT
figure;
plot(data_ascent.state.T, data_ascent.vincolo.T(1,:));
hold on; grid on;
plot(data_ascent.state.T, data_ascent.vincolo.T(2,:));
plot(data_ascent.state.T, data_ascent.vincolo.T(3,:));
title('REAZIONI ASSIALI E LONGITUDINALI (body frame)');
legend('Tx', 'Ty', 'Tz');
xlabel('t [s]'); ylabel('F [N]');

figure;
plot(data_ascent.state.T, data_ascent.vincolo.M(1,:));
hold on; grid on;
plot(data_ascent.state.T, data_ascent.vincolo.M(2,:));
plot(data_ascent.state.T, data_ascent.vincolo.M(3,:));
title('MOMENTO VINCOLARE (body frame)');
legend('Mx', 'My', 'Mz');
xlabel('t [s]'); ylabel('M [Nm]');

figure;
plot(data_ascent.state.T, Mtot);
hold on; grid on;
title('MOMENTO VINCOLARE MASSIMO (body frame)');
xlabel('t [s]'); ylabel('M [Nm]');

figure;
subplot(2,1,1)
plot(data_ascent.state.T, MassTest);
grid on;
title('MASSA PROVA');
xlabel('t [s]'); ylabel('M [kg]');

subplot(2,1,2)
for i = 1:length(data_ascent.state.T)
    if data_ascent.interp.alpha(i) < 0
        alphaP(i) = data_ascent.interp.alpha(i)*180/pi + data_ascent.interp.alpha(i)*180/pi*0.5;
    else
        alphaP(i) = data_ascent.interp.alpha(i)*180/pi + data_ascent.interp.alpha(i)*180/pi*0.5;
    end
end
plot(data_ascent.state.T, alphaP);
grid on; hold on;
plot(data_ascent.state.T, data_ascent.interp.alpha*180/pi);
legend('alpha Payload', 'alpha rocket')
title('ALPHA');
xlabel('t [s]'); ylabel('alpha [deg]')




% figure;
% plot(data_ascent.state.T, abs(data_ascent.ciao.Peso));
% grid on;
% xlabel('t [s]'); ylabel('peso [kg]')

clearvars -except settings data_ascent




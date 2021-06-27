clear all; clc; close all;

%% CONFIG
config;
x_prova = 0.3;

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
[Ta, Ya] = ode113(@ascent, [0, tf], Y0a, settings.ode.optionsasc1, settings); % till the apogee


[data_ascent] = recallOdeFcn(@ascent, Ta, Ya, settings);
data_ascent.state.Y = Ya;
data_ascent.state.T = Ta;
save('ascent_plot.mat', 'data_ascent');


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
Mtot = sqrt(data_ascent.vincolo.M(1,:).^2 + ...
    data_ascent.vincolo.M(2,:).^2 + ...
    data_ascent.vincolo.M(3,:).^2);
plot(data_ascent.state.T, Mtot);
hold on; grid on;
title('MOMENTO VINCOLARE MASSIMO (body frame)');
xlabel('t [s]'); ylabel('M [Nm]');

figure;
subplot(2,1,1)
MassProva = Mtot./(x_prova*9.81);
plot(data_ascent.state.T, MassProva);
grid on;
title('MASSA PROVA');
xlabel('t [s]'); ylabel('M [kg]');

subplot(2,1,2)
plot(data_ascent.state.T, data_ascent.interp.alpha*180/pi);
grid on;
title('ALPHA');
xlabel('t [s]'); ylabel('alpha [deg]')


figure;
plot(data_ascent.state.T, abs(data_ascent.ciao.Peso));
grid on;
xlabel('t [s]'); ylabel('peso [kg]')






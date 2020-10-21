function apogee = RunSim(settings)

% Author: Ruben Di Battista
% Skyward Experimental Rocketry | CRD Dept | crd@skywarder.eu
% email: ruben.dibattista@skywarder.eu
% Website: http://www.skywarder.eu
% April 2014; Last revision: 31.XII.2014
% License:  2-clause BSD

% Author: Francesco Colombi
% Skyward Experimental Rocketry | CRD Dept | crd@skywarder.eu
% email: francesco.colombi@skywarder.eu
% Release date: 16/04/2016

%% STARTING CONDITIONS

% Attitude
Q0 = angle2quat(settings.PHI, settings.OMEGA, 0*pi/180,'ZYX')';

%% WIND GENERATION

[uw, vw, ww, ~] = WindConstGenerator(settings.wind.Az, settings.wind.Mag);
tf = settings.ode.final_time;

%% ASCENT
X0 = [0 0 0]';
V0 = [0 0 0]';
W0 = [0 0 0]';
X0a = [X0; V0; W0; Q0; settings.m0; settings.Ixxf; settings.Iyyf; settings.Izzf];
[~,Ya] = ode113(@Ascent, [0, tf], X0a, settings.ode.optionsasc, settings, uw, vw, ww);

%% CALCULATE OUTPUT QUANTITIES

apogee = -Ya(end, 3);



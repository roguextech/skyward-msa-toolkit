function apogee = RunSim(settings)

%{

RunSim - This function tests the fins simulating the ascent

Author: Matteo Pozzoli
Skyward Experimental Rocketry | CRD Dept | crd@skywarder.eu
email: matteo.pozzoli@skywarder.eu
Website: http://www.skywarder.eu
Release date: 14/10/2019

Author: Adriano Filippo Inno
Skyward Experimental Rocketry | CRD Dept | crd@skywarder.eu
email: adriano.filippo.inno@skywarder.eu
Update date: 21/10/20

%}

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



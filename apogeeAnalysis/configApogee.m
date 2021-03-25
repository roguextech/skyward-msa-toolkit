%{

CONFIG - This script sets up all the parameters for the simulation
All the parameters are stored in the "settings" structure.

Author: Matteo Pozzoli
Skyward Experimental Rocketry | CRD Dept | crd@skywarder.eu
email: matteo.pozzoli@skywarder.eu
Release date: 23/11/2020

%}

%% SIMULATION SETUP
% Structural mass for the analysis
vars.msDeviation = 2;                           %[kg] ms = ms +- deviation
vars.nMass = 5;                                 %[/] number of mass points 

% Total impulse choice
vars.Itot_range = [9170 9190];                  %[Ns] toal impulse range for the analysis

% Acceleration plot
% set to false if you don't want the acceleration plot
settings.accelerationPlot = true;

%% LAUNCH SETUP
settings.OMEGA = 84*pi/180;               %[rad] Minimum Elevation Angle, user input in degrees (ex. 80)
settings.PHI = 0*pi/180;                  %[rad] Maximum Azimuth Angle from North Direction, user input in degrees (ex. 90)

%% UPWIND CASE
vars.wind.Mag(1) = 5;                           %[m/s] wind magnitude
vars.wind.Az(1) = 360*pi/180;                   %[rad] wind azimuth user input in degrees
vars.wind.El(1) = 0*pi/180;                     %[rad] wind elevation user input in degrees
% Aerobrakes height (1: closed, 2: 50% open, 3: fully open)
vars.control{1} = [1, 3];

% NOTE: wind aziumt angle indications (wind directed towards):
% 0 deg (use 360 instead of 0)  -> North
% 90 deg                        -> East
% 180 deg                       -> South
% 270 deg                       -> West

%% DOWNWIND CASE
vars.wind.Mag(2) = 9;                           %[m/s] wind magnitude
vars.wind.Az(2) = 180*pi/180;                   %[rad] wind azimuth user input in degrees
vars.wind.El(2) = 0*pi/180;                     %[rad] wind elevation user input in degrees

% Aerobrakes height (1: closed, 2: 50% open, 3: fully open)
vars.control{2} = [1];

% NOTE: wind aziumt angle indications (wind directed towards):
% 0 deg (use 360 instead of 0)  -> North
% 90 deg                        -> East
% 180 deg                       -> South
% 270 deg                       -> West

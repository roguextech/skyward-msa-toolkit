%{
CONFIG - This script sets up all the parameters for the simulation 
All the parameters are stored in the "settings" structure.

Author: Ruben Di Battista
Skyward Experimental Rocketry | CRD Dept | crd@skywarder.eu
email: ruben.dibattista@skywarder.eu
Website: http://www.skywarder.eu
License: 2-clause BSD

Author: Francesco Colombi
Skyward Experimental Rocketry | CRD Dept | crd@skywarder.eu
email: francesco.colombi@skywarder.eu

Author: Adriano Filippo Inno
Skyward Experimental Rocketry | CRD Dept | crd@skywarder.eu
email: adriano.filippo.inno@skywarder.eu
Release date: 18/10/2019

%}

%% LAUNCH SETUP
% launchpad 
settings.z0 = 0;                   %[m] Launchpad Altitude
settings.lrampa = 5.3;                %[m] LaunchPad route (launchpad length-distance from ground of the first hook)

% launchpad directions
% for a single run the maximum and the minimum value of the following
% angles must be the same.
settings.OMEGA = 90*pi/180;         %[rad] Minimum Elevation Angle, user input in degrees (ex. 80)       
settings.PHI = 0*pi/180;            %[rad] Maximum Azimuth Angle from North Direction, user input in degrees (ex. 90)

%% ENGINE DETAILS
%%%%%% Cesaroni 7545M1590-P
settings.motor.exp_time =   [0 0.004  0.019  0.063  0.153  0.182  0.247  0.616  1.028 2.111  2.551  2.635  2.796  3      3.349 3.541  3.5870];
settings.motor.exp_thrust = [0 556.85 1690.3 2359.2 2339.4 2570.1 2471.2 2497.6 2547  2316.4 2273.5 2253.8 1696.9 1472.9 247.1 108.73 0];

settings.mp = 3.457;                                                 % [kg]   Propellant Mass                                                
settings.mnc = 0.500;                                               % [kg]   Nosecone Mass
settings.tb = settings.motor.exp_time(end);                         % [s]    Burning time
settings.mfr = settings.mp/settings.tb;                             % [kg/s] Mass Flow Rate
settings.m0 = 20;                                                   % [kg]   Total Mass 
settings.ms = settings.m0 - settings.mp;                            % [kg]   Structural Mass


%% GEOMETRY DETAILS
% This parameters should be the same parameters set up in MISSILE DATCOM
% simulation.

settings.C = 0.16;                          % [m]      Caliber (Fuselage Diameter)
settings.S = pi*settings.C^2/4;             % [m^2]    Cross-sectional Surface

%% MASS GEOMERTY DETAILS
% x-axis: along the fuselage
% y-axis: right wing
% z-axis: downward

% inertias for full configuration (with all the propellant embarqued) obtained with CAD's
settings.Ixxf = 0.008795446;                    % [kg*m^2] Inertia to x-axis
settings.Iyyf = 2.050393979;                    % [kg*m^2] Inertia to y-axis
settings.Izzf = 2.050413838;                    % [kg*m^2] Inertia to z-axis

% inertias for empty configuration (all the propellant consumed) obtained with CAD's
settings.Ixxe = 0.008472446;                    % [kg*m^2] Inertia to x-axis
settings.Iyye = 1.712284592;                    % [kg*m^2] Inertia to y-axis
settings.Izze = 1.712304085;                    % [kg*m^2] Inertia to z-axis

%% INTEGRATION OPTIONS
settings.ode.final_time =  2000;                                                % [s] Final integration time
settings.ode.optionsasc = odeset('Events', @event_apogee,'InitialStep',1);      %ODE options for ascend
settings.ode.optionspad = odeset('Events', @event_pad);                         %ODE options for ascend

%% Random wind model
% Wind is generated randomly from the minimum to the maximum parameters which defines the wind.
% Setting the same values for min and max will fix the parameters of the wind.
settings.wind.Mag = 10;                   % [m/s] Magnitude
settings.wind.Az = (180)*pi/180;          % [rad] Azimuth, user input in degrees (ex. 90)

% NOTE: wind aziumt angle indications (wind directed towards):
% 0 deg (use 360 instead of 0)  -> North
% 90 deg                        -> East
% 180 deg                       -> South
% 270 deg                       -> West

%% Optimization Choice
settings.cal_min = 1;                    % minum stability margin required

%% XCP plot
settings.plot = true;

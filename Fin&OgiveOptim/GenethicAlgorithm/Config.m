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
settings.z0 = 109;                   %[m] Launchpad Altitude
settings.lrampa = 4.9;               %[m] LaunchPad route (launchpad length-distance from ground of the first hook)
settings.lat0 = 39.201778;                                                          % Launchpad latitude
settings.lon0 = -8.138368;                                                          % Launchpad longitude

% Gravity costant at launch latitude and altitude:
settings.g0 = gravitywgs84(settings.z0, settings.lat0);
% launchpad directions
% for a single run the maximum and the minimum value of the following
% angles must be the same.
settings.OMEGA = 84*pi/180;         %[rad] Minimum Elevation Angle, user input in degrees (ex. 80)       
settings.PHI = 0*pi/180;            %[rad] Maximum Azimuth Angle from North Direction, user input in degrees (ex. 90)

%% ENGINE DETAILS
% load motors data 
Motors = load('Motors.mat');
Motors = [Motors.Cesaroni Motors.Aerotech];

name = 'M2020';
% name = 'M1890';
% name = 'M1800';
% name = 'M2000R';

n_name = [Motors.MotorName] == name;
settings.motor.exp_time = Motors(n_name).t;
settings.motor.exp_thrust = Motors(n_name).T;
settings.motor.exp_m = Motors(n_name).m;
settings.mp = Motors(n_name).mp;         % [kg]   Propellant Mass                                                
settings.tb = Motors(n_name).t(end) ;    % [s]    Burning time
mm = Motors(n_name).mm;                  % [kg]   Total Mass of the Motor 
settings.ms = 18.5 + mm - settings.mp;   % [kg]   Structural Mass
settings.m0 = settings.ms + settings.mp; % [kg]   Total Mass
settings.mnc = 0.400;                    % [kg]   Nosecone Mass

clear ('Motors','name')


%% GEOMETRY DETAILS
% This parameters should be the same parameters set up in MISSILE DATCOM
% simulation.

settings.C = 0.15;                          % [m]      Caliber (Fuselage Diameter)
settings.S = pi*settings.C^2/4;             % [m^2]    Cross-sectional Surface

%% MASS GEOMERTY DETAILS
% x-axis: along the fuselage
% y-axis: right wing
% z-axis: downward

% inertias for full configuration (with all the propellant embarqued) obtained with CAD's
settings.Ixxf = 0.0540;                     % [kg*m^2] Inertia to x-axis
settings.Iyyf = 13.7274;                    % [kg*m^2] Inertia to y-axis
settings.Izzf = 13.7302;                    % [kg*m^2] Inertia to z-axis

% inertias for empty configuration (all the propellant consumed) obtained with CAD's
settings.Ixxe = 0.0498;                     % [kg*m^2] Inertia to x-axis
settings.Iyye = 11.5612;                    % [kg*m^2] Inertia to y-axis
settings.Izze = 11.5640;                    % [kg*m^2] Inertia to z-axis

%% INTEGRATION OPTIONS
settings.ode.final_time =  2000;                                                % [s] Final integration time
settings.ode.optionsasc = odeset('Events', @EventApogee,'InitialStep',1);      %ODE options for ascend
settings.ode.optionspad = odeset('Events', @EventPad);                         %ODE options for ascend

%% wind
settings.wind.Mag = 10;                   % [m/s] Magnitude

%% Optimization Choice
settings.cal_min = 1.5;                    % minum stability margin required

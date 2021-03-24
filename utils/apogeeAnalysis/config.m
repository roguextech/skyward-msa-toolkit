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
vars.ms = linspace(15,19,5);                   %[kg] rocket structural mass for the analysis

% Total impulse choice
vars.Itot_range = [9170 9190];                  %[Ns] toal impulse range for the analysis

% Acceleration plot
% set to false if you don't want the acceleration plot
settings.flag_a = true;


%% LAUNCH SETUP
% launchpad
settings.z0 = 109;                        %[m] Launchpad Altitude
lpin = 1.150;                             %[m] Distance from base of second pin
settings.lrampa = 5.9 - lpin;             %[m] LaunchPad route (total available route)

% Portugal coordinates
settings.lat0 = 39.201778;                % Launchpad latitude
settings.lon0 = -8.138368;                % Launchpad longitude

% Roccaraso coordinates
% settings.lat0 = 41.810093;              % Launchpad latitude
% settings.lon0 = 14.052546;              % Launchpad longitude

settings.g0 = gravitywgs84(settings.z0, settings.lat0);      % Gravity costant at launch latitude and altitude

% Launchpad directions
% for a single run the maximum and the minimum value of the following
% angles must be the same.
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


%% ENGINE DETAILS
% load motors data 
DATA_PATH = '../../data/';
filename = strcat(DATA_PATH,'Motors.mat');
Motors = load(filename);
motors = [Motors.Cesaroni Motors.Aerotech];

% save in settings the acceptable motors 
j = 1;
for i=1:size(motors,2)
    
    if motors(i).Itot > vars.Itot_range(1) && motors(i).Itot < vars.Itot_range(2)
        settings.motors(j) = motors(i);
        j = j + 1;
    end
  
end
clear('motors' , 'i' , 'j')


%% GEOMETRY DETAILS
% This parameters should be the same parameters set up in MISSILE DATCOM
% simulation.
settings.C = 0.150;                       % [m]      Caliber (Fuselage Diameter)
settings.S = pi*(settings.C/2)^2;         % [m^2]    Cross-sectional Surface


%% MASS GEOMERTY DETAILS
% x-axis: along the fuselage
% y-axis: right wing
% z-axis: downward

% inertias for full configuration (with all the propellant embarqued) obtained with CAD's
settings.Ixxf = 0.08;                     % [kg*m^2] Inertia to x-axis
settings.Iyyf = 13.05;                    % [kg*m^2] Inertia to y-axis
settings.Izzf = 13.05;                    % [kg*m^2] Inertia to z-axis

% inertias for empty configuration (all the propellant consumed) obtained with CAD's
settings.Ixxe = 0.07;                     % [kg*m^2] Inertia to x-axis
settings.Iyye = 10.06;                    % [kg*m^2] Inertia to y-axis
settings.Izze = 10.06;                    % [kg*m^2] Inertia to z-axis


%% AERODYNAMICS DETAILS
% These coefficients are obtained using MISSILE DATCOM
% (after parsing with the tool datcom_parser.py)
% The files are stored in the ../data folder with the following rule:
% rocket_name_full.mat | rocket_name_empty.mat
% e.g. R2a_full.mat    | R2a_empty.mat
% Relative Path of the data files (default: ../data/). Remember the trailing slash!!

% Coeffs is a 4D matrix given by Datcom that contains the aerodynamics
% coefficient computed for the input parameters (AoA,Betas,Altitudes,Machs)
% Note: All the parameters (AoA,Betas,Altitudes,Machs) must be the same for
% empty and full configuration

 DATA_PATH = '../../data/';

 % Coefficients in full configuration
 filename_full = strcat(DATA_PATH,'full.mat');
 CoeffsF = load(filename_full,'Coeffs');
 settings.CoeffsF = CoeffsF.Coeffs;
 clear('CoeffsF');
 
 % Coefficients in empty configuration
 filename_empty = strcat(DATA_PATH,'empty.mat');
 CoeffsE = load(filename_empty,'Coeffs');
 settings.CoeffsE = CoeffsE.Coeffs;
 clear('CoeffsE');
 
 
 s = load(filename_full,'State');
 settings.Alphas = s.State.Alphas';
 settings.Betas = s.State.Betas';
 settings.Altitudes = s.State.Altitudes';
 settings.Machs = s.State.Machs';
 clear('s');
 

%% INTEGRATION OPTIONS
settings.ode.final_time =  2000;          % [s] Final integration time

% create an option structure for the integrations:

% - AbsTol is the threshold below which the value of the solution becomes unimportant
% - RelTol is the tolerance betweeen two consecutive values
% - Events is the event function that defines when the integration must be
% - stopped (it has to be created)
% - InitialStep is the highest value tried by the solver

settings.ode.optionsasc = odeset('AbsTol',1E-3,'RelTol',1E-3,...
    'Events',@event_apogee,'InitialStep',1);    %ODE options for ascend

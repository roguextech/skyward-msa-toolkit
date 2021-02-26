%{

CONFIG - This script sets up all the parameters for the simulation
All the parameters are stored in the "settings" structure.


%}

%% LAUNCH SETUP
% launchpad pont the sor
settings.z0 = 109;                                                                  %[m] Launchpad Altitude
settings.lrampa = 4.9;                                                              %[m] LaunchPad route (distance from ground of the first hook)
settings.lat0 = 39.201778;                                                          % Launchpad latitude
settings.lon0 = -8.138368;                                                          % Launchpad longitude

% Gravity costant at launch latitude and altitude:
settings.g0 = gravitywgs84(settings.z0,settings.lat0);

% launchpad roccaraso
% settings.z0 = 1416;                                                                 %[m] Launchpad Altitude
% settings.lrampa = 4.9;                                                              %[m] LaunchPad route (distance from ground of the first hook)
% settings.lat0 = 41.810093;                                                          % Launchpad latitude
% settings.lon0 = 14.052546;                                                          % Launchpad longitude

% launchpad directions
% for a single run the maximum and the minimum value of the following
% angles must be the same.
settings.OMEGA = 84*pi/180; % [rad] Elevation Angle, user input in degrees (ex. 80)
settings.PHI = 0*pi/180;    % [rad] Azimuth Angle from North Direction, user input in degrees (ex. 90)
settings.upwind = false;       % If true, phi is selected according to wind direction (constant wind model only)

%% ENGINE DETAILS
% load motors data 
DATA_PATH = '../../data/';
filename = strcat(DATA_PATH,'Motors.mat');
Motors = load(filename);
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

settings.C = 0.15;                 % [m]      Caliber (Fuselage Diameter)
settings.S = pi*settings.C^2/4;    % [m^2]    Cross-sectional Surface

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

% DATA_PATH = '../data/';

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
settings.ode.final_time =  2000;                                    % [s] Final integration time

% create an option structure for the integrations:

% - AbsTol is the threshold below which the value of the solution becomes unimportant
% - RelTol is the tolerance betweeen two consecutive values
% - Events is the event function that defines when the integration must be
% - stopped (it has to be created)
% - InitialStep is the highest value tried by the solver

settings.ode.optionsasc1 = odeset('Events', @event_apogee, 'InitialStep', 1);       %ODE options for ascend
settings.ode.optionsasc2 = odeset('InitialStep', 1);                                %ODE options for due to the opening delay of the parachute

%% WIND DETAILS

%%%%% Random wind model
% Consider only constant wind, specified by the input
% Setting the same values for min and max will fix the parameters of the wind.
settings.wind.Mag = 6;       % [m/s] Magnitude
settings.wind.El = 0*pi/180; % [rad] Elevation, user input in degrees (ex. 90)
settings.wind.Az = (180)*pi/180; % [rad] Azimuth, user input in degrees (ex. 90)

% NOTE: wind azimuth angle indications (wind directed towards):
% 0 deg (use 360 instead of 0)  -> North
% 90 deg                        -> East
% 180 deg                       -> South
% 270 deg                       -> West

%% PARAMETER SENSITIVITY DETAILS
% The analysis works in two possible ways:
% Option 1) sensitivity ON MULTIPLE PARAMETERS considering DETERMINISTIC 
%   variantions relative w.r.t. the nominal value, equal for all flight
%   conditions (alpha/beta/mach/alt).
% Option 2) sensitivity ON A SINGLE PARAMETER considering a STOCHASTIC 
%   variantions relative w.r.t. the nominal value.  
%   The random variations are different for all flight conditions and for
%   all stochastic simulations, with uniform distribution

% Define the parameters to study as a cell array of strings.
% Possible values are all the aerodynamics coefficients as well as 
% the structural mass of the rocket ('ms')
% E.g: 
%   settings.sensitivity.param = {'CA' 'CN' 'CY' 'CM' 'CL' 'CYB' 'CNA' 'ms'};
settings.sensitivity.param = {'CA'};

% Determines which simulation is run
settings.sensitivity.stoch = 1;  
% - false/0: consider deterministic paramters variations  (option 1)
% - true/1: consider random variations (option 2)

%%% Option 1 :
% Relative variations of the coefficients, as a vector. Equal for all
% coefficeint, the length does not have to match the number of parameters 
settings.sensitivity.MeanCoeffVarPerc = [-0.15 -0.1 -0.05 0 0.05 0.1 0.15];

%%% Option 2
% Number of simulations
settings.sensitivity.N = 200;
% Maximum random variation of the coefficient wrt mean/nominal value
settings.sensitivity.MaxRelVariation = 0.1;



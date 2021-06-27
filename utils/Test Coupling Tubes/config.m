%{

CONFIG - This script sets up all the parameters for the simulation
All the parameters are stored in the "settings" structure.

REVISIONS:
- #0 27/06/2021, Release, Davide Rosato
%}

%% TEST SETUP
% Choose the distance where the test mass will be displaced
settings.xProva = 0.3;                                      % [m]

%% PAYLOAD GEOMETRY
% Payload geometry variables needed
settings.mPay = 5.5;                                        % [kg] Payload mass
settings.xcgPay = 0.4;                                      % [m] Payload xcg from nozzle tip
settings.Lpay = 0.6;                                        % [m] Payload length
settings.IxxP = 0.0619;                                     % [kg*m^2] Payload inertia to x-axis
settings.IyyP = 0.1959;                                     % [kg*m^2] Payload inertia to y-axis
settings.IzzP = 0.1959;                                     % [kg*m^2] Payload inertia to z-axis

%% PAYLOAD AERODYNAMICS
% Coefficients in empty configuration
CoeffsP = load('payloadAero.mat', 'Coeffs');
settings.CoeffsP = CoeffsP.Coeffs;
clear('CoeffsP');

%% LAUNCH SETUP
% launchpad directions
% for a single run the maximum and the minimum value of the following angles must be the same.
settings.OMEGAmin = 84*pi/180;                              % [rad] Minimum Elevation Angle, user input in degrees (ex. 80)
settings.OMEGAmax = 84*pi/180;                              % [rad] Maximum Elevation Angle, user input in degrees (ex. 80)
settings.PHImin = 0*pi/180;                                 % [rad] Minimum Azimuth Angle from North Direction, user input in degrees (ex. 90)
settings.PHImax = 0*pi/180;                                 % [rad] Maximum Azimuth Angle from North Direction, user input in degrees (ex. 90)

% !! ATTENTION !! The following 2 work just for the stochastichs simulations with constant wind model only
settings.upwind = false;                                    % If true, phi is selected opposite to the wind direction
settings.PHIsigma = 0*pi/180;                               % [deg] If upwind is true, you can select a variance for the direction

%% AEROBRAKES SETTINGS
settings.control = 1;                                       % aerobrakes, 1-2-3 for 0%, 50% or 100% opened

%% WIND DETAILS
% select which model you want to use:

%%%%% Matlab Wind Model
settings.wind.model = false;
% matlab hswm model, wind model on altitude based on historical data

% input Day and Hour as arrays to run stochastic simulations
settings.wind.DayMin = 105;                                 % [d] Minimum Day of the launch
settings.wind.DayMax = 105;                                 % [d] Maximum Day of the launch
settings.wind.HourMin = 4;                                  % [h] Minimum Hour of the day
settings.wind.HourMax = 4;                                  % [h] Maximum Hour of the day
settings.wind.ww = 0;                                       % [m/s] Vertical wind speed

%%%%% Input wind
settings.wind.input = false;
% Wind is generated for every altitude interpolating with the coefficient defined below

settings.wind.inputGround = 9;                            % wind magnitude at the ground [m/s]
settings.wind.inputAlt = [0 100 600 750 900 1500 4000];    % altitude vector [m]
settings.wind.inputMult = [0 0 10 15 20 30 30];            % percentage of increasing magnitude at each altitude
settings.wind.inputAzimut = [30 30 30 30 30 30 30];        % wind azimut angle at each altitude (toward wind incoming direction) [deg]

settings.wind.inputUncertainty = [0, 0];
% settings.wind.inputUncertainty = [a,b];      wind uncertanties:
% - a, wind magnitude percentage uncertanty: magn = magn *(1 +- a)
% - b, wind direction band uncertanty: dir = dir 1 +- b

%%%%% Random wind model
% if both the model above are false

% Wind is generated randomly from the minimum to the maximum parameters which defines the wind.
% Setting the same values for min and max will fix the parameters of the wind.
settings.wind.MagMin = 10;                                   % [m/s] Minimum Magnitude
settings.wind.MagMax = 10;                                   % [m/s] Maximum Magnitude
settings.wind.ElMin = 0*pi/180;                             % [rad] Minimum Elevation, user input in degrees (ex. 0)
settings.wind.ElMax = 0*pi/180;                             % [rad] Maximum Elevation, user input in degrees (ex. 0) (Max == 90 Deg)
settings.wind.AzMin = (180)*pi/180;                         % [rad] Minimum Azimuth, user input in degrees (ex. 90)
settings.wind.AzMax = (180)*pi/180;                         % [rad] Maximum Azimuth, user input in degrees (ex. 90)

% NOTE: wind azimuth angle indications (wind directed towards):
% 0 deg (use 360 instead of 0)  -> North
% 90 deg                        -> East
% 180 deg                       -> South
% 270 deg                       -> West

%{

CONFIG - This script sets up all the parameters for the simulation
All the parameters are stored in the "settings" structure.

Author: Francesco Colombi
Skyward Experimental Rocketry | CRD Dept | crd@skywarder.eu
email: francesco.colombi@skywarder.eu
Release date: 16/04/2016

%}

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

%% ROCCARASO TERRAIN
% !! ATTENTION !!  the following works only at Roccaraso
settings.terrain = false; 
if settings.terrain  
     settings.funZ = funZ_gen('zdata.mat', settings.lat0, settings.lon0, true, 'xy');
end

%% AEROBRAKES SETTINGS
settings.control = '0%';                                    % aerobrakes 0%, 50% or 100% opened

%% DESCENT PHASE MODEL
settings.descent6DOF = false;                               % set to true in order to start a 6DOF parachute descent phase

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

settings.wind.input_ground = 7;                             % wind magnitude at the ground [m/s]
settings.wind.input_alt = [0 100 600 750 900 1500 4000];    % altitude vector [m]
settings.wind.input_mult = [0 0 10 15 20 30 30];            % percentage of increasing magnitude at each altitude
settings.wind.input_azimut = [30 30 30 30 30 30 30];        % wind azimut angle at each altitude (toward wind incoming direction) [deg]


settings.wind.input_uncertainty = [1, 1];
% settings.wind.input_uncertainty = [a,b];      wind uncertanties:
% - a, wind magnitude percentage uncertanty: magn = magn *(1 +- a)
% - b, wind direction band uncertanty: dir = dir 1 +- b

%%%%% Random wind model
% if both the model above are false

% Wind is generated randomly from the minimum to the maximum parameters which defines the wind.
% Setting the same values for min and max will fix the parameters of the wind.
settings.wind.MagMin = 1;                                   % [m/s] Minimum Magnitude
settings.wind.MagMax = 1;                                   % [m/s] Maximum Magnitude
settings.wind.ElMin = 0*pi/180;                             % [rad] Minimum Elevation, user input in degrees (ex. 0)
settings.wind.ElMax = 0*pi/180;                             % [rad] Maximum Elevation, user input in degrees (ex. 0) (Max == 90 Deg)
settings.wind.AzMin = (360)*pi/180;                         % [rad] Minimum Azimuth, user input in degrees (ex. 90)
settings.wind.AzMax = (360)*pi/180;                         % [rad] Maximum Azimuth, user input in degrees (ex. 90)

% NOTE: wind azimuth angle indications (wind directed towards):
% 0 deg (use 360 instead of 0)  -> North
% 90 deg                        -> East
% 180 deg                       -> South
% 270 deg                       -> West

%% BALLISTIC SIMULATION
% Set to True to run a ballistic (without drogues) simulation
settings.ballistic = false;

%% STOCHASTIC DETAILS
% If N > 1 the stochastic routine is started
settings.stoch.N = 1;                                       % Number of cases

%% PLOT DETAILS
settings.plots = true;

%% LANDING POINTS
% satellite maps of the landing zone 
settings.landing_map = false;                               % 2D map
settings.satellite3D = false;                               % 3D map

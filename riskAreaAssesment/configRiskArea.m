%{

CONFIG - This script sets up all the parameters for the simulation
All the parameters are stored in the "settings" structure.

REVISIONS:
- #0 11/04/2021, Release, Adriano FIlippo Inno & Jacopo Carradori

%}

%% LAUNCH SETUP
% launchpad directions
% for a single run the maximum and the minimum value of the following angles must be the same.
settings.OMEGA = 85*pi/180;         % [rad] Nominal Elevation Angle, user input in degrees (ex. 80)
settings.PHImin = 0*pi/180;         % [rad] Minimum Azimuth Angle from North Direction, user input in degrees (ex. 90)
settings.PHImax = 360*pi/180;       % [rad] Maximum Azimuth Angle from North Direction, user input in degrees (ex. 90)

% !! ATTENTION !! The following 2 work just for the stochastichs simulations with constant wind model only
settings.upwind = true;             % If true, phi is selected opposite to the wind direction
settings.PHIsigma = 20*pi/180;      % [deg] If upwind is true, you can select a variance for the direction

%% ROCCARASO TERRAIN
% !! ATTENTION !!  the following works only at Roccaraso
settings.terrain = true; 
if settings.terrain  
     settings.funZ = funZGen('zdata.mat', settings.lat0, settings.lon0, true, 'xy');
end

%% WIND DETAILS
% select which model you want to use:

%%%%% Random wind model
% if both the model above are false

% Wind is generated randomly from the minimum to the maximum parameters which defines the wind.
% Setting the same values for min and max will fix the parameters of the wind.
settings.wind.MagMin = 0;           % [m/s] Minimum Magnitude
settings.wind.MagMax = 10;          % [m/s] Maximum Magnitude
settings.wind.ElMin = 0*pi/180;     % [rad] Minimum Elevation, user input in degrees (ex. 0)
settings.wind.ElMax = 0*pi/180;     % [rad] Maximum Elevation, user input in degrees (ex. 0) (Max == 90 Deg)
settings.wind.AzMin = (0)*pi/180;   % [rad] Minimum Azimuth, user input in degrees (ex. 90)
settings.wind.AzMax = (360)*pi/180; % [rad] Maximum Azimuth, user input in degrees (ex. 90)

% NOTE: wind azimuth angle indications (wind directed towards):
% 0 deg (use 360 instead of 0)  -> North
% 90 deg                        -> East
% 180 deg                       -> South
% 270 deg                       -> West

%% STOCHASTIC DETAILS
% If N > 1 the stochastic routine is started
settings.stoch.N = 200;            % Number of cases

%% LANDING POINTS
% satellite maps of the landing zone 
settings.landingMap = false;       % 2D map
settings.satellite3D = false;       % 3D map

%% COMPATIBILITY SETTINGS
% this settings are needed to work with the commonFunctions folder, do not
% modify it unless you now what you're doing                                   
settings.control = 1;              
settings.wind.model = false;
settings.wind.input = false;
settings.descent6DOF = false;

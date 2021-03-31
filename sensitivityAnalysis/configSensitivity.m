%{

CONFIG - This script sets up all the parameters for the simulation
All the parameters are stored in the "settings" structure.


%}

%% LAUNCH SETUP
% launchpad directions
% for a single run the maximum and the minimum value of the following
% angles must be the same.
settings.OMEGA = 84*pi/180; % [rad] Elevation Angle, user input in degrees (ex. 80)
settings.PHI = 0*pi/180;    % [rad] Azimuth Angle from North Direction, user input in degrees (ex. 90)
settings.upwind = false;    % If true, phi is selected according to wind direction (constant wind model only)

%% WIND DETAILS
%%%%% Matlab Wind Model
settings.wind.model = false;

%%%%% Input wind
settings.wind.input = false;

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

%% PARAMETERS FOR SENSITIVITY ANALYSIS
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
%   settings.sensitivity.para = {'CA' 'CN' 'CY' 'CM' 'CL' 'CYB' 'CNA' 'ms'};
settings.sensitivity.para = {'CA','CN','ms'};

% Determines which simulation is run
settings.sensitivity.stoch = false;  
% - false: consider deterministic paramters variations  (option 1)
% - true: consider random variations (option 2)

%%% Option 1 :
% Relative variations of the coefficients, as a vector. Equal for all
% coefficeint, the length does not have to match the number of parameters 
settings.sensitivity.deltaDet = [-0.15 -0.1 -0.05 0 0.05 0.1 0.15];

%%% Option 2
% Number of simulations
settings.sensitivity.N = 10;
% Relative standard deviation for each of the uncertain variables.
settings.sensitivity.stdStoch = [0.05,... % thrust
                                 0.1,... % CA
                                 0.2];   % structural mass

% Thrust uncertainty modes: 
% "independent" to apply uncertainty on all experimental points independently, 
% "same" to apply the same uncertainty to all points.                           
settings.sensitivity.thrustUncertainty = "independent"; 

%% COMPATIBILITY SETTINGS
settings.stoch.N = 1;                                     

settings.control = '0%';              

 settings.wind.input_uncertainty = [0,0]; 


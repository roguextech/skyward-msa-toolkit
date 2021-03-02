%{

CONFIG - This script sets up all the parameters for the simulation
All the parameters are stored in the "settings" structure.

Author: Francesco Colombi
Skyward Experimental Rocketry | CRD Dept | crd@skywarder.eu
email: francesco.colombi@skywarder.eu
Release date: 16/04/2016

%}

%% LAUNCH SETUP
% launchpad pont the sor
settings.z0 = 109;                                                                  %[m] Launchpad Altitude
lpin = 1.150;                                                                       %[m] Distance from base of second pin
settings.lrampa = 5.9 - lpin;                                              %[m] LaunchPad route (total available route)
settings.lat0 = 39.201778;                                                          % Launchpad latitude
settings.lon0 = -8.138368;                                                          % Launchpad longitude

% launchpad roccaraso
% settings.z0 = 1416;                                                               %[m] Launchpad Altitude
% settings.lpin = 1.150;                                                            %[m] Distance from base of second pin
% settings.lrampa = 5.9 - settings.lpin;                                            %[m] LaunchPad route (total available route)
% settings.lat0 = 41.810093;                                                        % Launchpad latitude
% settings.lon0 = 14.052546;                                                        % Launchpad longitude

settings.g0 = gravitywgs84(settings.z0, settings.lat0);                             % Gravity costant at launch latitude and altitude


settings.satellite3D = false;

settings.terrain = false;       % ATTENTION: it works only at Roccaraso
if settings.terrain  
     settings.funZ = funZ_gen('zdata.mat', settings.lat0, settings.lon0, true, 'xy');    % Altitude map computation
end

% launchpad directions
% for a single run the maximum and the minimum value of the following
% angles must be the same.
settings.OMEGAmin = 84*pi/180; % [rad] Minimum Elevation Angle, user input in degrees (ex. 80)
settings.OMEGAmax = 84*pi/180; % [rad] Maximum Elevation Angle, user input in degrees (ex. 80)
settings.PHImin = 0*pi/180;    % [rad] Minimum Azimuth Angle from North Direction, user input in degrees (ex. 90)
settings.PHImax = 0*pi/180;    % [rad] Maximum Azimuth Angle from North Direction, user input in degrees (ex. 90)
settings.upwind = false;       % If true, phi is selected according to wind direction (constant wind model only)
settings.PHIsigma = 0*pi/180;  % Stocasthic simulation only

%% ENGINE DETAILS
% load motors data 
DATA_PATH = '../data/';
filename = strcat(DATA_PATH,'Motors.mat');
Motors = load(filename);
Motors = [Motors.Cesaroni, Motors.Aerotech];

% name = 'M2020';
% name = 'M1890';
% name = 'M1800';
name = 'M2000R';
%name = 'L1365M';

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
settings.Ixxf = 0.08;                     % [kg*m^2] Inertia to x-axis
settings.Iyyf = 13.01;                    % [kg*m^2] Inertia to y-axis
settings.Izzf = 13.01;                    % [kg*m^2] Inertia to z-axis

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

% DATA_PATH = '../data/';

% Coefficients in full configuration
filename_full = strcat(DATA_PATH, 'full.mat');
CoeffsF = load(filename_full, 'Coeffs');
settings.CoeffsF = CoeffsF.Coeffs;
clear('CoeffsF');

% Coefficients in empty configuration
filename_empty = strcat(DATA_PATH, 'empty.mat');
CoeffsE = load(filename_empty, 'Coeffs');
settings.CoeffsE = CoeffsE.Coeffs;
clear('CoeffsE');

s = load(filename_full, 'State');
settings.Alphas = s.State.Alphas';
settings.Betas = s.State.Betas';
settings.Altitudes = s.State.Altitudes';
settings.Machs = s.State.Machs';
clear('s');

settings.control = '0%';      % aerobrakes 0% 50% or 100% opened

%% PARACHUTES DETAILS
% parachute 1
settings.para(1).S = 0.5;     % [m^2] Surface
settings.para(1).mass = 0.2;  % [kg] Parachute Mass
settings.para(1).CD = 0.78;   % [/] Parachute Drag Coefficient
settings.para(1).CL = 0;      % [/] Parachute Lift Coefficient
settings.para(1).delay = 1;   % [s] drogue opening delay
settings.para(1).z_cut = 600; % [m] Final altitude of the parachute

% parachute 2
settings.para(2).S = 10.5;    % [m^2] Surface
settings.para(2).mass = 1.5;  % [kg] Parachute Mass
settings.para(2).CD = 0.7;    % [/] Parachute Drag Coefficient
settings.para(2).CL = 0;      % [/] Parachute Lift Coefficient
settings.para(2).z_cut = 0;   % [m] Final altitude of the parachute

%% DESCENT PHASE MODEL
settings.descent6DOF = false;
% set to true in order to start a 6DOF parachute descent phase

% only if setting.descent6DOF == true
% additional geometry details needed
settings.xcg = 1.33;          % [m] CG postion (empty)
settings.Lnose = 0.28;        % [m] Nosecone Length

% parachute 1
settings.para(1).CX = 1.4;    % [/] Parachute Longitudinal Drag Coefficient
settings.para(1).L = 4;       % [m] Shock Chord Length
settings.para(1).K = 1000;    % [N/m^2] shock cord elastic constant
settings.para(1).C = 0;       % [Ns/m] shock cord dynamic coefficient
settings.para(1).m = 1;       % [m^2/s] Coefficient of the surface vs. time opening model
settings.para(1).nf = 12;     % [/] Adimensional Opening Time
settings.para(1).Vexit = 5;   % [m/s] expulsion speed

% parachute 2
settings.para(2).CX = 1.2;    % [/] Parachute Longitudinal Drag Coefficient
settings.para(2).L = 6;       % [m] shock cord length
settings.para(2).K = 2000;    % [N/m^2] shock cord elastic constant
settings.para(2).C = 0;       % [Ns/m] shock cord dynamic coefficient
settings.para(2).m = 1;       % [m^2/s] Coefficient of the surface vs. time opening model
settings.para(2).nf = 8.7;    % [/] Adimensional Opening Time

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
settings.ode.optionspara = odeset('Events', @event_para_cut);                       %ODE options for the parachutes
settings.ode.optionsdesc = odeset('Events', @event_landing);                        %ODE options for ballistic descent

% Settings for descent 6dof simulation
settings.ode.optionsDrogue6DOF = odeset('Events', @event_para_cut,'AbsTol',1e-6,'RelTol',1e-6);         %ODE options for due to the extraction of the main parachute
settings.ode.optionsMainExt6DOF = odeset('Events', @event_main_exit,'AbsTol',1e-6,'RelTol',1e-6);       %ODE options for due to the extraction of the main parachute
settings.ode.optionsMain6DOF = odeset('Events', @event_landing,'AbsTol',1e-6,'RelTol',1e-6);            %ODE options for due to the extraction of the main parachute

%% WIND DETAILS
% select which model you want to use:

%%%%% Matlab Wind Model
settings.wind.model = false;
% matlab hswm model, wind model on altitude based on historical data

% input Day and Hour as arrays to run stochastic simulations
settings.wind.DayMin = 105;                         % [d] Minimum Day of the launch
settings.wind.DayMax = 105;                         % [d] Maximum Day of the launch
settings.wind.HourMin = 4;                          % [h] Minimum Hour of the day
settings.wind.HourMax = 4;                          % [h] Maximum Hour of the day
settings.wind.ww = 0;                               % [m/s] Vertical wind speed

%%%%% Input wind
settings.wind.input = false;
% Wind is generated for every altitude interpolating with the coefficient defined below

settings.wind.input_ground = 7;                          % wind magnitude at the ground [m/s]
settings.wind.input_alt = [0 100 600 750 900 1500 4000]; % altitude vector [m]
settings.wind.input_mult = [0 0 10 15 20 30 30];         % percentage of increasing magnitude at each altitude
settings.wind.input_azimut = [30 30 30 30 30 30 30];     % wind azimut angle at each altitude (toward wind incoming direction) [deg]


settings.wind.input_uncertainty = [1, 1];
% settings.wind.input_uncertainty = [a,b];      wind uncertanties:
% - a, wind magnitude percentage uncertanty: magn = magn *(1 +- a)
% - b, wind direction band uncertanty: dir = dir 1 +- b

%%%%% Random wind model
% if both the model above are false

% Wind is generated randomly from the minimum to the maximum parameters which defines the wind.
% Setting the same values for min and max will fix the parameters of the wind.
settings.wind.MagMin = 5;           % [m/s] Minimum Magnitude
settings.wind.MagMax = 5;           % [m/s] Maximum Magnitude
settings.wind.ElMin = 0*pi/180;     % [rad] Minimum Elevation, user input in degrees (ex. 0)
settings.wind.ElMax = 0*pi/180;     % [rad] Maximum Elevation, user input in degrees (ex. 0) (Max == 90 Deg)
settings.wind.AzMin = (90)*pi/180; % [rad] Minimum Azimuth, user input in degrees (ex. 90)
settings.wind.AzMax = (90)*pi/180; % [rad] Maximum Azimuth, user input in degrees (ex. 90)

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

settings.stoch.N = 1;                               % Number of cases

%%% launch probability details
settings.stoch.prob.x_lim = 2e3;                    % Max ovest displacement [m]
settings.stoch.prob.V_lim = 50;                     % Max drogue velocity [Pa]

%%% Safe Ellipse (roccaraso)
settings.prob.SafeEllipse.a = 1100;
settings.prob.SafeEllipse.b = 2800;
settings.prob.SafeEllipse.x0  = 0;
settings.prob.SafeEllipse.y0 = -300;
settings.prob.SafeEllipse.alpha = 10;

%% PLOT DETAILS
settings.plots = true;

%% LANDING POINTS
% satellite maps of the landing zone 
settings.landing_map = false;

% delta limit on the coordinates 
settings.lim_lat = 0.04; 
settings.lim_lon = 0.025;

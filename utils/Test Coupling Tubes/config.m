%% AERODYNAMICS
% Coefficients in full configuration
CoeffsF = load('full.mat', 'Coeffs');
settings.CoeffsF = CoeffsF.Coeffs;
clear('CoeffsF');

% Coefficients in empty configuration
CoeffsE = load('empty.mat', 'Coeffs');
settings.CoeffsE = CoeffsE.Coeffs;
clear('CoeffsE');

% Coefficients of the payload
CoeffsP = load('payloadAero.mat', 'Coeffs');
settings.CoeffsP = CoeffsP.Coeffs;
clear('CoeffsP');

s = load('full.mat', 'State');
settings.Alphas = s.State.Alphas';
settings.Betas = s.State.Betas';
settings.Altitudes = s.State.Altitudes';
settings.Machs = s.State.Machs';
clear('s');

%% LAUNCH SETUP
% launchpad - pont the sor
settings.z0 = 109;                                      % [m] Launchpad Altitude
settings.lpin = 1.150;                                  % [m] Distance from base of second pin
settings.lrampa = 5.9 - settings.lpin;                  % [m] LaunchPad route (total available route)
settings.lat0 = 39.201778;                              % Launchpad latitude
settings.lon0 = -8.138368;                              % Launchpad longitude

% launchpad directions
% for a single run the maximum and the minimum value of the following angles must be the same.
settings.OMEGAmin = 84*pi/180;                              % [rad] Minimum Elevation Angle, user input in degrees (ex. 80)
settings.OMEGAmax = 84*pi/180;                              % [rad] Maximum Elevation Angle, user input in degrees (ex. 80)
settings.PHImin = 0*pi/180;                                 % [rad] Minimum Azimuth Angle from North Direction, user input in degrees (ex. 90)
settings.PHImax = 0*pi/180;                                 % [rad] Maximum Azimuth Angle from North Direction, user input in degrees (ex. 90)

% !! ATTENTION !! The following 2 work just for the stochastichs simulations with constant wind model only
settings.upwind = false;                                    % If true, phi is selected opposite to the wind direction
settings.PHIsigma = 0*pi/180;                               % [deg] If upwind is true, you can select a variance for the direction

% launchpad - roccaraso
% settings.z0 = 1416;                                     % [m] Launchpad Altitude
% settings.lpin = 1.150;                                  % [m] Distance from base of second pin
% settings.lrampa = 5.9 - settings.lpin;                  % [m] LaunchPad route (total available route)
% settings.lat0 = 41.810093;                              % Launchpad latitude
% settings.lon0 = 14.052546;                              % Launchpad longitude

settings.g0 = gravitywgs84(settings.z0, settings.lat0); % Gravity costant at launch latitude and altitude

%% ENGINE DETAILS
% load motors data
Motors = load('Motors.mat'); Motors = [Motors.Cesaroni, Motors.Aerotech];

name = 'M2000Rbis';
% name = 'L1350-CS';
iMotor = [Motors.MotorName] == name;
settings.motor.expTime = Motors(iMotor).t;
settings.motor.expThrust = Motors(iMotor).T;
settings.motor.expM = Motors(iMotor).m;
settings.mp = Motors(iMotor).mp;                        % [kg]   Propellant Mass                                                
settings.tb = Motors(iMotor).t(end) ;                   % [s]    Burning time
mm = Motors(iMotor).mm;                                 % [kg]   Total Mass of the Motor 
settings.mNoMot = 17.873;
settings.ms = settings.mNoMot + mm - settings.mp;       % [kg]   Structural Mass
settings.m0 = settings.ms + settings.mp;                % [kg]   Total Mass
settings.mnc = 0.400;                                   % [kg]   Nosecone Mass
settings.mPayload = 5.5;

%% GEOMETRY/DATCOM DETAILS
% This parameters should be the same parameters set up in MISSILE DATCOM
% simulation.

settings.C = 0.15;                                         % [m] Caliber (Fuselage Diameter)
settings.S = pi*settings.C^2/4;                            % [m^2] Cross-sectional Surface
settings.xcg = [0.4, 1.33];                               % [m] CG postion [full, empty]
settings.xcgPay = 0.4;
settings.Lnose = 0.26;                                     % [m] Nosecone Length
settings.OgType = 'KARMAN';
% settings.rocketLength = 2.495;                             % [m] Rocket Length
settings.Lcenter = 0.34; % [m] fuselage length

%%% finset
settings.Chord1 = 0.35;                                    % [m] attached chord length
settings.Chord2 = 0.12;                                    % [m] free chord length
settings.Height = 0.12;                                    % [m] fin heigth     
settings.shape = 'rect';                                   % [/] fin shape
settings.Npanel = 3;                                       % [m] number of fins
settings.Ler = 0.003;                                      % [deg] Leading edge radius
settings.d = 0;                                            % [m] rocket tip-fin distance
settings.zupRaw = 0.0015;                                 % [m] fin semi-thickness 
settings.LmaxuRaw = 0.0015;                               % [m] Fraction of chord from leading edge to max thickness

%%% protub data
settings.xprot = settings.Lcenter + settings.Lnose - 0.85; % axial position 
settings.nloc = 3;                                         % number of brakes
settings.lprot = 0.005;                                    % brakes thickness
settings.wprot = 0.088;                                    % brakes width

%% MASS GEOMERTY DETAILS
% x-axis: along the fuselage
% y-axis: right wing
% z-axis: downward

% inertias for full configuration (with all the propellant embarqued) obtained with CAD's
settings.Ixxf = 0.0619;                       % [kg*m^2] Inertia to x-axis
settings.Iyyf = 0.1959;                      % [kg*m^2] Inertia to y-axis
settings.Izzf = 0.1959;                      % [kg*m^2] Inertia to z-axis

% inertias for empty configuration (all the propellant consumed) obtained with CAD's
settings.Ixxe = 0.07;                       % [kg*m^2] Inertia to x-axis
settings.Iyye = 10.06;                      % [kg*m^2] Inertia to y-axis
settings.Izze = 10.06;                      % [kg*m^2] Inertia to z-axis

% inertias of the payload
settings.IxxP = 0.0619;                       % [kg*m^2] Inertia to x-axis
settings.IyyP = 0.1959;                      % [kg*m^2] Inertia to y-axis
settings.IzzP = 0.1959;                      % [kg*m^2] Inertia to z-axis

%% INTEGRATION OPTIONS
settings.ode.finalTime =  2000;    % [s] Final integration time

% create an option structure for the integrations:

% - AbsTol is the threshold below which the value of the solution becomes unimportant
% - RelTol is the tolerance betweeen two consecutive values
% - Events is the event function that defines when the integration must be
% - stopped (it has to be created)
% - InitialStep is the highest value tried by the solver

settings.ode.optionsasc1 = odeset('Events', @eventApogee, 'InitialStep', 1);       %ODE options for ascend

%% ROCCARASO TERRAIN
% !! ATTENTION !!  the following works only at Roccaraso
settings.terrain = false; 
if settings.terrain  
     settings.funZ = funZGen('zData.mat', settings.lat0, settings.lon0);
end

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

settings.wind.inputGround = 20;                            % wind magnitude at the ground [m/s]
settings.wind.inputAlt = [0 100 600 750 900 1500 4000];    % altitude vector [m]
settings.wind.inputMult = [0 0 10 15 20 30 30];            % percentage of increasing magnitude at each altitude
settings.wind.inputAzimut = [360 90 360 10 50 280 10];        % wind azimut angle at each altitude (toward wind incoming direction) [deg]

settings.wind.inputUncertainty = [0, 0];
% settings.wind.inputUncertainty = [a,b];      wind uncertanties:
% - a, wind magnitude percentage uncertanty: magn = magn *(1 +- a)
% - b, wind direction band uncertanty: dir = dir 1 +- b

%%%%% Random wind model
% if both the model above are false

% Wind is generated randomly from the minimum to the maximum parameters which defines the wind.
% Setting the same values for min and max will fix the parameters of the wind.
settings.wind.MagMin = 20;                                   % [m/s] Minimum Magnitude
settings.wind.MagMax = 20;                                   % [m/s] Maximum Magnitude
settings.wind.ElMin = 0*pi/180;                             % [rad] Minimum Elevation, user input in degrees (ex. 0)
settings.wind.ElMax = 0*pi/180;                             % [rad] Maximum Elevation, user input in degrees (ex. 0) (Max == 90 Deg)
settings.wind.AzMin = (45)*pi/180;                         % [rad] Minimum Azimuth, user input in degrees (ex. 90)
settings.wind.AzMax = (45)*pi/180;                         % [rad] Maximum Azimuth, user input in degrees (ex. 90)

% NOTE: wind azimuth angle indications (wind directed towards):
% 0 deg (use 360 instead of 0)  -> North
% 90 deg                        -> East
% 180 deg                       -> South
% 270 deg                       -> West

%% BALLISTIC SIMULATION
% Set to True to run a ballistic (without drogues) simulation
settings.ballistic = true;

%% STOCHASTIC DETAILS
% If N > 1 the stochastic routine is started
settings.stoch.N = 1;                                       % Number of cases
% Choose to open a parallel pool of local or threads workers
settings.parThreads = true;                                      % set to false to run parpool of local workers

%% PLOT DETAILS
settings.plots = true;

%% LANDING POINTS
% satellite maps of the landing zone 
settings.landingMap = false;                               % 2D map
settings.satellite3D = false;                               % 3D map


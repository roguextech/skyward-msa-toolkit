%{

%}

%% LAUNCH SETUP
% launchpad - pont the sor
settings.z0 = 109;                                      % [m] Launchpad Altitude
settings.lpin = 1.150;                                  % [m] Distance from base of second pin
settings.lrampa = 5.9 - settings.lpin;                  % [m] LaunchPad route (total available route)
settings.lat0 = 39.201778;                              % Launchpad latitude
settings.lon0 = -8.138368;                              % Launchpad longitude

% launchpad - roccaraso
% settings.z0 = 1416;                                     % [m] Launchpad Altitude
% settings.lpin = 1.150;                                  % [m] Distance from base of second pin
% settings.lrampa = 5.9 - settings.lpin;                  % [m] LaunchPad route (total available route)
% settings.lat0 = 41.810093;                              % Launchpad latitude
% settings.lon0 = 14.052546;                              % Launchpad longitude

settings.g0 = gravitywgs84(settings.z0, settings.lat0); % Gravity costant at launch latitude and altitude

%% ENGINE DETAILS
% load motors data 
filename = strcat(dataPath,'Motors.mat');
Motors = load(filename); Motors = [Motors.Cesaroni, Motors.Aerotech];

name = 'M2000Rbis';
% name = 'L1350';
iMotor = [Motors.MotorName] == name;
settings.motor.exp_time = Motors(iMotor).t;
settings.motor.exp_thrust = Motors(iMotor).T;
settings.motor.exp_m = Motors(iMotor).m;
settings.mp = Motors(iMotor).mp;                        % [kg]   Propellant Mass                                                
settings.tb = Motors(iMotor).t(end) ;                   % [s]    Burning time
mm = Motors(iMotor).mm;                                 % [kg]   Total Mass of the Motor 
settings.mNoMot = 17.873;
settings.ms = settings.mNoMot + mm - settings.mp;       % [kg]   Structural Mass
settings.m0 = settings.ms + settings.mp;                % [kg]   Total Mass
settings.mnc = 0.400;                                   % [kg]   Nosecone Mass

%% GEOMETRY/DATCOM DETAILS
% This parameters should be the same parameters set up in MISSILE DATCOM
% simulation.

settings.C = 0.15;                                                % [m] Caliber (Fuselage Diameter)
settings.S = pi*settings.C^2/4;                                   % [m^2] Cross-sectional Surface
settings.xcg = [1.49, 1.33];                                      % [m] CG postion [full, empty]
settings.Lnose = 0.26;                                            % [m] Nosecone Length
settings.OgType = 'KARMAN';
settings.rocketLength = 2.495;                                    % [m] Rocket Length
settings.Lcenter = settings.rocketLength - settings.Lnose;        % [m] fuselage length

%%% finset
settings.Chord1 = 0.35;                                           % [m] attached chord length
settings.Chord2 = 0.12;                                           % [m] free chord length
settings.Height = 0.12;                                           % [m] fin heigth     
settings.shape = 'rect';                                          % [/] fin shape
settings.Npanel = 3;                                              % [m] number of fins
settings.Ler = 0.003;                                             % [deg] Leading edge radius
settings.d = 0;                                                   % [m] rocket tip-fin distance
settings.zup_raw = 0.0015;                                        % [m] fin semi-thickness 
settings.Lmaxu_raw = 0.0015;                                      % [m] Fraction of chord from leading edge to max thickness

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
settings.Ixxf = 0.08;                       % [kg*m^2] Inertia to x-axis
settings.Iyyf = 13.05;                      % [kg*m^2] Inertia to y-axis
settings.Izzf = 13.05;                      % [kg*m^2] Inertia to z-axis

% inertias for empty configuration (all the propellant consumed) obtained with CAD's
settings.Ixxe = 0.07;                       % [kg*m^2] Inertia to x-axis
settings.Iyye = 10.06;                      % [kg*m^2] Inertia to y-axis
settings.Izze = 10.06;                      % [kg*m^2] Inertia to z-axis

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
filename_full = strcat(dataPath, 'full.mat');
CoeffsF = load(filename_full, 'Coeffs');
settings.CoeffsF = CoeffsF.Coeffs;
clear('CoeffsF');

% Coefficients in empty configuration
filename_empty = strcat(dataPath, 'empty.mat');
CoeffsE = load(filename_empty, 'Coeffs');
settings.CoeffsE = CoeffsE.Coeffs;
clear('CoeffsE');

s = load(filename_full, 'State');
settings.Alphas = s.State.Alphas';
settings.Betas = s.State.Betas';
settings.Altitudes = s.State.Altitudes';
settings.Machs = s.State.Machs';
clear('s');

%% PARACHUTES DETAILS
% parachute 1
settings.para(1).S = 0.4;       % [m^2]   Surface
settings.para(1).mass = 0.2;    % [kg]   Parachute Mass
settings.para(1).CD = 0.78;     % [/] Parachute Drag Coefficient
settings.para(1).CL = 0;        % [/] Parachute Lift Coefficient
settings.para(1).delay = 1;     % [s] drogue opening delay
settings.para(1).z_cut = 450;   % [m] Final altitude of the parachute

% parachute 2
settings.para(2).S = 10.5;      % [m^2]   Surface
settings.para(2).mass = 1.5;    % [kg]   Parachute Mass
settings.para(2).CD = 0.7;      % [/] Parachute Drag Coefficient
settings.para(2).CL = 0;        % [/] Parachute Lift Coefficient
settings.para(2).z_cut = 0;     % [m] Final altitude of the parachute

% parachute 1
settings.para(1).CX = 1.4;      % [/] Parachute Longitudinal Drag Coefficient
settings.para(1).L = 4;         % [m] Shock Chord Length
settings.para(1).K = 1000;      % [N/m^2] Shock Chord Elastic Constant
settings.para(1).C = 0;         % [Ns/m] Shock Chord Dynamic Coefficient
settings.para(1).m = 1;         % [m^2/s] Coefficient of the surface vs. time opening model
settings.para(1).nf = 12;       % [/] Adimensional Opening Time
settings.para(1).Vexit = 5;     % [m/s] Expulsion Speed

% parachute 2
settings.para(2).CX = 1.2;      % [/] Parachute Longitudinal Drag Coefficient
settings.para(2).L = 6;         % [m] Shock Chord Length
settings.para(2).K = 2000;      % [N/m^2] Shock Chord Elastic Constant
settings.para(2).C = 0;         % [Ns/m] Shock Chord Dynamic Coefficient
settings.para(2).m = 1;         % [m^2/s] Coefficient of the surface vs. time opening model
settings.para(2).nf = 8.7;      % [/] Adimensional Opening Time

%% INTEGRATION OPTIONS
settings.ode.final_time =  2000;    % [s] Final integration time

% create an option structure for the integrations:

% - AbsTol is the threshold below which the value of the solution becomes unimportant
% - RelTol is the tolerance betweeen two consecutive values
% - Events is the event function that defines when the integration must be
% - stopped (it has to be created)
% - InitialStep is the highest value tried by the solver

settings.ode.optionsasc1 = odeset('Events', @eventApogee, 'InitialStep', 1);       %ODE options for ascend
settings.ode.optionsasc2 = odeset('InitialStep', 1);                                %ODE options for due to the opening delay of the parachute
settings.ode.optionspara = odeset('Events', @event_para_cut);                       %ODE options for the parachutes
settings.ode.optionsdesc = odeset('Events', @event_landing);                        %ODE options for ballistic descent
settings.ode.optionspad = odeset('Events', @EventPad);                              %ODE options for the launchpad phase

% Settings for descent 6dof simulation
settings.ode.optionsDrogue6DOF = odeset('Events', @event_para_cut,'AbsTol',1e-6,'RelTol',1e-6);         %ODE options for due to cutting of the drogue chute
settings.ode.optionsMainExt6DOF = odeset('Events', @event_main_exit,'AbsTol',1e-6,'RelTol',1e-6);       %ODE options for due to the extraction of the main chute
settings.ode.optionsMain6DOF = odeset('Events', @event_landing,'AbsTol',1e-6,'RelTol',1e-6);            %ODE options to terminate descent phase


%% STOCHASTIC DETAILS
%%% launch probability details
settings.stoch.prob.x_lim = 2e3;                    % Max ovest displacement [m]
settings.stoch.prob.V_lim = 50;                     % Max drogue velocity [Pa]
settings.stoch.prob.XCP_lim = 1.5;                  % Min XCP

%%% Safe Ellipse - roccaraso
settings.prob.SafeEllipse.a = 1100;
settings.prob.SafeEllipse.b = 2800;
settings.prob.SafeEllipse.x0  = 0;
settings.prob.SafeEllipse.y0 = -300;
settings.prob.SafeEllipse.alpha = 10;

%% PLOT DETAILS
settings.plots = true;

%% LANDING POINTS
% satellite maps of the landing zone 
% delta limit on the coordinates (for the landing map)
settings.lim_lat = 0.04; 
settings.lim_lon = 0.025;

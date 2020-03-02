
%% LAUNCH SETUP

% launchpad
settings.z0 = 0;                   %[m] Launchpad Altitude
settings.lrampa = 4.9;                %[m] LaunchPad route (launchpad length-distance from ground of the first hook)

% launchpad directions
% for a single run the maximum and the minimum value of the following
% angles must be the same.
settings.OMEGA = 85*pi/180;        %[rad] Minimum Elevation Angle, user input in degrees (ex. 80)
settings.PHI = 0*pi/180;       %[rad] Maximum Azimuth Angle from North Direction, user input in degrees (ex. 90)

%% ENGINE DETAILS

% %
% %         settings.motor.Name = 'K550';
% %         settings.motor.exp_time = [0 0.13 0.38 0.63 0.88 1.14 1.39...
% %             1.64 1.9 2.15 2.40 2.66 2.91 3.16 3.5];                         %[s]
% %
% %         settings.motor.exp_thrust = [ 0 139.8 158.07 171.978 178.769 ...
% %             178.247 158.859 132.922 111.005 92.082 74.075 44.837 16.156...
%             4.589 0.000  ] * 9.81/2.2;                                      % [N]
%
%         settings.ms = 6.330;                                                % [kg]   Structural Mass
%         settings.mp = 0.889;                                                % [kg]   Propellant Mass
%         settings.m0 = settings.ms + settings.mp;                            % [kg]   Overall Mas
%         settings.mnc = 0.300;                                               % [kg]   Nosecone Mass
%         settings.tb = 3.5;                                                  % [s]    Burning time
%


%         % Aerotech K695R-L
%         settings.motor.exp_time = [0, 0.02:0.05:0.82, 0.88:0.05:2.23];
%
%         settings.motor.exp_thrust = [ 0 540.57 716.61 724.39 740.18 751.53 762.31 821.36 908.55 894.53 885.86 881.97 875.41 869.85 863.18 857.51 847.39,...
%           844.38 834.96 825.7 817.69 810.69 793.9 781.77 766.09 750.53 739.41 721.05 703.71 689.03 674.91 662.67 646.1,...
%           633.76 616.52 603.96 590.2 574.71 567.59 569.37 463.39 268.23 121.55 40.92 7.23 3.91];
%
%         settings.mp = 0.918;
%         settings.tb = settings.motor.exp_time(end);
%         settings.mfr = settings.mp/settings.tb;                             % [kg/s] Mass Flow Rate
%
%
%         % 75 max power
%
%         settings.motor.exp_time = [0 0.1000 0.2000 0.3000 0.4000 0.5000 1.0000 1.5000 2.0000 2.5000 3.0000 3.2500 3.3500 3.5000 3.7500 3.8000 4.0000 4.2500 4.5000 4.7500 5.0000 5.5000 6.0000 6.5000];
%
%         settings.motor.exp_thrust = [0 2411,2135,2015,2000,2055,2098,1860,1788,1659,1468,1423,1334,1201,934,930,881,600,468,400,290,85,23,0];
%
%         settings.mp = 3.951;

%%
% motor I366 
% 
% DATA = ...
%     [ 0       0
%     0.027 323.256
%     0.088 485.393
%     0.148 483.744
%     0.208 479.926
%     0.268 473.365
%     0.327 466.192
%     0.388 457.444
%     0.448 448.751
%     0.509 441.477
%     0.570 430.236
%     0.630 421.524
%     0.690 411.757
%     0.750 398.876
%     0.810 387.496
%     0.870 375.430
%     0.930 361.325
%     0.991 345.057
%     1.053 330.392
%     1.112 312.636
%     1.173 293.508
%     1.233 275.085
%     1.292 262.408
%     1.353 230.881
%     1.413 118.008
%     1.474 23.611
%     1.535 0.000];
% 
% settings.motor.exp_time = DATA(:, 1);
% settings.motor.exp_thrust = DATA(:, 2);
% settings.mp = 0.3136;

% motor I327

DATA=[  0  0
    0.0090 393.741
   0.026 432.602
   0.161 387.142
   0.245 382.742
   0.303 371.011
   0.601 372.477
   0.702 371.011
   0.798 375.41
   0.983 371.744
   1.04 370.278
   1.104 371.744
   1.199 343.148
   1.3 335.816
   1.335 357.813
   1.365 327.751
   1.399 265.427
   1.428 185.505
   1.451 156.91
   1.493 112.183
   1.553 68.19
   1.601 32.262
   1.656 11.732
   1.723 0.0];

settings.motor.exp_time = DATA(:, 1);
settings.motor.exp_thrust = DATA(:, 2);
settings.mp = 0.354;


settings.tb = settings.motor.exp_time(end);
settings.mfr = settings.mp/settings.tb;



      
        
        
        
        
        
% GEOMETRY DETAILS
% This parameters should be the same parameters set up in MISSILE DATCOM
% simulation.

settings.C = 0.09;                          % [m]      Caliber (Fuselage Diameter)
settings.S = 0.0064;                        % [m^2]    Cross-sectional Surface
L = 2.02;                                   % [m]      Rocket length

%% MASS GEOMERTY DETAILS
% x-axis: along the fuselage
% y-axis: right wing
% z-axis: downward

% inertias for full configuration (with all the propellant embarqued) obtained with CAD's
settings.Ixxf = 0.008795446;                    % [kg*m^2] Inertia to x-axis
settings.Iyyf = 2.050393979;                    % [kg*m^2] Inertia to y-axis
settings.Izzf = 2.050413838;                    % [kg*m^2] Inertia to z-axis

% inertias for empty configuration (all the propellant consumed) obtained with CAD's
settings.Ixxe = 0.008472446;                    % [kg*m^2] Inertia to x-axis
settings.Iyye = 1.712284592;                    % [kg*m^2] Inertia to y-axis
settings.Izze = 1.712304085;                    % [kg*m^2] Inertia to z-axis


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

 DATA_PATH = '/Users/teo/Desktop/skyward/skyward-matlab-rocket-simulator/data/';

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

settings.ode.final_time =  2000;                 % [s] Final integration time

% create an option structure for the integrations:

% - AbsTol is the threshold below which the value of the solution becomes unimportant
% - RelTol is the tolerance betweeen two consecutive values
% - Events is the event function that defines when the integration must be
% - stopped (it has to be created)
% - InitialStep is the highest value tried by the solver

settings.ode.optionsasc = odeset('AbsTol',1E-3,'RelTol',1E-3,...
    'Events',@event_apogee,'InitialStep',1);    %ODE options for ascend

settings.ode.optionsdrg1 = odeset('AbsTol',1E-3,'RelTol',1E-3,...
    'Events',@event_drg2_opening);              %ODE options for drogue

settings.ode.optionsdrg2 = odeset('AbsTol',1E-3,'RelTol',1E-3,...
    'Events',@event_landing);              %ODE options for drogue

settings.ode.optionsdesc = odeset('AbsTol',1E-3,'RelTol',1E-12,...
    'Events',@event_landing);                   %ODE options for ballistic descent



%% Random wind model


% Wind is generated randomly from the minimum to the maximum parameters which defines the wind.
% Setting the same values for min and max will fix the parameters of the wind.
settings.wind.MagMin = 4;                   % [m/s] Minimum Magnitude
settings.wind.MagMax = 4;                   % [m/s] Maximum Magnitude
settings.wind.ElMin = 0*pi/180;             % [rad] Minimum Elevation, user input in degrees (ex. 0)
settings.wind.ElMax = 0*pi/180;             % [rad] Maximum Elevation, user input in degrees (ex. 0) (Max == 90 Deg)
settings.wind.AzMin = (180)*pi/180;         % [rad] Minimum Azimuth, user input in degrees (ex. 90)
settings.wind.AzMax = (180)*pi/180;         % [rad] Maximum Azimuth, user input in degrees (ex. 90)

% NOTE: wind aziumt angle indications (wind directed towards):
% 0 deg (use 360 instead of 0)  -> North
% 90 deg                        -> East
% 180 deg                       -> South
% 270 deg                       -> West

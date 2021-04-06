%{
configOptimization - this script sets the user configuration for the aerodynamics
surfaces optimization.

CALLED SCRIPTS: /

CALLED FUNCTIONS: /

CALLED DATA FILES: /

REVISIONS:
- 0     21/10/20,   release     Adriano Filippo Inno
%}

%% LAUNCH SETUP
settings.OMEGA = 84*pi/180;        % [rad] Minimum Elevation Angle, user input in degrees (ex. 80)
settings.PHI = 0*pi/180;           % [rad] Maximum Azimuth Angle from North Direction, user input in degrees (ex. 90)

%% wind
settings.wind.Mag = 9;             % [m/s] Maximum wind-magnitude allowed

%% Optimization Choice
settings.minStabilityMargin = 1.5; % minum stability margin required

%% States
% State values in which the aerodynamic coefficients will be computed
datcom.Mach = 0.1:0.1:1;
datcom.Alpha = [-20 -15 -10 -7.5 -5 -2.5 -1 -0.1 0 0.1 1 2.5 5 7.5 10 15 20];
datcom.Beta = 0;
datcom.Alt = settings.z0 + 0:1000:4000; 

%% VARIABLES BOUNDARIES
%%% Lower boundary
lb(1) = 35;                        % 1 --> chord1         [cm]
lb(2) = 10;                        % 2 --> chord2         [cm]
lb(3) = 10;                        % 3 --> heigth         [cm]
lb(4) = 2;                         % 4 --> Fin type       [/]
lb(5) = 28;                        % 5 --> Ogive Length   [cm]
lb(6) = 1;                         % 6 --> Ogive Type     [/]

%%% Upper boundary
ub(1) = 35;                        % 1 --> chord1         [cm]
ub(2) = 20;                        % 2 --> chord2         [cm]
ub(3) = 40;                        % 3 --> heigth         [cm]
ub(4) = 2;                         % 4 --> Fin type       [/]
ub(5) = 28;                        % 5 --> Ogive Length   [cm]
ub(6) = 1;                         % 6 --> Ogive Type     [/]

%%% Inequality constraint (A*x < b)
% imposing the fixed chord, x(1), to be greater than the free chord x(2)
% so -x(1) + x(2) < 0
% imposing the fixed chord, x(1), to be greater than the heigth x(3) to
% reduce the flexibility, so -x(1) + x(3) < 0

A = [-1 1 0 0 0 0
     -1 0 1 0 0 0 ];
b = [0; 0];

%% COMPATIBILITY SETTINGS
% this settings are needed to work with the commonFunctions folder,  do not
% modify it unless you now what you're doing
settings.stoch.N = 1;                                     
settings.control = 1;              
settings.wind.model = false;
settings.wind.input = false;

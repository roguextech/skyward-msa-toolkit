%{
CONFIG - This script sets up all the parameters for missile Datcom
All the parameters are stored in the "datcom" structure.

Author: Adriano Filippo Inno
Skyward Experimental Rocketry | CRD Dept | crd@skywarder.eu
email: adriano.filippo.inno@skywarder.eu
Release date: 18/10/2019

%}

%% States
% State values in which the aerodynamic coefficients will be computed
datcom.Mach = 0.05:0.05:1;
datcom.Alpha = [-10 -7.5 -5 -2.5 -1.5 -1 -0.5 -0.1 0 0.1 0.5 1 1.5 2.5 5 7.5 10];
datcom.Beta = [-0.1 0 0.1];
datcom.Alt = 0:200:4000;

%% Design Parameters
datcom.Chord1 = 0.3; 
datcom.Chord2 = 0.1; 
datcom.Height = 0.15;                            
datcom.shape = 'parall';

%% Fixed Parameters
vars.xcg = [1.75, 1.62];                        % [m] CG position [full, empty]
datcom.D = 0.15;                                % [m] rocket diameter
datcom.Lnose = 0.37;                            % [m] nose length
datcom.Lcenter = 2.4;                           % [m] Lcenter : Centerbody length
datcom.Npanel = 3;                              % [m] number of fins
datcom.Phif = [0 120 240];                      % [deg] Angle of each panel
datcom.Ler = 0.003;                             % [deg] Leading edge radius
datcom.d = 0;                                   % [m] rocket tip-fin distance
datcom.zup_raw = 0.0015;                        % [m] fin semi-thickness 
datcom.Lmaxu_raw = 0.006;                       % [m] Fraction of chord from leading edge to max thickness

%% Ogive parameters
datcom.OgType = 'KARMAN';
datcom.NosePower = 1/2;

%% Protuberance parameters
datcom.xprot = 1; % axial position 
datcom.nloc = 3; % number of brakes
datcom.lprot = 0.005; % brakes thickness
datcom.wprot = 0.088; % brakes width
vars.hprot = [0 0.0387/2 0.0387]; % brakes length, first entry must be always 0!

%% Run 
autoMatricesProtub(datcom, vars);

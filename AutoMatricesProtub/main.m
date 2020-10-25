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
datcom.s.Mach = 0.1:0.1:1;
datcom.s.Alpha = [-10 -5 -2.5 -1.5 -1 -0.5 -0.1 0.1 0.5 1 1.5 2.5 5 10];
datcom.s.Beta = [-0.1 0.1];
datcom.s.Alt = 0:500:4000;

%% Design Parameters
% looping for various dimension of the fins [m]
datcom.design.Chord1 = 0.3; 
datcom.design.Chord2 = 0.15; 
datcom.design.shape = 'parall';

%% Fixed Parameters
datcom.para.xcg = [2.2, 2.1];                       % [m] CG position [full, empty]
datcom.para.D = 0.15;                               % [m] rocket diameter
datcom.para.S = datcom.para.D^2/4*pi;               % [m^2] rocket cross section
datcom.para.Lnose = 0.3;                            % [m] nose length
datcom.para.Lcenter = 2.5;                          % [m] Lcenter : Centerbody length
datcom.para.Npanel = 3;                             % [m] number of fins
datcom.para.Phif = [0 120 240];                     % [deg] Angle of each panel
datcom.para.Ler = 0.003;                            % [deg] Leading edge radius
datcom.para.d = 0;                                  % [m] rocket tip-fin distance
datcom.para.zup_raw = 0.0015;                       % [m] fin semi-thickness 
datcom.para.Lmaxu_raw = 0.006;                      % [m] Fraction of chord from leading edge to max thickness
datcom.para.C1Hratio = 2;                           % [/] fin chord-heigth ratio

%% Protuberance parameters
datcom.xprot = 1; % axial position 
datcom.nloc = 3; % number of brakes
datcom.lprot = 0.005; % brakes thickness
datcom.wprot = 0.08; % brakes width
datcom.hprot = 0:0.001:0.04; % bakes length, first entry must be always 0!

%% Run 
autoMatricesProtub(datcom);


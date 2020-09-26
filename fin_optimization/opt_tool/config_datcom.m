%{
CONFIG - This script sets up all the parameters for missile Datcom
All the parameters are stored in the "datcom" structure.

Author: Ruben Di Battista
Skyward Experimental Rocketry | CRD Dept | crd@skywarder.eu
email: ruben.dibattista@skywarder.eu
Website: http://www.skywarder.eu
License: 2-clause BSD

Author: Francesco Colombi
Skyward Experimental Rocketry | CRD Dept | crd@skywarder.eu
email: francesco.colombi@skywarder.eu

Author: Adriano Filippo Inno
Skyward Experimental Rocketry | CRD Dept | crd@skywarder.eu
email: adriano.filippo.inno@skywarder.eu
Release date: 18/10/2019

%}

%% States
% State values in which the aerodynamic coefficients will be computed
datcom.s.Mach = 0.05:0.05:1;
datcom.s.Alpha = [-10 -7.5 -5 -2.5 -1.5 -1 -0.5 -0.1 0.1 0.5 1 1.5 2.5 5 7.5 10];
datcom.s.Beta = [-0.1 0.1];
datcom.s.Alt = 0:200:4000;

%% Design Parameters
% looping for various dimension of the fins [m]
datcom.design.Chord1 = 0.17:0.01:0.2; 
datcom.design.Chord2 = 0.08:0.01:0.1; 
datcom.design.shape = 'rect';

%% Fixed Parameters
datcom.para.xcg = [1.7, 1.6];                       % [m] CG position [full, empty]
datcom.para.D = settings.C;                         % [m] rocket diameter
datcom.para.S = settings.S;                         % [m^2] rocket cross section
datcom.para.Lnose = 0.5;                            % [m] nose length
datcom.para.Lcenter = 2.15;                         % [m] Lcenter : Centerbody length
datcom.para.Npanel = 4;                             % [m] number of fins
datcom.para.Phif = [0 90 180 270];                  % [deg] Angle of each panel
datcom.para.Ler = 0.003;                            % [deg] Leading edge radius
datcom.para.d = 0;                                  % [m] rocket tip-fin distance
datcom.para.zup_raw = 0.0015;                       % [m] fin semi-thickness 
datcom.para.Lmaxu_raw = 0.006;                      % [m] Fraction of chord from leading edge to max thickness
datcom.para.C1Hratio = 2;                           % [/] fin chord-heigth ratio

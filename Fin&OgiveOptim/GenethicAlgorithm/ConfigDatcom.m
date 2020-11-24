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
datcom.Mach = 0.1:0.1:1;
datcom.Alpha = [-20 -15 -10 -7.5 -5 -2.5 -1 -0.1 0 0.1 1 2.5 5 7.5 10 15 20];
datcom.Beta = 0;
datcom.Alt = 0:1000:4000; 

%% Fixed Parameters
datcom.xcg = [1.75, 1.62] - 0.4;                           % [m] CG position from the end of Lcenter [full, empty]
datcom.D = settings.C;                                     % [m] rocket diameter
datcom.S = settings.S;                                     % [m^2] rocket cross section
datcom.Lcenter = 2.4;                                      % [m] Lcenter : Centerbody length
datcom.Npanel = 3;                                         % [m] number of fins
datcom.Phif = [0 120 240];                                 % [deg] Angle of each panel
datcom.Ler = 0.003;                                        % [deg] Leading edge radius
datcom.d = 0;                                              % [m] rocket tip-fin distance
datcom.zup_raw = 0.0015;                                   % [m] fin semi-thickness 
datcom.Lmaxu_raw = 0.006;                                  % [m] Fraction of chord from leading edge to max thickness

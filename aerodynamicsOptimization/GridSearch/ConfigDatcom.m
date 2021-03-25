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
datcom.s.Alpha = [-7.5 -5 -2.5 -1.5 -1 -0.5 -0.1 0 0.1 0.5 1 1.5 2.5 5 7.5];
datcom.s.Beta = [-0.1 0 0.1];
datcom.s.Alt = 0:500:4000;

%% Design Parameters
%%%%%%% fins
datcom.design.Chord1 = 0.3:0.01:0.4;                           % [m] chord fixed length
datcom.design.Chord2 = 0.13:0.01:0.15;                            % [m] chord free length 
datcom.design.Heigth = 0.13:0.01:0.15;                            % [m] chord free length 
rect = false; iso = false; parall = true;                       % choose the shapes that you wanna try

%%%%%%% ogive
datcom.design.Lnose = 0.3;%0.25:0.01:0.3;                             % [m] ogive length
datcom.design.NosePower = [];                                   % [/] Power coefficient of the NoseCone, put a empty vector to avoid power ogive.
Karman = true; Haack = false; Ogive = false;                    % choose the shapes that you wanna try 

%% Fixed Parameters
datcom.para.xcg = [1.9, 1.8];                                   % [m] CG position [full, empty]
datcom.para.D = settings.C;                                     % [m] rocket diameter
datcom.para.S = settings.S;                                     % [m^2] rocket cross section
datcom.para.Lcenter = 2.5;                                      % [m] Lcenter : Centerbody length
datcom.para.Npanel = 3;                                         % [m] number of fins
datcom.para.Ler = 0.003;                                        % [deg] Leading edge radius
datcom.para.d = 0;                                              % [m] rocket tip-fin distance
datcom.para.zup_raw = 0.0015;                                   % [m] fin semi-thickness 
datcom.para.Lmaxu_raw = 0.006;                                  % [m] Fraction of chord from leading edge to max thickness

%% Do not touch these parameters 
Shape = ["rect", "iso", "parall"];                              
datcom.design.Shape = Shape([rect, iso, parall]);                 
OtherTypes = ["KARMAN", 'HAACK', 'OGIVE'];                     
OtherTypes = OtherTypes([Karman, Haack, Ogive]);          
datcom.design.OgType = [repmat("POWER", [1, length(datcom.design.NosePower)]), OtherTypes];    


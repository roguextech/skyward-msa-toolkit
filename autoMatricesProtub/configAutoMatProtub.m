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
datcom.Alpha = [-22 -15 -10 -7.5 -5 -2.5 -1 -0.5 -0.1 0 0.1 0.5 1 2.5 5 7.5 10 15 22];
datcom.Beta = [-5 -2.5 -0.1 0 0.1 2.5 5];
datcom.Alt = settings.z0 + (0:400:4000);

%{
configAutoMatProtub - This script sets up all the parameters for missile
                      DATCOM. All the parameters are stored in the "datcom"
                      structure.

CALLED SCRIPTS: /

CALLED FUNCTIONS: /

CALLED DATA FILES: /

REVISIONS:
- 0     27/03/2021,     release     Adriano Filippo Inno
%}

%% States
% State values in which the aerodynamic coefficients will be computed
datcom.Mach = 0.05:0.0005:1;
datcom.Alpha = [-22 -15 -10 -7.5 -5 -2.5 -1 -0.5 -0.1 0 0.1 0.5 1 2.5 5 7.5 10 15 22];
datcom.Beta = [-5 -2.5 -0.1 0 0.1 2.5 5];
datcom.Alt = settings.z0 + (0:400:4000);

%% Aerobrakes
vars.hprot = linspace(0, 0.0387, 3);                  % brakes length, first entry must be always 0!
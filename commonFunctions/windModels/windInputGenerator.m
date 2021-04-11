function [uw, vw, ww] = windInputGenerator(settings, z, uncert)
%{

windInputGenerator - This function allows to use a custom set of wind, defined in config.m 

INPUTS:
- settings,  struct(motor, CoeffsE, CoeffsF, para, ode, stoch, prob, wind),rocket data structure;
- z,         double [1, 1], local altitude;
- uncert,    double [2, 1], wind uncertanties.

OUTPUTS:
- uw,        double [1, 1], wind component along x [m/s];
- vw,        double [1, 1], wind component along y [m/s];
- ww,        double [1, 1], wind component along z [m/s];

CALLED FUNCTIONS: /

REVISION:
-#0, 13/03/2018, Release, Adriano Filippo Inno


%}
settings.wind.inputMatr = [ (settings.wind.inputGround * (1 + settings.wind.inputMult/100))
                             settings.wind.inputAzimut
                             settings.wind.inputAlt ];
                         
magn = (1 + uncert(1)/100).*settings.wind.inputMatr(1, :);
dir = mod(180 + settings.wind.inputMatr(2, :), 360);
dir = dir + uncert(2);

uw_vect = magn.*cosd(dir);
vw_vect = magn.*sind(dir);
h_vect = settings.wind.inputMatr(3, :);

h = -z;

if h < 0
    h = 0;
end

if h > h_vect(end)
    error('The current altitude of the missile is out of range of the settings.wind.inputAlt variable, fix it in config.m ')
end

uw = interp1(h_vect, uw_vect, h);
vw = interp1(h_vect, vw_vect, h);
ww = 0; 
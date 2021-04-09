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
settings.wind.input_matr = [ (settings.wind.input_ground * (1 + settings.wind.input_mult/100))
                             settings.wind.input_azimut
                             settings.wind.input_alt ];
                         
magn = (1 + uncert(1)/100).*settings.wind.input_matr(1, :);
dir = mod(180 + settings.wind.input_matr(2, :), 360);
dir = dir + uncert(2);

uw_vect = magn.*cosd(dir);
vw_vect = magn.*sind(dir);
h_vect = settings.wind.input_matr(3, :);

h = -z;

if h < 0
    h = 0;
end

if h > h_vect(end)
    error('The current altitude of the missile is out of range of the settings.wind.input_alt variable, fix it in config.m ')
end

uw = interp1(h_vect, uw_vect, h);
vw = interp1(h_vect, vw_vect, h);
ww = 0; 
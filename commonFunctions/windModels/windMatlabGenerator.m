function [uw, vw, ww] = windMatlabGenerator(settings, z, t, Hour, Day)
%{

wind_generator - Function that generates wind components in NED reference frame, based on hwm07 model

INPUT:      
- settings, struct(motor, CoeffsE, CoeffsF, para, ode, stoch, prob, wind), structure of rocket data;
- z,        double [1, 1], local altitude [m];
- t,        double [1, 1] time sample [t];
- Hour,     double [1, 1] hour of the day of the needed simulation;
- Day,      double [1, 1] day of the year of the needed simulation.

OUTPUTS:
- uw,       double [1, 1], wind component along x [m/s];
- vw,       double [1, 1], wind component along y [m/s];
- ww,       double [1, 1], wind component along z [m/s];

CALLED FUNCTIONS: /

REVISION: 
-#0, 17/01/2016, Release, Gabriele Poiana

%}

h = -z + settings.z0;
if h < 0
    h = 0;
end

if nargin == 3
    if settings.wind.HourMin == settings.wind.HourMax && settings.wind.HourMin == settings.wind.HourMax
        Day = settings.wind.DayMin;
        Hour = settings.wind.HourMin;
    end
end

Seconds = Hour*3600;

%% HORIZONTAL WIND

[uw,vw] = atmoshwm(settings.lat0,settings.lon0,h,'day',Day,...
    'seconds',Seconds+t,'model','quiet','version','14');    % NED reference
ww = settings.wind.ww;


end



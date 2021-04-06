function [apogee] = optimizationGA(x, datcom, settings)
%{
optimizationGA - cost function of the optimization

INPUTS:
- x,        double [6, 1], optimization variable, check the config for explanation;
- datcom,   struct (),     variables needed in Datcom
- settings, struct (motor, CoeffsE, CoeffsF, para, ode, stoch, prob, wind), 
                   simulation data.
OUTPUTS:
- apogee,   double [1, 1], optimization output of the cost function, [m].

CALLED FUNCTIONS: createFor006, datcomParser5, quickApogeeOnly

REVISIONS:
- 0     21/10/20,   release     Adriano Filippo Inno
%}

datcomPath = '../../commonFunctions/Datcom/';

%% RETRIVING DATCOM VARS FROM THE OPTIMIZATION VARIABLES
% x is looping
datcom.Chord1 = x(1)/100;
datcom.Chord2 = x(2)/100;
datcom.Height = x(3)/100;

if x(4) == 1
    datcom.shape = 'iso';
elseif x(4) == 2
    datcom.shape = 'rect';
elseif x(4) == 3
    datcom.shape = 'parall';
end

datcom.Lnose = x(5)/100;

if x(6) == 1
    datcom.OgType = "KARMAN";
elseif x(6) == 2
    datcom.OgType = 'HAACK';
elseif x(6) == 3
    datcom.OgType = 'OGIVE';
elseif x(6) == 4
    datcom.OgType = "POWER";
    datcom.NosePower = 1/3;
elseif x(6) == 5
    datcom.OgType = "POWER";
    datcom.NosePower = 1/2;
elseif x(6) == 6
    datcom.OgType = "POWER";
    datcom.NosePower = 3/4;
end

%% COMPUTE DIFFERENT APOGEE
% the output is the mean of different simulation cases apogee

%%% setting the aerodynamics states
settings.Alphas = datcom.Alpha;
settings.Betas = datcom.Beta;
settings.Altitudes = datcom.Alt;
settings.Machs = datcom.Mach;

%%% aerodynmics coefficient - full
xcg = settings.xcg - settings.Lnose;
datcom.xcg = xcg(1) + datcom.Lnose;
createFor006(datcom, settings, datcomPath);
[settings.CoeffsF, ~] = datcomParser();

%%% aerodynmics coefficient - empty
datcom.xcg = xcg(2) + datcom.Lnose;
createFor006(datcom, settings, datcomPath);
[settings.CoeffsE, ~] = datcomParser();

%%% case1
settings.wind.Az = (360)*pi/180;
apogee1 = quickApogeeOnly(settings);

%%% case2
settings.wind.Az = (180)*pi/180;
apogee2 = quickApogeeOnly(settings);

%%% case3
settings.wind.Mag = 1;
settings.wind.Az = (360)*pi/180;
apogee3 = quickApogeeOnly(settings);

%%% case4
settings.wind.Az = (180)*pi/180;
apogee4 = quickApogeeOnly(settings);

apogee = (- apogee1 - apogee2 - apogee3 - apogee4)/4; % apogee mean
% the sign is inverted to obtain a maxmization problem
function [apogee] = OptimizationGA(x, datcom, settings)
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

%%
settings.Alphas = datcom.Alpha;
settings.Betas = datcom.Beta;
settings.Altitudes = datcom.Alt;
settings.Machs = datcom.Mach;
%%%
xcg = datcom.xcg;
datcom.xcg = xcg(1) + datcom.Lnose;
createFor006(datcom);
[settings.CoeffsF, ~] = datcomParser5();
%%%
datcom.xcg = xcg(2) + datcom.Lnose;
createFor006(datcom);
[settings.CoeffsE, ~] = datcomParser5();

settings.wind.Az = (360)*pi/180;
apogee1 = RunSim(settings)
settings.wind.Az = (180)*pi/180;
apogee2 = RunSim(settings)

settings.wind.Mag = 1;
settings.wind.Az = (360)*pi/180;
apogee3 = RunSim(settings)
settings.wind.Az = (180)*pi/180;
apogee4 = RunSim(settings)
apogee = (- apogee1 - apogee2 - apogee3 - apogee4)/4 ;

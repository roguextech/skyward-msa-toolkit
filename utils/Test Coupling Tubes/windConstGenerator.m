function [uw, vw, ww, Az] = windConstGenerator(windData)
%{

windConstGenerator - function that generates constant wind components in NED axes

INPUTS:
- windData, struct:
            - windDAta.AzMin, Minimum angle of Azimuth from North [°];
            - windData.AzMax, Maximum angle of Azimuth from North [°];
            - windData.ElMin, Minimum angle of Elevation [°];
            - windData.ElMax, Maximum angle of Elevation [°];
            - windData.MagMin, Minimum magnitude of Wind [m/s];
            - windData.MagMax, Maximum magnitude of Wind [m/s];

OUTPUTS:
- uw,      double [1, 1], wind component along x [m/s];
- vw,      double [1, 1], wind component along y [m/s];
- ww,      double [1, 1], wind component along z [m/s];
- Az,      double [1, 1], angle of Azimuth from North [°].

CALLED FUNCTIONS: /

REVISIONS:
-#0, 25/04/2014, Release, Ruben Di Battista

%}


% extracting struct fields
if all(isfield(windData, ["AzMin", "AzMax", "MagMin", "MagMax"]))
    AzMin = windData.AzMin;
    AzMax = windData.AzMax;
    ElMin = windData.ElMin;
    ElMax = windData.ElMax;
    MagMin = windData.MagMin;
    MagMax = windData.MagMax;
    Az = AzMin + (AzMax - AzMin)*rand;
    El = ElMin + (ElMax - ElMin)*rand;
    Mag = MagMin + (MagMax - MagMin)*rand;
elseif all(isfield(windData, ["Az", "Mag"]))
    Az = windData.Az;
    Mag = windData.Mag;
    El = 0;
else
    error('constant wind not well defined')
end

R = Mag*angle2dcm(Az, El, 0, 'ZYX');
R(abs(R) < 1e-4) = 0;

uw = R(1, 1);
vw = R(1, 2);
ww = R(1, 3);

if abs(uw) < 1e-3
    uw = 0;
end

if abs(vw) < 1e-3
    vw = 0;
end

if abs(ww) < 1e-3
    ww = 0;
end

end


function [uw, vw, ww, Az] = WindConstGenerator(Az, Mag)
%{
 windgen(AzMin,AzMax,ElMin,ElMax,MagMin,MagMax)
function that generates wind components in NED axes based on altitude

Vector Orientation
AzMin = 0; Minimum angle of Azimuth from North
AzMax = 2*pi; Maximum angle of Azimuth from North
ElMin = 0; Minimum angle of Elevation
ElMax = pi/2; Maximum angle of Elevatiom

Author: Ruben Di Battista
Skyward Experimental Rocketry | CRD Dept | crd@skywarder.eu
email: ruben.dibattista@skywarder.eu
Website: http://www.skywarder.eu
April 2014; Last revision: 25.IV.2014
License:  2-clause BSD
%}

% Random Wind Vector
El = 0;
R = Mag*angle2dcm(Az, El, 0,'ZYX');
R(abs(R) < 1e-4) = 0;

uw = R(1,1);
vw = R(1,2);
ww = R(1,3);

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


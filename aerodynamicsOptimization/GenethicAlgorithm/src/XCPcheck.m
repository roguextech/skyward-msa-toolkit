function [c, ceq] = XCPcheck(x, datcom, settings)
%{
optimizationGA - cost function of the optimization

INPUTS:
- x,        double [6, 1], optimization variable, check the config for explanation;
- datcom,   struct (),     variables needed in Datcom
- settings, struct (motor, CoeffsE, CoeffsF, para, ode, stoch, prob, wind), 
                   simulation data.
OUTPUTS:
- c,        double [1, 1], non linear constrain of the optimization.

CALLED FUNCTIONS: createFor006, datcomParser, launchPadFreeDyn, windConstGenerator

REVISIONS:
- 0     21/10/20,   release     Adriano Filippo Inno
- 1     01/05/21,   update      Adriano Filippo Inno
                    improved by vectorization and removng equal cases
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

%% AERODYNAMICS COEFFICIENT - LAUNCHPAD
% this section is computed just ones, then the resulting axial coefficient 
% is keeped costant without. This consists in a small approxation on the final
% result but the computational time is reduced by 5/10%
if ~isfield(settings, 'CA0')            
    
    %%% states to compute the exit pad velocity
    datcom.Mach = 0.05;
    datcom.Alpha = [-0.1, 0, 0.1];
    datcom.Beta = 0;
    datcom.Alt = settings.z0;
    
    xcg = settings.xcg - settings.Lnose;
    datcom.xcg = xcg(1) + datcom.Lnose;
    createFor006(datcom, settings, datcomPath);
    [Coeffs0, ~] = datcomParser();
    settings.CA0 = Coeffs0.CA(2);
end

%% LAUNCHPAD DYNAMICS
%%% Initial State
X0pad = [0; 0; 0; 0];
%%% Attitude
Q0 = angleToQuat(settings.PHI, settings.OMEGA, 0*pi/180)';

[Tpad, Ypad] = ode45(@launchPadFreeDyn, [0, 10], X0pad, settings.ode.optionspad,...
    settings, Q0, settings.CA0);

%% COMPUTING THE LAUNCHPAD STABILITY DATA
% both lateral and longitudinal XCP involved
% the launchpad dynamics is used to write the aerodyanmics states
T = (288.15 - 0.0065*settings.z0);       % temperature
a = sqrt(T*1.4*287.0531);                % sound speed @launchpad
datcom.Mach = Ypad(end, 4)/a;            % Mach @launchpad
datcom.Alt = settings.z0;                % Altitude @launchpad

% Wind directions to be looped symmetrical wrt to the launch
% note: 180 deg is discarded because 0 deg is ever a worst case scenario)
Az = linspace(0, 135, 4)*pi/180;         

%%% pre-allocation
XCPlon = zeros(4, 1);
XCPlat = zeros(4, 1);
alphaExit = zeros(1, 4);
betaExit = zeros(4, 1);
for i = 1:4 
    settings.wind.Az = Az(i);            % set the direction
    [uw, vw, ww] = windConstGenerator(settings.wind);
    inertialWind = [uw, vw, ww];
    bodyWind = quatrotate(Q0', inertialWind);
    bodyVelocity = [Ypad(end, 4), 0, 0];
    Vr = bodyVelocity - bodyWind;
    ur = Vr(1); vr = Vr(2); wr = Vr(3);
    alphaExit(i) = round(atand(wr/ur), 1);
    betaExit(i) = round(atand(vr/ur), 1);
end
%%% alpha vector imposed to be symmetric and populated
alphaVectorPositive = [abs(alphaExit) 1 2.5 5 10];
alphaVectorPositive = sort(alphaVectorPositive);
datcom.Alpha = unique(sort([-alphaVectorPositive, 0, alphaVectorPositive]));
datcom.Beta = sort(betaExit);

%%% aerodynmics coefficient - full
datcom.xcg = xcg(1) + datcom.Lnose;
createFor006(datcom, settings, datcomPath);
[CoeffsF, ~] = datcomParser();

%%% aerodynmics coefficient - empty
datcom.xcg = xcg(2) + datcom.Lnose;
createFor006(datcom, settings, datcomPath);
[CoeffsE, ~] = datcomParser();

for i = 1:4
    indexAlpha = find(alphaExit(i) == datcom.Alpha);
    indexBeta = find(betaExit(i) == datcom.Beta);
    %%% longitudnal XCP
    XCPfullLon = -CoeffsF.X_C_P(indexAlpha, 1, indexBeta);
    XCPemptyLon = -CoeffsE.X_C_P(indexAlpha, 1, indexBeta);
    XCPlon(i) = Tpad(end)/settings.tb*(XCPemptyLon - XCPfullLon) + XCPfullLon;
    
    %%% lateral XCP
    if betaExit(i) ~= 0                     % if beta = 0 it's not possible to compute it
        XCPfullLat = -CoeffsF.CLN(indexAlpha, 1, indexBeta)/CoeffsF.CY(indexAlpha, 1, indexBeta);
        XCPemptyLat = -CoeffsE.CLN(indexAlpha, 1, indexBeta)/CoeffsE.CY(indexAlpha, 1, indexBeta);
        XCPlat(i) = Tpad(end)/settings.tb*(XCPemptyLat - XCPfullLat) + XCPfullLat;
    else
        XCPlat(i) = 5;
    end
end
    
XCPconstraining = min([XCPlon; XCPlat]); % taking the most constraining one

ceq = [];
c = settings.minStabilityMargin - XCPconstraining;